#![no_std]

extern crate core;

use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::peripherals;
use embassy_nrf::uarte;

// Provide the interrupt binding type for UARTE0 inside the crate so
// application `main` does not need to define `bind_interrupts!`.
bind_interrupts!(struct Irqs {
    UARTE0 => uarte::InterruptHandler<peripherals::UARTE0>;
});
use core::cell::UnsafeCell;
use core::ffi::CStr;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicUsize, Ordering};
use embassy_time::{with_timeout, Duration, Timer};
use heapless::spsc::Queue;
use heapless::Vec;
use rtt_target::rprintln;

// `build.rs` will generate `queue_cap.rs` into OUT_DIR which defines
// `pub const QUEUE_CAP: usize = ...;`. It picks value from the
// `UART_QUEUE_CAP` env var (if set) or from cargo features.
include!(concat!(env!("OUT_DIR"), "/queue_cap.rs"));
// no atomic globals needed when Uart is stored as a `'static` in `main`
use embassy_nrf::interrupt;
use embassy_nrf::interrupt::InterruptExt;
use embassy_nrf::Peri;

/// UART driver for nRF52 using Embassy async runtime.
///
/// Provides non-blocking UART communication using lock-free SPSC queues.
/// Background tasks handle the actual hardware TX/RX operations.
///
/// # Usage
///
/// ```no_run
/// let mut uart = Uart::new();
/// uart.hw_init(uarte0, rx_pin, tx_pin, Priority::P3, spawner);
///
/// // Write data
/// uart.println("Hello, world!");
/// uart.write(b"raw bytes");
///
/// // Read data
/// if let Some(byte) = uart.read() {
///     rprintln!("Received: {}", byte);
/// }
/// ```
///
/// # Safety
///
/// `hw_init()` must be called exactly once before using any other methods.
/// The internal queue endpoints use `UnsafeCell` for interior mutability,
/// which is safe under the single-initialization discipline enforced by the
/// API design.
pub struct Uart {
    // Queue endpoints are stored in UnsafeCell to allow interior mutability.
    // SAFETY: hw_init() initializes these exactly once before any other method
    // is called. After initialization, access is synchronized by the task model:
    // - tx_prod: only accessed from user code (single-threaded)
    // - rx_cons: only accessed from user code (single-threaded)
    tx_prod: UnsafeCell<MaybeUninit<heapless::spsc::Producer<'static, u8, QUEUE_CAP>>>,
    rx_cons: UnsafeCell<MaybeUninit<heapless::spsc::Consumer<'static, u8, QUEUE_CAP>>>,
}

// SAFETY: Uart is designed to be shared across tasks.
// The UnsafeCell access is guarded by:
// 1. hw_init() is called exactly once before any other method
// 2. After init, tx_prod/rx_cons are each accessed from only one logical context
// 3. The SPSC queue itself provides lock-free synchronization
unsafe impl Sync for Uart {}

struct StaticUart(UnsafeCell<MaybeUninit<Uart>>);
unsafe impl Sync for StaticUart {}
static UART_STORAGE: StaticUart = StaticUart(UnsafeCell::new(MaybeUninit::uninit()));

// Atomic counters to track number of bytes in RX and bytes enqueued to TX.
static RX_COUNT: AtomicUsize = AtomicUsize::new(0);
static TX_COUNT: AtomicUsize = AtomicUsize::new(0);

/// Initialize the global UART instance.
///
/// This should be called once at startup to create the global UART instance.
///
/// # Safety
///
/// Must be called exactly once. Calling this function multiple times will
/// result in undefined behavior.
///
/// # Returns
///
/// A static reference to the initialized UART instance.
pub fn init_global(u: Uart) -> &'static Uart {
    unsafe {
        (*UART_STORAGE.0.get()).as_mut_ptr().write(u);
        &*(*UART_STORAGE.0.get()).as_ptr()
    }
}

impl Uart {
    /// Return number of bytes currently available in RX queue.
    pub fn available(&self) -> usize {
        RX_COUNT.load(Ordering::SeqCst)
    }

    /// Return an estimate of how many bytes can be enqueued to TX.
    pub fn available_for_write(&self) -> usize {
        let cap = QUEUE_CAP;
        let used = TX_COUNT.load(Ordering::SeqCst);
        if used >= cap {
            0
        } else {
            cap - used
        }
    }

    /// Asynchronously wait until all enqueued TX bytes have been transmitted.
    pub async fn flush(&self) {
        loop {
            if TX_COUNT.load(Ordering::SeqCst) == 0 {
                break;
            }
            Timer::after(Duration::from_millis(5)).await;
        }
    }

    /// Enqueue a Rust `&str` for transmission.
    ///
    /// Converts the string to bytes and appends it to the TX queue.
    /// Note: Does not add a NUL terminator.
    ///
    /// # Arguments
    ///
    /// * `s` - The string to enqueue
    pub fn print(&self, s: &str) {
        // Convert &str to bytes and forward to `print_c` with a temporary NUL-terminated buffer.
        // Note: avoid heap allocation; use a small stack buffer up to capacity.
        // Simply write the string bytes directly - more efficient than converting to CStr
        self.write(s.as_bytes());
    }

    /// Enqueue a C-style NUL-terminated string into the TX queue.
    ///
    /// # Arguments
    ///
    /// * `s` - The C string to enqueue (including NUL terminator)
    ///
    /// # Note
    ///
    /// Stops enqueueing if the TX queue becomes full.
    pub fn print_c(&self, s: &CStr) {
        unsafe {
            // SAFETY: hw_init() must have initialized tx_prod before any public method is called.
            let prod_ptr = (*self.tx_prod.get()).as_mut_ptr();
            let prod: &mut heapless::spsc::Producer<'static, u8, QUEUE_CAP> = &mut *prod_ptr;
            for &b in s.to_bytes_with_nul() {
                if prod.enqueue(b).is_ok() {
                    TX_COUNT.fetch_add(1, Ordering::SeqCst);
                } else {
                    // Queue full - log warning
                    rprintln!("[UART] TX queue full, dropping data");
                    break;
                }
            }
        }
    }

    /// Dequeue all available bytes from the RX consumer and return them
    /// as a NUL-terminated `heapless::Vec<u8, QUEUE_CAP>`. Returns `None` if no
    /// data was available or the consumer is not initialized.
    pub fn try_read(&self) -> Option<Vec<u8, QUEUE_CAP>> {
        let mut buf: Vec<u8, QUEUE_CAP> = Vec::new();
        let mut popped = 0usize;
        unsafe {
            // SAFETY: hw_init() must have initialized rx_cons before any public method is called.
            let cons_ptr = (*self.rx_cons.get()).as_mut_ptr();
            let cons: &mut heapless::spsc::Consumer<'static, u8, QUEUE_CAP> = &mut *cons_ptr;
            while let Some(b) = cons.dequeue() {
                if buf.push(b).is_err() {
                    break;
                }
                popped += 1;
            }
        }
        if popped == 0 {
            None
        } else {
            // adjust RX counter
            RX_COUNT.fetch_sub(popped, Ordering::SeqCst);
            // Append terminating NUL if space permits
            let _ = buf.push(0);
            Some(buf)
        }
    }
}

impl Uart {
    /// Non-blocking read: dequeue a single byte if available.
    pub fn read(&self) -> Option<u8> {
        unsafe {
            let cons_ptr = (*self.rx_cons.get()).as_mut_ptr();
            let cons: &mut heapless::spsc::Consumer<'static, u8, QUEUE_CAP> = &mut *cons_ptr;
            let res = cons.dequeue();
            if res.is_some() {
                RX_COUNT.fetch_sub(1, Ordering::SeqCst);
            }
            res
        }
    }

    /// Read up to `out.len()` bytes into the provided buffer. Returns number of bytes read.
    pub fn read_into(&self, out: &mut [u8]) -> usize {
        let mut i = 0usize;
        unsafe {
            let cons_ptr = (*self.rx_cons.get()).as_mut_ptr();
            let cons: &mut heapless::spsc::Consumer<'static, u8, QUEUE_CAP> = &mut *cons_ptr;
            while i < out.len() {
                match cons.dequeue() {
                    Some(b) => {
                        out[i] = b;
                        i += 1;
                    }
                    None => break,
                }
            }
        }
        if i > 0 {
            RX_COUNT.fetch_sub(i, Ordering::SeqCst);
        }
        i
    }

    /// Non-blocking write: enqueue as many bytes as will fit. Returns number enqueued.
    pub fn write(&self, data: &[u8]) -> usize {
        let mut written = 0usize;
        unsafe {
            let prod_ptr = (*self.tx_prod.get()).as_mut_ptr();
            let prod: &mut heapless::spsc::Producer<'static, u8, QUEUE_CAP> = &mut *prod_ptr;
            for &b in data {
                if prod.enqueue(b).is_err() {
                    break;
                }
                written += 1;
            }
        }
        if written > 0 {
            TX_COUNT.fetch_add(written, Ordering::SeqCst);
        }
        written
    }

    /// Convenience: write a `&str` (as bytes). Returns bytes enqueued.
    pub fn write_str(&self, s: &str) -> usize {
        self.write(s.as_bytes())
    }

    /// Convenience: write line (adds CRLF) and return bytes enqueued.
    pub fn println(&self, s: &str) -> usize {
        let mut written = self.write_str(s);
        // CRLF
        written += self.write(b"\r\n");
        written
    }

    /// Write a CStr and return number of bytes enqueued (including the trailing NUL).
    pub fn write_cstr(&self, s: &CStr) -> usize {
        let mut written = 0usize;
        unsafe {
            let prod_ptr = (*self.tx_prod.get()).as_mut_ptr();
            let prod: &mut heapless::spsc::Producer<'static, u8, QUEUE_CAP> = &mut *prod_ptr;
            for &b in s.to_bytes_with_nul() {
                if prod.enqueue(b).is_err() {
                    break;
                }
                written += 1;
            }
        }
        if written > 0 {
            TX_COUNT.fetch_add(written, Ordering::SeqCst);
        }
        written
    }
}

impl Uart {
    pub fn new() -> Self {
        Uart {
            tx_prod: UnsafeCell::new(MaybeUninit::uninit()),
            rx_cons: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    /// Initialize hardware and spawn internal tasks.
    /// The crate provides the `Irqs` binding internally; callers do not
    /// need to supply an interrupt token.
    pub fn hw_init(
        &mut self,
        uarte_peri: Peri<'static, peripherals::UARTE0>,
        rx_pin: Peri<'static, peripherals::P0_08>,
        tx_pin: Peri<'static, peripherals::P0_06>,
        prio: interrupt::Priority,
        spawner: Spawner,
    ) {
        interrupt::UARTE0.set_priority(prio);
        rprintln!("[UARTE] NVIC priority set for UARTE0 (from hw_init)");
        let cfg = uarte::Config::default();
        let uarte = uarte::Uarte::new(uarte_peri, rx_pin, tx_pin, Irqs, cfg);
        let (tx, rx) = uarte.split();

        static mut TX_QUEUE: Queue<u8, QUEUE_CAP> = Queue::new();
        static mut RX_QUEUE: Queue<u8, QUEUE_CAP> = Queue::new();
        #[allow(static_mut_refs)]
        let (tx_prod, tx_cons) = unsafe { TX_QUEUE.split() };
        #[allow(static_mut_refs)]
        let (rx_prod, rx_cons) = unsafe { RX_QUEUE.split() };
        // store the producer and consumer into the Uart instance fields
        unsafe {
            (*self.tx_prod.get()).as_mut_ptr().write(tx_prod);
        }
        unsafe {
            (*self.rx_cons.get()).as_mut_ptr().write(rx_cons);
        }

        let _ = spawner.spawn(uarte_tx_task_owned(tx, tx_cons));
        rprintln!("[UARTE] TX task spawned (from hw_init)");
        let _ = spawner.spawn(uarte_rx_task_owned(rx, rx_prod));
        rprintln!("[UARTE] RX task spawned (from hw_init)");
    }
}

pub type Driver = Uart;

impl Default for Uart {
    fn default() -> Self {
        Uart::new()
    }
}

#[embassy_executor::task]
async fn uarte_tx_task_owned(
    mut tx: uarte::UarteTx<'static>,
    mut tx_cons: heapless::spsc::Consumer<'static, u8, QUEUE_CAP>,
) -> ! {
    rprintln!("[UARTE-TX] started (owned task)");
    loop {
        let mut buf: Vec<u8, 64> = Vec::new();
        while let Some(b) = tx_cons.dequeue() {
            if buf.push(b).is_err() {
                break;
            }
            if buf.len() >= 64 {
                break;
            }
        }
        if buf.is_empty() {
            Timer::after(Duration::from_millis(20)).await;
            continue;
        }
        // we've consumed `buf.len()` bytes from the TX queue; adjust counter
        TX_COUNT.fetch_sub(buf.len(), Ordering::SeqCst);
        match tx.write(&buf).await {
            Ok(()) => rprintln!("[UARTE-TX] sent {} bytes", buf.len()),
            Err(e) => rprintln!("[UARTE-TX] write error: {:?}", e),
        }
    }
}

#[embassy_executor::task]
async fn uarte_rx_task_owned(
    mut rx: uarte::UarteRx<'static>,
    mut rx_prod: heapless::spsc::Producer<'static, u8, QUEUE_CAP>,
) -> ! {
    rprintln!("[UARTE-RX] started (owned task)");
    loop {
        let mut first = [0u8; 1];
        // Wait for the first byte (blocking until available)
        match rx.read(&mut first).await {
            Ok(()) => {
                // We received the first byte — start a short collection window
                let mut buf = [0u8; 64];
                let mut len: usize = 0;
                buf[len] = first[0];
                len += 1;

                // Collect additional bytes until we observe ~5ms of inactivity.
                // We'll poll with 1ms sub-timeouts and reset the idle counter
                // whenever a byte arrives within that 1ms window. When we see
                // 5 consecutive 1ms periods with no input, we end collection.
                let mut idle_misses: u8 = 0;
                while idle_misses < 5 && len < buf.len() {
                    let mut tmp = [0u8; 1];
                    match with_timeout(Duration::from_millis(1), rx.read(&mut tmp)).await {
                        Ok(read_res) => {
                            match read_res {
                                Ok(()) => {
                                    if len < buf.len() {
                                        buf[len] = tmp[0];
                                        len += 1;
                                        // reset idle counter because we received data quickly
                                        idle_misses = 0;
                                    } else {
                                        // buffer full
                                        break;
                                    }
                                }
                                Err(e) => {
                                    rprintln!("[UARTE-RX] read error during collect: {:?}", e);
                                    break;
                                }
                            }
                        }
                        Err(_) => {
                            // timeout (no data for 1ms)
                            idle_misses += 1;
                        }
                    }
                }

                // Enqueue all collected bytes at once (or until the queue is full)
                let mut enqueued = 0usize;
                for i in 0..len {
                    if rx_prod.enqueue(buf[i]).is_ok() {
                        enqueued += 1;
                    } else {
                        break;
                    }
                }
                if enqueued > 0 {
                    RX_COUNT.fetch_add(enqueued, Ordering::SeqCst);
                }
                rprintln!("[UARTE-RX] collected {} bytes, enqueued {}", len, enqueued);
            }
            Err(e) => {
                rprintln!("[UARTE-RX] read error: {:?}", e);
                Timer::after(Duration::from_millis(10)).await;
            }
        }
    }
}
