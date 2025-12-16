#![no_std]

extern crate core;

use core::marker::Sync;
use core::default::Default;
use embassy_executor::Spawner;
use embassy_nrf::uarte;
use embassy_nrf::bind_interrupts;
use embassy_nrf::peripherals;

// Provide the interrupt binding type for UARTE0 inside the crate so
// application `main` does not need to define `bind_interrupts!`.
bind_interrupts!(struct Irqs {
    UARTE0 => uarte::InterruptHandler<peripherals::UARTE0>;
});
use heapless::Vec;
use core::sync::atomic::{AtomicUsize, Ordering};
use core::ffi::CStr;
use rtt_target::rprintln;
use heapless::spsc::Queue;
use core::mem::MaybeUninit;
use core::cell::UnsafeCell;
use embassy_time::{Duration, Timer, with_timeout};

// `build.rs` will generate `queue_cap.rs` into OUT_DIR which defines
// `pub const QUEUE_CAP: usize = ...;`. It picks value from the
// `UART_QUEUE_CAP` env var (if set) or from cargo features.
include!(concat!(env!("OUT_DIR"), "/queue_cap.rs"));
// no atomic globals needed when Uart is stored as a `'static` in `main`
use embassy_nrf::interrupt;
use embassy_nrf::Peri;
use embassy_nrf::interrupt::InterruptExt;

pub struct Uart {
    // No queue endpoints stored here; hw_init will split queues and
    // spawn tasks that take ownership of the endpoints. Uart itself
    // only carries hardware handles if needed (none required currently).
    tx_prod: UnsafeCell<MaybeUninit<heapless::spsc::Producer<'static, u8, QUEUE_CAP>>>,
    rx_cons: UnsafeCell<MaybeUninit<heapless::spsc::Consumer<'static, u8, QUEUE_CAP>>>,
}
// `Uart` will be stored as a `'static` reference and shared across tasks.
// The `UnsafeCell` interior access is guarded by the single-init / usage
// discipline enforced by `hw_init` + spawn ordering. Marking `Uart` as
// `Sync` is unsafe but acceptable under these usage constraints.
unsafe impl Sync for Uart {}

struct StaticUart(UnsafeCell<MaybeUninit<Uart>>);
unsafe impl Sync for StaticUart {}
static UART_STORAGE: StaticUart = StaticUart(UnsafeCell::new(MaybeUninit::uninit()));

// Atomic counters to track number of bytes in RX and bytes enqueued to TX.
static RX_COUNT: AtomicUsize = AtomicUsize::new(0);
static TX_COUNT: AtomicUsize = AtomicUsize::new(0);

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
        if used >= cap { 0 } else { cap - used }
    }

    /// Asynchronously wait until all enqueued TX bytes have been transmitted.
    pub async fn flush(&self) {
        loop {
            if TX_COUNT.load(Ordering::SeqCst) == 0 { break; }
            Timer::after(Duration::from_millis(5)).await;
        }
    }

    /// Enqueue a Rust `&str` as a NUL-terminated C-string.
    pub fn print(&self, s: &str) {
        // Convert &str to bytes and forward to `print_c` with a temporary NUL-terminated buffer.
        // Note: avoid heap allocation; use a small stack buffer up to capacity.
        let mut buf: Vec<u8, 129> = Vec::new();
        for &b in s.as_bytes() {
            let _ = buf.push(b);
        }
        // append nul
        let _ = buf.push(0);
        // Create CStr from bytes-with-nul; safe because we ensured trailing nul
        if let Ok(cstr) = CStr::from_bytes_with_nul(buf.as_slice()) {
            self.print_c(cstr);
        }
    }

    /// Enqueue a C-style NUL-terminated string into the TX queue.
    pub fn print_c(&self, s: &CStr) {
        unsafe {
            // SAFETY: `hw_init` must have initialized `self.tx_prod` before
            // `print_c` is ever called. Read the producer from the instance and enqueue bytes.
            let prod_ptr = (*self.tx_prod.get()).as_mut_ptr();
            let prod: &mut heapless::spsc::Producer<'static, u8, QUEUE_CAP> = &mut *prod_ptr;
            for &b in s.to_bytes_with_nul() {
                if prod.enqueue(b).is_ok() {
                    TX_COUNT.fetch_add(1, Ordering::SeqCst);
                } else {
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
                    Some(b) => { out[i] = b; i += 1; }
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
                if prod.enqueue(b).is_err() { break; }
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
                if prod.enqueue(b).is_err() { break; }
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
        Uart { tx_prod: UnsafeCell::new(MaybeUninit::uninit()), rx_cons: UnsafeCell::new(MaybeUninit::uninit()) }
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
        unsafe { (*self.tx_prod.get()).as_mut_ptr().write(tx_prod); }
        unsafe { (*self.rx_cons.get()).as_mut_ptr().write(rx_cons); }

        let _ = spawner.spawn(uarte_tx_task_owned(tx, tx_cons));
        rprintln!("[UARTE] TX task spawned (from hw_init)");
        let _ = spawner.spawn(uarte_rx_task_owned(rx, rx_prod));
        rprintln!("[UARTE] RX task spawned (from hw_init)");
    }
}

pub type Driver = Uart;

impl Uart {
    #[allow(dead_code)]
    pub fn spawn_tasks(&mut self, tx: uarte::UarteTx<'static>, rx: uarte::UarteRx<'static>, spawner: Spawner) {
        static mut TX_QUEUE: Queue<u8, QUEUE_CAP> = Queue::new();
        static mut RX_QUEUE: Queue<u8, QUEUE_CAP> = Queue::new();
        #[allow(static_mut_refs)]
        let (tx_prod, tx_cons) = unsafe { TX_QUEUE.split() };
        #[allow(static_mut_refs)]
        let (rx_prod, rx_cons) = unsafe { RX_QUEUE.split() };
        unsafe { (*self.tx_prod.get()).as_mut_ptr().write(tx_prod); }
        unsafe { (*self.rx_cons.get()).as_mut_ptr().write(rx_cons); }
        let _ = spawner.spawn(uarte_tx_task_owned(tx, tx_cons));
        rprintln!("[UARTE] TX task spawned (via uart module)");
        let _ = spawner.spawn(uarte_rx_task_owned(rx, rx_prod));
        rprintln!("[UARTE] RX task spawned (via uart module)");
    }

    #[allow(dead_code)]
    pub fn write_buf(&self, buf: &Vec<u8, 64>) {
        let _ = buf;
    }
}

impl Default for Uart {
    fn default() -> Self {
        Uart::new()
    }
}

#[embassy_executor::task]
async fn uarte_tx_task_owned(mut tx: uarte::UarteTx<'static>, mut tx_cons: heapless::spsc::Consumer<'static, u8, QUEUE_CAP>) -> ! {
    rprintln!("[UARTE-TX] started (owned task)");
    loop {
        let mut buf: Vec<u8, 64> = Vec::new();
        while let Some(b) = tx_cons.dequeue() {
            if buf.push(b).is_err() { break; }
            if buf.len() >= 64 { break; }
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
async fn uarte_rx_task_owned(mut rx: uarte::UarteRx<'static>, mut rx_prod: heapless::spsc::Producer<'static, u8, QUEUE_CAP>) -> ! {
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
