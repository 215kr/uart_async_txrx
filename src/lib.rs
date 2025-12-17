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
use core::cell::RefCell;
use core::ffi::CStr;
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

// Atomic counters to track number of bytes in RX and bytes enqueued to TX.
static RX_COUNT: AtomicUsize = AtomicUsize::new(0);
static TX_COUNT: AtomicUsize = AtomicUsize::new(0);

/// UART driver in uninitialized state.
///
/// This type represents a UART that has been created but not yet initialized.
/// To use the UART, you must call `hw_init()` which consumes this type and
/// returns a `UartInitialized`.
///
/// # Example
///
/// ```no_run
/// let uart = Uart::new();
/// let uart = uart.hw_init(uarte0, rx_pin, tx_pin, Priority::P3, spawner);
/// uart.println("Hello, world!");
/// ```
pub struct UartUninitialized {
    _private: (),
}

/// UART driver in initialized state.
///
/// This type represents a UART that has been initialized and is ready to use.
/// All read/write operations are only available on this type, ensuring
/// compile-time safety.
///
/// # Safety
///
/// Uses `RefCell` for interior mutability, which provides runtime borrow checking.
/// This is safe in single-threaded contexts (typical for embedded systems).
pub struct UartInitialized {
    tx_prod: RefCell<heapless::spsc::Producer<'static, u8, QUEUE_CAP>>,
    rx_cons: RefCell<heapless::spsc::Consumer<'static, u8, QUEUE_CAP>>,
}

// Type alias for backward compatibility
/// Type alias for `UartUninitialized`.
///
/// For backward compatibility, `Uart` refers to the uninitialized state.
/// After calling `hw_init()`, you get a `UartInitialized`.
pub type Uart = UartUninitialized;

impl UartUninitialized {
    /// Create a new uninitialized UART instance.
    ///
    /// # Returns
    ///
    /// An uninitialized UART. You must call `hw_init()` before using it.
    pub fn new() -> Self {
        UartUninitialized { _private: () }
    }

    /// Initialize hardware and spawn internal tasks.
    ///
    /// This method consumes the uninitialized UART and returns an initialized one.
    /// The crate provides the `Irqs` binding internally; callers do not
    /// need to supply an interrupt token.
    ///
    /// # Arguments
    ///
    /// * `uarte_peri` - UARTE peripheral
    /// * `rx_pin` - RX pin
    /// * `tx_pin` - TX pin
    /// * `prio` - Interrupt priority
    /// * `spawner` - Embassy task spawner
    ///
    /// # Returns
    ///
    /// An initialized UART ready for use.
    pub fn hw_init(
        self,
        uarte_peri: Peri<'static, peripherals::UARTE0>,
        rx_pin: Peri<'static, peripherals::P0_08>,
        tx_pin: Peri<'static, peripherals::P0_06>,
        prio: interrupt::Priority,
        spawner: Spawner,
    ) -> UartInitialized {
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

        let _ = spawner.spawn(uarte_tx_task_owned(tx, tx_cons));
        rprintln!("[UARTE] TX task spawned (from hw_init)");
        let _ = spawner.spawn(uarte_rx_task_owned(rx, rx_prod));
        rprintln!("[UARTE] RX task spawned (from hw_init)");

        UartInitialized {
            tx_prod: RefCell::new(tx_prod),
            rx_cons: RefCell::new(rx_cons),
        }
    }
}

impl Default for UartUninitialized {
    fn default() -> Self {
        Self::new()
    }
}

impl UartInitialized {
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
        let mut prod = self.tx_prod.borrow_mut();
        for &b in s.to_bytes_with_nul() {
            if prod.enqueue(b).is_ok() {
                TX_COUNT.fetch_add(1, Ordering::SeqCst);
            } else {
                rprintln!("[UART] TX queue full, dropping data");
                break;
            }
        }
    }

    /// Dequeue all available bytes from the RX consumer and return them
    /// as a NUL-terminated `heapless::Vec<u8, QUEUE_CAP>`. Returns `None` if no
    /// data was available.
    pub fn try_read(&self) -> Option<Vec<u8, QUEUE_CAP>> {
        let mut buf: Vec<u8, QUEUE_CAP> = Vec::new();
        let mut popped = 0usize;

        let mut cons = self.rx_cons.borrow_mut();
        while let Some(b) = cons.dequeue() {
            if buf.push(b).is_err() {
                break;
            }
            popped += 1;
        }

        if popped == 0 {
            None
        } else {
            RX_COUNT.fetch_sub(popped, Ordering::SeqCst);
            let _ = buf.push(0);
            Some(buf)
        }
    }

    /// Non-blocking read: dequeue a single byte if available.
    pub fn read(&self) -> Option<u8> {
        self.rx_cons.borrow_mut().dequeue().map(|b| {
            RX_COUNT.fetch_sub(1, Ordering::SeqCst);
            b
        })
    }

    /// Read up to `out.len()` bytes into the provided buffer. Returns number of bytes read.
    pub fn read_into(&self, out: &mut [u8]) -> usize {
        let mut cons = self.rx_cons.borrow_mut();
        let mut i = 0;

        while i < out.len() {
            match cons.dequeue() {
                Some(b) => {
                    out[i] = b;
                    i += 1;
                }
                None => break,
            }
        }

        if i > 0 {
            RX_COUNT.fetch_sub(i, Ordering::SeqCst);
        }
        i
    }

    /// Non-blocking write: enqueue as many bytes as will fit. Returns number enqueued.
    pub fn write(&self, data: &[u8]) -> usize {
        let mut prod = self.tx_prod.borrow_mut();
        let mut written = 0;

        for &b in data {
            if prod.enqueue(b).is_err() {
                break;
            }
            written += 1;
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
        written += self.write(b"\r\n");
        written
    }

    /// Write a CStr and return number of bytes enqueued (including the trailing NUL).
    pub fn write_cstr(&self, s: &CStr) -> usize {
        let mut prod = self.tx_prod.borrow_mut();
        let mut written = 0;

        for &b in s.to_bytes_with_nul() {
            if prod.enqueue(b).is_err() {
                break;
            }
            written += 1;
        }

        if written > 0 {
            TX_COUNT.fetch_add(written, Ordering::SeqCst);
        }
        written
    }
}

pub type Driver = UartInitialized;

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
        match rx.read(&mut first).await {
            Ok(()) => {
                let mut buf = [0u8; 64];
                let mut len: usize = 0;
                buf[len] = first[0];
                len += 1;

                let mut idle_misses: u8 = 0;
                while idle_misses < 5 && len < buf.len() {
                    let mut tmp = [0u8; 1];
                    match with_timeout(Duration::from_millis(1), rx.read(&mut tmp)).await {
                        Ok(read_res) => match read_res {
                            Ok(()) => {
                                if len < buf.len() {
                                    buf[len] = tmp[0];
                                    len += 1;
                                    idle_misses = 0;
                                } else {
                                    break;
                                }
                            }
                            Err(e) => {
                                rprintln!("[UARTE-RX] read error during collect: {:?}", e);
                                break;
                            }
                        },
                        Err(_) => {
                            idle_misses += 1;
                        }
                    }
                }

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
