//! Portable, `no_std`, zero-allocation application-side BareBoot client core.
//!
//! BareBoot itself remains an independent bootloader. This crate only helps a
//! running application recognize the bus-specific "enter bootloader" triggers
//! and write the cross-reset SRAM mailbox cookie before an architecture-specific
//! reset (performed by the caller / HAL).
//!
//! Source contract: `bareBoot/docs/handsfree-app-updates.md` (authoritative),
//! with wire constants cross-checked against
//! `bareboot/crates/bareboot-protocol/src/lib.rs`.
//!
//! # Contract summary
//!
//! 1. A host emits a transport-specific trigger (sections 2 and 3).
//! 2. The application decodes it with one of the `trigger_*` helpers here.
//! 3. The application writes [`REQUEST_BOOTLOADER`] to the mailbox located at
//!    the bootloader's initial stack pointer (section 2 / 4.1) via
//!    [`write_cookie`], disables interrupts, and resets.
//! 4. After reset BareBoot inspects the mailbox and stays in its boot window.
//!
//! This crate performs no allocation, no bus I/O and no reset. It depends on
//! nothing but `core`.

#![no_std]
// SAFETY: volatile MMIO is required by the BareBoot mailbox contract. Unsafe is
// limited to `BootTarget::mailbox_addr`, `BootTarget::is_present`,
// `write_cookie` and `clear_cookie`; each block documents alignment, validity
// and lifetime assumptions. The trigger decoders and layout logic are safe.
#![allow(unsafe_code)]

#[cfg(test)]
mod tests;

// ---------------------------------------------------------------------------
// Wire constants (doc sections 2 and 3, plus bareboot-protocol).
// ---------------------------------------------------------------------------

/// Token written to the cross-reset SRAM mailbox to request bootloader entry.
///
/// Source: `docs/handsfree-app-updates.md` section 2 and `bareboot-protocol`.
pub const REQUEST_BOOTLOADER: u64 = 0x5984_E3FA_6CA1_589B;

/// Legacy Katapult/CanBoot alias of [`REQUEST_BOOTLOADER`].
///
/// Source: `docs/handsfree-app-updates.md` section 2.
pub const REQUEST_CANBOOT: u64 = REQUEST_BOOTLOADER;

/// Token the bootloader writes when handing control back to the application.
///
/// Source: `docs/handsfree-app-updates.md` section 2 and `bareboot-protocol`.
pub const REQUEST_START_APP: u64 = 0x7B06_EC45_A9A8_243D;

/// Signature identifying a Katapult/BareBoot-compatible bootloader ("CanBoot!").
///
/// Source: `docs/handsfree-app-updates.md` section 4.1 and `bareboot-protocol`.
pub const BAREBOOT_MAGIC: u64 = 0x2174_6F6F_426E_6143;

/// In-band UART byte sequence requesting serial bootloader entry.
///
/// Source: `docs/handsfree-app-updates.md` section 2 table and `bareboot-protocol`.
pub const SERIAL_BL_REQ: &[u8] = b"~ \x1c Request Serial Bootloader!! ~";

/// Single-byte I2C (and legacy I3C) target strobe for bootloader entry.
///
/// Source: `docs/handsfree-app-updates.md` section 3.1.
pub const I2C_CMD_ENTER_BOOTLOADER: u8 = 0xF0;

/// Standard CAN / CAN-FD arbitration ID for the host reboot command.
///
/// Source: `docs/handsfree-app-updates.md` section 3.3.
pub const CAN_ADMIN_REBOOT_ID: u32 = 0x3F0;

/// Reboot opcode inside the CAN admin frame (`KLIPPER_REBOOT_CMD`).
///
/// Source: `docs/handsfree-app-updates.md` section 3.3.
pub const CAN_CMD_REBOOT: u8 = 0x02;

/// 8-byte SPI sync sequence shifted in while CS_N is asserted.
///
/// Source: `docs/handsfree-app-updates.md` section 3.5.
pub const SPI_SYNC: [u8; 8] = [0x5A, 0xA5, 0xF0, 0x59, 0x84, 0xE3, 0xFA, 0x6C];

/// MIPI I3C CCC `RSTACT`, broadcast form.
///
/// Source: `docs/handsfree-app-updates.md` section 3.2.
pub const I3C_CCC_RSTACT_BCAST: u8 = 0x2A;

/// MIPI I3C CCC `RSTACT`, direct form.
///
/// Source: `docs/handsfree-app-updates.md` section 3.2.
pub const I3C_CCC_RSTACT_DIRECT: u8 = 0x9A;

/// `RSTACT` defining byte selecting "Target System Reset".
///
/// Source: `docs/handsfree-app-updates.md` section 3.2.
pub const I3C_RSTACT_TARGET_RESET: u8 = 0x02;

/// USB vendor control-request `bRequest` (`BAREBOOT_CMD`).
///
/// Source: `docs/handsfree-app-updates.md` section 3.4.3.
pub const USB_VENDOR_BREQUEST: u8 = 0xBB;

/// USB vendor control-request `wValue` (`CMD_ENTER_BOOTLOADER`).
///
/// Source: `docs/handsfree-app-updates.md` section 3.4.3.
pub const USB_CMD_ENTER_BOOTLOADER: u16 = 0x0001;

/// Value written to the BLE `OTA_CONTROL` characteristic to enter BareBoot.
///
/// Source: `docs/handsfree-app-updates.md` section 3.7.
pub const BLE_OTA_CONTROL_WRITE: u8 = 0x01;

/// `bRequest` value of a USB DFU `DFU_DETACH` request.
///
/// Source: `docs/handsfree-app-updates.md` section 3.4.2.
const USB_REQ_DFU_DETACH: u8 = 0x00;

/// Line-coding baud rate used by the CDC ACM "1200-baud touch".
///
/// Source: `docs/handsfree-app-updates.md` section 3.4.1.
const USB_1200_BAUD: u32 = 1200;

/// Lower bound of a valid initial stack pointer (doc section 4.1).
const SRAM_SP_LOW: u32 = 0x2000_0000;

/// Upper bound of a valid initial stack pointer (doc section 4.1).
const SRAM_SP_HIGH: u32 = 0x2008_0000;

// ---------------------------------------------------------------------------
// Bootloader memory layout.
// ---------------------------------------------------------------------------

/// Flash / SRAM layout of a BareBoot-capable target.
///
/// The mailbox is *not* stored here: it is derived at runtime from the
/// bootloader's initial stack pointer so that bootloader layout changes do not
/// require an application change (doc section 6).
///
/// Source: `docs/handsfree-app-updates.md` sections 2, 4.1 and 6.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BootTarget {
    /// Base address of the bootloader slot (vector table, `bl_vectors[0]`).
    pub flash_base: u32,
    /// Base address of the application slot (upper bound for the reset vector).
    pub app_base: u32,
    /// Top of SRAM, used only for the fallback mailbox location.
    pub sram_top: u32,
}

impl BootTarget {
    /// Describe a bootloader slot and its SRAM bounds.
    ///
    /// Source: `docs/handsfree-app-updates.md` sections 2 and 4.1.
    pub const fn new(flash_base: u32, app_base: u32, sram_top: u32) -> Self {
        Self {
            flash_base,
            app_base,
            sram_top,
        }
    }

    /// Address of the cross-reset mailbox: the bootloader's initial SP.
    ///
    /// Reads `flash_base[0]` (`bl_vectors[0]`) volatilely; that value is the
    /// top of the bootloader stack, and the 8-byte token lives just below it.
    ///
    /// # Safety
    ///
    /// The returned address is only meaningful when `flash_base` points at a
    /// real, mapped bootloader vector table. This crate cannot validate that;
    /// the platform supplying [`BootTarget`] must.
    ///
    /// Source: `docs/handsfree-app-updates.md` sections 2, 3.6 and 4.1.
    pub fn mailbox_addr(&self) -> usize {
        // SAFETY: `flash_base` is supplied by the platform as the bootloader
        // vector-table base. This is a single aligned 32-bit volatile read of
        // `bl_vectors[0]`, exactly as specified in doc section 2. The pointer is
        // never dereferenced normally and the read cannot be optimized away.
        unsafe { core::ptr::read_volatile(self.flash_base as *const u32) as usize }
    }

    /// Whether a Katapult/BareBoot-compatible bootloader appears to be present.
    ///
    /// Checks the two vector-table words from doc section 4.1: the initial SP
    /// must lie in `[0x20000000, 0x20080000]` and the reset vector must lie in
    /// `[flash_base, app_base)`.
    ///
    /// # Platform note
    ///
    /// This performs real volatile flash reads. On a host it must not be called
    /// with fabricated addresses; use a fake word buffer via
    /// `is_present_words` instead (that helper is covered by the unit tests).
    ///
    /// Source: `docs/handsfree-app-updates.md` section 4.1.
    pub fn is_present(&self) -> bool {
        // SAFETY: both reads are aligned 32-bit volatile reads of the first two
        // entries of the bootloader vector table at `flash_base`. The platform
        // guarantees `flash_base` is mapped flash and `flash_base + 4` is in
        // bounds. No reference is formed and no mutation occurs.
        unsafe {
            let initial_sp = core::ptr::read_volatile(self.flash_base as *const u32);
            let reset_vec = core::ptr::read_volatile((self.flash_base + 4) as *const u32);
            is_present_words(self.flash_base, self.app_base, initial_sp, reset_vec)
        }
    }

    /// Fallback mailbox address (`sram_top - 8`) for targets without a
    /// discoverable bootloader vector table, such as WCH BLE entry.
    ///
    /// Saturates at zero rather than wrapping if `sram_top < 8`.
    ///
    /// Source: `docs/handsfree-app-updates.md` sections 3.7 and 4.1.
    pub const fn fallback_mailbox_addr(&self) -> usize {
        self.sram_top.saturating_sub(8) as usize
    }
}

/// Pure presence decision shared by [`BootTarget::is_present`] and the tests.
///
/// `initial_sp` is `flash_base[0]` and `reset_vec` is `flash_base[1]` of a fake
/// or real vector table.
const fn is_present_words(flash_base: u32, app_base: u32, initial_sp: u32, reset_vec: u32) -> bool {
    let sp_ok = initial_sp >= SRAM_SP_LOW && initial_sp <= SRAM_SP_HIGH;
    let vec_ok = reset_vec >= flash_base && reset_vec < app_base;
    sp_ok && vec_ok
}

// ---------------------------------------------------------------------------
// Bus trigger decoders. All are pure functions: no MMIO, no allocation.
// ---------------------------------------------------------------------------

/// Decode the I2C target trigger: exactly one byte, `0xF0`.
///
/// A multi-byte write is normal register traffic and must not be consumed.
///
/// Source: `docs/handsfree-app-updates.md` sections 3.1 and 6.
pub fn trigger_i2c(frame: &[u8]) -> bool {
    frame.len() == 1 && frame[0] == I2C_CMD_ENTER_BOOTLOADER
}

/// Decode an I3C CCC `RSTACT` with defining byte `0x02` (target system reset).
///
/// Both the broadcast (`0x2A`) and direct (`0x9A`) CCC forms are accepted.
///
/// Source: `docs/handsfree-app-updates.md` section 3.2.
pub fn trigger_i3c_ccc(ccc: u8, defining: u8) -> bool {
    (ccc == I3C_CCC_RSTACT_BCAST || ccc == I3C_CCC_RSTACT_DIRECT)
        && defining == I3C_RSTACT_TARGET_RESET
}

/// Decode a raw I3C/I2C data byte equal to `0xF0`.
///
/// Source: `docs/handsfree-app-updates.md` sections 3.1 and 3.2.
pub fn trigger_i3c_byte(byte: u8) -> bool {
    byte == I2C_CMD_ENTER_BOOTLOADER
}

/// Decode the CAN admin reboot frame: ID `0x3F0`, `[0x02, uid[6]]`, UID match.
///
/// The payload must be exactly seven bytes.
///
/// Source: `docs/handsfree-app-updates.md` section 3.3.
pub fn trigger_can(id: u32, payload: &[u8], uid: &[u8; 6]) -> bool {
    id == CAN_ADMIN_REBOOT_ID
        && payload.len() == 7
        && payload[0] == CAN_CMD_REBOOT
        && &payload[1..7] == &uid[..]
}

/// Decode the UART in-band request as a substring of `buf`.
///
/// Accepts the magic string anywhere in the buffer so it can coexist with
/// other console traffic.
///
/// Source: `docs/handsfree-app-updates.md` sections 2 and 3.
pub fn trigger_uart(buf: &[u8]) -> bool {
    buf.windows(SERIAL_BL_REQ.len())
        .any(|window| window == SERIAL_BL_REQ)
}

/// Decode the SPI sync sequence as an exact prefix of `frame`.
///
/// Extra bytes after the 8-byte sync are allowed.
///
/// Source: `docs/handsfree-app-updates.md` section 3.5.
pub fn trigger_spi(frame: &[u8]) -> bool {
    frame.starts_with(&SPI_SYNC)
}

/// Decode a USB vendor control transfer (`bRequest = 0xBB`, `wValue = 0x0001`).
///
/// Source: `docs/handsfree-app-updates.md` section 3.4.3.
pub fn trigger_usb_vendor(brequest: u8, wvalue: u16) -> bool {
    brequest == USB_VENDOR_BREQUEST && wvalue == USB_CMD_ENTER_BOOTLOADER
}

/// Decode the USB CDC ACM 1200-baud touch.
///
/// Source: `docs/handsfree-app-updates.md` section 3.4.1.
pub fn trigger_usb_1200_baud(baud: u32) -> bool {
    baud == USB_1200_BAUD
}

/// Decode a USB DFU `DFU_DETACH` request (`bRequest = 0`).
///
/// Source: `docs/handsfree-app-updates.md` section 3.4.2.
pub fn trigger_usb_dfu_detach(brequest: u8) -> bool {
    brequest == USB_REQ_DFU_DETACH
}

/// Decode a BLE `OTA_CONTROL` write of `0x01`.
///
/// Source: `docs/handsfree-app-updates.md` section 3.7.
pub fn trigger_ble_ctrl(value: u8) -> bool {
    value == BLE_OTA_CONTROL_WRITE
}

// ---------------------------------------------------------------------------
// Mailbox cookie access. The caller owns interrupt masking and the reset.
// ---------------------------------------------------------------------------

/// Write [`REQUEST_BOOTLOADER`] to the mailbox derived from `target`.
///
/// The caller must disable interrupts, perform a system reset immediately
/// after this call, and keep the target SRAM powered through that reset. This
/// function deliberately does neither: architecture reset lives in the HAL.
///
/// Source: `docs/handsfree-app-updates.md` sections 2, 3.6 and 4.1.
///
/// # Safety
///
/// `target.mailbox_addr()` must resolve to a valid, 8-byte-aligned, preserved
/// SRAM location (the bootloader initial SP is naturally 8-byte aligned). The
/// write is volatile so it survives until the reset without being optimized
/// away; a compiler fence orders it before the caller's reset sequence.
pub fn write_cookie(target: &BootTarget) {
    // SAFETY: the platform supplied `target` and guarantees its `flash_base`
    // is a mapped bootloader vector table. The resulting mailbox is the
    // bootloader's initial SP, an aligned 64-bit SRAM slot reserved across
    // reset by the bootloader (doc section 2). A single aligned volatile write
    // is sound; no reference is formed and no aliasing invariant is broken.
    unsafe {
        let mailbox = target.mailbox_addr() as *mut u64;
        core::ptr::write_volatile(mailbox, REQUEST_BOOTLOADER);
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}

/// Clear the mailbox cookie to zero (e.g. to cancel a pending request).
///
/// Uses the same address derivation and volatility guarantees as
/// [`write_cookie`]; the caller still owns interrupt masking and reset.
///
/// Source: `docs/handsfree-app-updates.md` section 2 (bootloader clears token).
pub fn clear_cookie(target: &BootTarget) {
    // SAFETY: identical reasoning to `write_cookie`: aligned, preserved SRAM
    // slot derived from the bootloader initial SP, written exactly once via a
    // volatile 64-bit store, followed by a compiler fence.
    unsafe {
        let mailbox = target.mailbox_addr() as *mut u64;
        core::ptr::write_volatile(mailbox, 0u64);
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}
