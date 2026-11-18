//! # Descriptor-driven register-map server core (`RegMap`)
//!
//! Mirrors the Miniata control-table semantics
//! (`ref/Miniata/firmware/src/control_table.h` / `control_table.cpp`) as a portable,
//! `#![no_std]`, zero-allocation framework:
//!
//! - every register is a descriptor: byte [`offset`][RegEntry::offset], byte
//!   [`width`][RegEntry::width], [`access`][RegEntry::access] mode ([`AccessMode::Ro`]
//!   or [`AccessMode::Rw`]), and an optional [`hook`][RegEntry::hook] fired after all
//!   bytes of that register have arrived,
//! - storage is a bounded, fixed-size byte bank (`[u8; CAP]`, no heap); unmapped byte
//!   offsets *inside* the declared map extent are harmless padding (upstream
//!   `ControlTableData` reserved bytes),
//! - writes are width/extent/access validated as one bus phase *before* any byte is
//!   applied (upstream `writeRegister()` `reg + len > CONTROL_TABLE_SIZE` check, tightened),
//! - partial multi-byte writes buffer byte-by-byte in the register's storage and the
//!   register is only "applied" (hook fired) once its full width has arrived,
//! - reads stream with auto-increment, little-endian, from the latched register pointer
//!   (upstream `comm_i2c.cpp` register-pointer idiom, minus the `0xFF` burst protocol,
//!   which is intentionally not ported — see `design.md` D3).
//!
//! ## Wire protocol (design D3)
//!
//! ```text
//! write phase: [ptr]              latches the register pointer (no state modification)
//!              [ptr, d0..dn]      latches, then writes n LE bytes starting at ptr
//! read phase:  N bytes from latched ptr, LE, auto-increment across registers
//! ```
//!
//! The pointer auto-advances by every processed byte (write payload and read burst
//! alike) — a plain memory-map idiom, no per-register framing on the wire.
//!
//! ## Concurrency contract (design D4)
//!
//! The core is single-task-only with no internal synchronization: bus IRQs buffer bytes
//! and set an event flag; one task (e.g. the motor loop) drains events into
//! [`RegMap::write_phase`] / [`RegMap::read_phase`] under `critical_section::with`.
//! The table is never touched from ISR context. Write hooks run synchronously within
//! the drain path — they must be short and non-blocking and must not re-enter the map
//! (structurally guaranteed: hooks receive only the application handler context
//! `&mut C` and a shared value slice `&[u8]`; no map/bank reference is aliased in).


/// Register access mode, mirroring the RO/RW distinction of the Miniata control-table
/// model (`ref/Miniata/firmware/src/control_table.h`: telemetry registers
/// `REG_PRESENT_*` are host-read-only; command/target registers are RW).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccessMode {
    /// Read-only register: bus write phases targeting it are rejected without any
    /// state change. Application-side updates via [`RegMap::set`] are still allowed
    /// (telemetry publisher path, upstream `data.present_* = ...` field updates).
    Ro,
    /// Read-write register: plain memory-map semantics with optional write hook.
    Rw,
}

/// Outcome of a completed write hook. Self-clearing command semantics are expressible
/// here: the Miniata `REG_SYSTEM_CMD` register self-clears after the command is
/// dispatched (upstream `ControlTable::checkAndClearCommand()` clears `system_cmd` to
/// `CMD_NONE` once the pending command has been consumed, `control_table.cpp`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HookResult {
    /// Hook accepted the write; the register keeps the value as written.
    Applied,
    /// Hook accepted the command and the register self-clears to zero after the hook
    /// runs (upstream `checkAndClearCommand()` semantics).
    SelfClear,
}

/// Write-hook signature. A plain `fn` pointer into the application handler context —
/// no closures and no heap allocation.
///
/// The hook must **not** re-enter the register map: it only receives a mutable borrow
/// of the application handler context and a shared little-endian view of the freshly
/// written register bytes. It runs synchronously in the single drain task (design D4)
/// and must not block.
pub type WriteHookFn<C> = fn(&mut C, u8, &[u8]) -> HookResult;

/// One register descriptor. Declaratively declared by application register profiles
/// via [`RegEntry::rw`] / [`RegEntry::ro`] / [`RegEntry::rw_hooked`] and
/// [`RegMap::add_register`].
///
/// Overlapping entries are rejected by [`RegMap::add_register`]. `RegEntry` is a
/// plain-value type (`Copy`/`Clone` regardless of the handler context type `C`, since
/// it only holds a function pointer and scalars).
pub struct RegEntry<C> {
    /// Byte offset of the register start inside the map bank.
    pub offset: u8,
    /// Byte width of the register (`1` for byte registers like `REG_SYSTEM_CMD`,
    /// `4` for little-endian `f32` registers like `REG_TARGET_VEL`).
    pub width: u8,
    /// Access mode for bus-originated writes.
    pub access: AccessMode,
    /// Optional write hook, invoked exactly once after all bytes of the register have
    /// arrived on the bus.
    pub hook: Option<WriteHookFn<C>>,
}

impl<C> RegEntry<C> {
    /// Read-write register descriptor without a hook.
    pub const fn rw(offset: u8, width: u8) -> Self {
        Self {
            offset,
            width,
            access: AccessMode::Rw,
            hook: None,
        }
    }

    /// Read-only register descriptor without a hook.
    pub const fn ro(offset: u8, width: u8) -> Self {
        Self {
            offset,
            width,
            access: AccessMode::Ro,
            hook: None,
        }
    }

    /// Read-write register descriptor with a write hook (command register, e.g.
    /// upstream `REG_SYSTEM_CMD` semantics via [`HookResult::SelfClear`]).
    pub const fn rw_hooked(offset: u8, width: u8, hook: WriteHookFn<C>) -> Self {
        Self {
            offset,
            width,
            access: AccessMode::Rw,
            hook: Some(hook),
        }
    }
}

/// `RegEntry` does not contain any `C` value (only the hook function pointer), so it
/// is `Copy`/`Clone` for *all* context types — a manual impl instead of a derive,
/// which would wrongly require `C: Copy`.
impl<C> Clone for RegEntry<C> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<C> Copy for RegEntry<C> {}

/// Register-map / bus-phase error.
///
/// Upstream Reference: `ref/Miniata/firmware/src/control_table.cpp` (`writeRegister()` /
/// `readRegister()` `false` returns) + `comm_i2c.cpp` register validity checks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// A pointer latch or byte range reached outside the declared map extent. The
    /// operation is rejected without touching any register and without moving the
    /// pointer (spec `multibus-server`: *Register bounds rejection*).
    Unmapped,
    /// A bus write-phase payload would modify one or more bytes inside a register
    /// marked [`AccessMode::Ro`]. The whole phase is rejected with no state change,
    /// matching upstream `control_table.cpp` `writeRegister()` semantics.
    ReadOnly,
    /// A direct register-oriented access used a byte count that does not match the
    /// register's declared width (upstream `writeRegister()` width validation analog).
    WidthMismatch,
    /// The read-side `out` buffer is smaller than the requested byte count.
    BufferTooSmall,
    /// Map construction error: zero-width entry, entry exceeding the bank capacity,
    /// byte range overlapping another register, or descriptor capacity exhausted.
    Descriptor,
}

/// Descriptor-driven register-map server core.
///
/// - `C` — application handler context passed to write hooks (no closures, no heap),
/// - `CAP` — bank (map) capacity in bytes; a plain stack array, no heap,
/// - `MAX_ENTRIES` — maximum number of register descriptors.
///
/// Valid byte range ("declared map") is `[0, extent)` where `extent` is the largest
/// register end among the descriptors. Unmapped byte offsets inside the extent are
/// harmless padding, readable and writable as plain bytes with no hook (upstream
/// `ControlTableData` reserved bytes).
///
/// Upstream Reference: `ref/Miniata/firmware/src/control_table.{h,cpp}`,
/// `ref/Miniata/firmware/src/comm_i2c.cpp` (register-pointer idiom).
pub struct RegMap<C, const CAP: usize, const MAX_ENTRIES: usize> {
    /// Bounded byte bank; no heap. Little-endian on the wire ("Lehmer float
    /// little-endian wire format" upstream).
    bank: [u8; CAP],
    /// Declared map extent in bytes (max register end).
    extent: usize,
    /// Register descriptors (order arbitrary, non-overlapping).
    entries: [Option<RegEntry<C>>; MAX_ENTRIES],
    /// Number of live descriptors.
    entry_count: usize,
    /// Per-register accumulator of consecutively written bytes; a register "applies"
    /// (hook fires) when its declared width is reached (spec: *Width-checked partial
    /// writes*).
    received: [u8; MAX_ENTRIES],
    /// Latched register pointer (auto-incrementing across read/write phases).
    ptr: u8,
    /// Application handler context handed to write hooks.
    ctx: C,
}

impl<C, const CAP: usize, const MAX_ENTRIES: usize> RegMap<C, CAP, MAX_ENTRIES> {
    /// Creates an empty map. Register profiles are then declared via
    /// [`RegEntry`] + [`RegMap::register`] / [`RegMap::add_register`].
    pub const fn new(ctx: C) -> Self {
        Self {
            bank: [0u8; CAP],
            extent: 0,
            entries: [None; MAX_ENTRIES],
            entry_count: 0,
            received: [0u8; MAX_ENTRIES],
            ptr: 0,
            ctx,
        }
    }

    /// Consuming builder-style descriptor addition; chains [`RegMap::add_register`].
    ///
    /// ```ignore
    /// let map = RegMap::new(ctx)
    ///     .register(RegEntry::rw(0x14, 4))?
    ///     .register(RegEntry::rw_hooked(0x50, 1, sys_cmd))?;
    /// ```
    pub fn register(mut self, entry: RegEntry<C>) -> Result<Self, Error> {
        self.add_register(entry)?;
        Ok(self)
    }

    /// Adds a register descriptor. Validates the descriptor: non-zero width, within
    /// bank capacity, no overlap with existing registers; extends the declared map
    /// extent to cover the new register.
    ///
    /// Upstream Reference: `ref/Miniata/firmware/src/control_table.h`
    /// (`CONTROL_TABLE_SIZE` extent).
    pub fn add_register(&mut self, entry: RegEntry<C>) -> Result<(), Error> {
        if entry.width == 0 {
            return Err(Error::Descriptor);
        }
        let start = usize::from(entry.offset);
        let end = start + usize::from(entry.width);
        if end > CAP {
            return Err(Error::Descriptor);
        }
        for existing in self.iter_entries() {
            let s = usize::from(existing.offset);
            let e = s + usize::from(existing.width);
            if start < e && end > s {
                return Err(Error::Descriptor);
            }
        }
        if self.entry_count >= MAX_ENTRIES {
            return Err(Error::Descriptor);
        }
        self.entries[self.entry_count] = Some(entry);
        self.entry_count += 1;
        if end > self.extent {
            self.extent = end;
        }
        Ok(())
    }

    /// Declared map extent in bytes.
    pub fn extent(&self) -> usize {
        self.extent
    }

    /// Latched register pointer (diagnostics / glue test support; design D3 pointer
    /// latch + auto-increment idiom from `comm_i2c.cpp`).
    pub fn pointer(&self) -> u8 {
        self.ptr
    }

    /// Mutable access to the application handler context between bus events.
    pub fn handler(&mut self) -> &mut C {
        &mut self.ctx
    }

    /// Bus write-phase entry point (design D3): `data` is the raw write-phase frame —
    /// first byte is the register pointer, remaining bytes the little-endian payload.
    /// An empty frame is a no-op. A frame consisting of only the pointer byte latches
    /// the pointer without modifying anything.
    ///
    /// Validation happens for the *whole phase* before any byte is applied:
    /// pointer in range (else [`Error::Unmapped`]), payload range within extent
    /// (else [`Error::Unmapped`]), no [`AccessMode::Ro`] register touched on the
    /// wire (else [`Error::ReadOnly`]; note a Ro *pointer latch without payload* is
    /// legal — that is exactly how hosts read Ro telemetry). On any error no
    /// register byte moves and the pointer is left untouched.
    ///
    /// Payload bytes are stored little-endian at the auto-incrementing pointer;
    /// spanning register boundaries is a plain memory-map write. When a register's
    /// full width has arrived (across one or several phases) its write hook runs
    /// exactly once, then the per-register accumulator resets.
    ///
    /// Upstream Reference: `ref/Miniata/firmware/src/control_table.cpp`
    /// `writeRegister()`, `ref/Miniata/firmware/src/comm_i2c.cpp` register-pointer
    /// idiom (the `0xFF` burst protocol is intentionally not ported, design D3).
    pub fn write_phase(&mut self, data: &[u8]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }
        let start = usize::from(data[0]);
        if start >= self.extent {
            return Err(Error::Unmapped);
        }
        if data.len() == 1 {
            // Pointer-only phase: latch, no state modification (design D3).
            self.ptr = data[0];
            return Ok(());
        }
        let payload = &data[1..];
        let end = start + payload.len();
        if end > self.extent {
            return Err(Error::Unmapped);
        }
        // Whole-phase read-only pre-check: any Ro byte inside the touched range
        // rejects the entire phase, leaving all registers untouched
        // (spec: *Read-only register write rejection*).
        for entry in self.iter_entries() {
            if entry.access == AccessMode::Ro {
                let s = usize::from(entry.offset);
                let e = s + usize::from(entry.width);
                if start < e && end > s {
                    return Err(Error::ReadOnly);
                }
            }
        }
        // Latch the pointer and apply the payload with auto-increment.
        self.ptr = end as u8;
        self.bank[start..end].copy_from_slice(payload);
        // Per-register bookkeeping and hook dispatch.
        for i in 0..self.entry_count {
            let entry = match self.entries[i] {
                Some(e) => e,
                None => continue,
            };
            let s = usize::from(entry.offset);
            let w = usize::from(entry.width);
            if start >= s + w || end <= s {
                continue;
            }
            // A payload covering the register from its start resets the accumulator
            // (fresh write); a payload starting mid-register continues the buffered
            // partial write of a previous phase.
            if start == s {
                self.received[i] = 0;
            }
            self.received[i] += (end.min(s + w) - start.max(s)) as u8;
            if self.received[i] >= w as u8 {
                // Register complete: apply exactly once, then reset.
                self.received[i] = 0;
                if let Some(hook) = entry.hook {
                    // Disjoint field borrows; the hook cannot re-enter the map.
                    let result = hook(&mut self.ctx, entry.offset, &self.bank[s..s + w]);
                    match result {
                        HookResult::Applied => {}
                        // Upstream ControlTable::checkAndClearCommand() self-clear.
                        HookResult::SelfClear => self.bank[s..s + w].fill(0),
                    }
                }
            }
        }
        Ok(())
    }

    /// Bus read-phase entry point (design D3): streams exactly `len` little-endian
    /// bytes from the latched pointer with auto-increment into `out`, crossing
    /// register boundaries as a plain memory-map read, and leaves the pointer after
    /// the last streamed byte. `out.len() >= len` required. The full request must fit
    /// inside the declared extent (else [`Error::Unmapped`]; nothing is streamed and
    /// the pointer does not move).
    ///
    /// Upstream Reference: `ref/Miniata/firmware/src/comm_i2c.cpp` (pointer register
    /// read; spec scenario *Pointer latch then read*: `[0x20]` + 16-byte read leaves
    /// the pointer at `0x30`).
    pub fn read_phase(&mut self, len: usize, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < len {
            return Err(Error::BufferTooSmall);
        }
        if len == 0 {
            return Ok(0);
        }
        let ptr = usize::from(self.ptr);
        if ptr + len > self.extent {
            return Err(Error::Unmapped);
        }
        out[..len].copy_from_slice(&self.bank[ptr..ptr + len]);
        self.ptr = (ptr + len) as u8;
        Ok(len)
    }

    /// Application-side register write (whole registers only, byte width checked).
    /// Bypasses [`AccessMode::Ro`] so the single motor-loop task can publish
    /// telemetry into read-only registers (upstream `data.present_* = ...` updates),
    /// and resets the per-register bus-side partial-write accumulator.
    ///
    /// Upstream Reference: `ref/Miniata/firmware/src/control_table.cpp`
    /// `writeRegister()` (width-validated analog).
    pub fn set(&mut self, offset: u8, bytes: &[u8]) -> Result<(), Error> {
        let (idx, entry) = self.find_entry(offset).ok_or(Error::Unmapped)?;
        let w = usize::from(entry.width);
        if bytes.len() != w {
            return Err(Error::WidthMismatch);
        }
        self.bank[usize::from(entry.offset)..usize::from(entry.offset) + w]
            .copy_from_slice(bytes);
        self.received[idx] = 0;
        Ok(())
    }

    /// Application-side register read (whole registers only, byte width bounded).
    /// `out.len() >= register width` required. Write hooks have no effect on reads.
    ///
    /// Upstream Reference: `ref/Miniata/firmware/src/control_table.cpp`
    /// `readRegister()`.
    pub fn get(&self, offset: u8, out: &mut [u8]) -> Result<usize, Error> {
        let (_, entry) = self.find_entry(offset).ok_or(Error::Unmapped)?;
        let w = usize::from(entry.width);
        if out.len() < w {
            return Err(Error::BufferTooSmall);
        }
        let start = usize::from(entry.offset);
        out[..w].copy_from_slice(&self.bank[start..start + w]);
        Ok(w)
    }

    fn find_entry(&self, offset: u8) -> Option<(usize, RegEntry<C>)> {
        for i in 0..self.entry_count {
            if let Some(entry) = self.entries[i] {
                if entry.offset == offset {
                    return Some((i, entry));
                }
            }
        }
        None
    }

    fn iter_entries(&self) -> impl Iterator<Item = RegEntry<C>> + '_ {
        self.entries[..self.entry_count].iter().filter_map(|e| *e)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test handler context collecting hook invocations (std test code; no heap in the
    /// library itself — a plain fixed-size log keeps the test no_std-clean too).
    #[derive(Default)]
    struct Handler {
        log: heapless_test::Log,
    }

    // Minimal fixed-size logging helper to avoid introducing a heap into tests.
    mod heapless_test {
        #[derive(Default)]
        pub struct Log {
            pub items: [u8; 16],
            pub len: usize,
        }
        impl Log {
            pub fn push(&mut self, v: u8) {
                if self.len < 16 {
                    self.items[self.len] = v;
                    self.len += 1;
                }
            }
        }
    }

    fn sys_cmd_hook(ctx: &mut Handler, _offset: u8, value: &[u8]) -> HookResult {
        ctx.log.push(value[0]);
        HookResult::SelfClear
    }

    fn vel_hook(ctx: &mut Handler, _offset: u8, _value: &[u8]) -> HookResult {
        ctx.log.push(0xEE);
        HookResult::Applied
    }

    /// Miniata-shaped test table: byte regs 0x00..., f32 targets, Ro telemetry block
    /// 0x20..0x30, tuning 0x14/0x3C..., REG_SYSTEM_CMD 0x50 (upstream control_table.h).
    type MiniMap = RegMap<Handler, 96, 9>;

    fn mini_map() -> MiniMap {
        MiniMap::new(Handler::default())
            .register(RegEntry::rw(0x00, 1))              // REG_DEVICE_ID byte
            .unwrap()
            .register(RegEntry::rw(0x14, 4))              // REG_TARGET_VEL f32 LE
            .unwrap()
            .register(RegEntry::ro(0x20, 4))              // REG_PRESENT_OUT_POS f32 LE
            .unwrap()
            .register(RegEntry::ro(0x24, 4))              // REG_PRESENT_OUT_VEL f32 LE
            .unwrap()
            .register(RegEntry::ro(0x28, 4))              // REG_PRESENT_TORQUE_IQ f32 LE
            .unwrap()
            .register(RegEntry::ro(0x2C, 4))              // REG_PRESENT_MOT_POS f32 LE
            .unwrap()
            .register(RegEntry::rw(0x3C, 4))              // REG_POS_KP f32 LE
            .unwrap()
            .register(RegEntry::rw_hooked(0x50, 1, sys_cmd_hook)) // REG_SYSTEM_CMD byte
            .unwrap()
    }

    fn le_f32(v: f32) -> [u8; 4] {
        v.to_le_bytes()
    }

    #[test]
    fn descriptors_reject_overlap_zero_width_and_capacity() {
        let mut map = MiniMap::new(Handler::default());
        map.add_register(RegEntry::rw(0x10, 4)).unwrap();
        // Overlap: 0x12..0x14 hits the register at 0x10..0x14.
        assert_eq!(map.add_register(RegEntry::ro(0x12, 2)), Err(Error::Descriptor));
        // Zero width is meaningless.
        assert_eq!(map.add_register(RegEntry::rw(0x20, 0)), Err(Error::Descriptor));
        // Exceeds bank capacity.
        assert_eq!(map.add_register(RegEntry::rw(0x50, 100)), Err(Error::Descriptor));
        // Non-overlapping addition still works afterwards.
        map.add_register(RegEntry::ro(0x1C, 4)).unwrap();
        assert_eq!(map.extent(), 0x20);
    }

    #[test]
    fn unmapped_pointer_latch_is_rejected_without_state_change() {
        let mut map = mini_map(); // extent = 0x51
        let before = map.pointer();
        assert_eq!(map.write_phase(&[0xFE]), Err(Error::Unmapped));
        assert_eq!(map.pointer(), before);
        assert_eq!(map.write_phase(&[0x54]), Err(Error::Unmapped)); // beyond extent
        assert_eq!(map.pointer(), before);
    }

    #[test]
    fn unmapped_write_range_is_rejected_without_touching_any_register() {
        let mut map = mini_map();
        map.set(0x3C, &le_f32(1.5)).unwrap();
        // Payload [0x50 + 3 bytes] would run off the extent (0x51).
        assert_eq!(map.write_phase(&[0x4E, 0, 0, 0, 0]), Err(Error::Unmapped));
        let mut out = [0u8; 4];
        map.get(0x3C, &mut out).unwrap();
        assert_eq!(out, le_f32(1.5), "no byte may move on a rejected phase");
        // Pointer untouched as well.
        assert_eq!(map.pointer(), 0);
    }

    #[test]
    fn unmapped_read_is_rejected_and_pointer_unchanged() {
        let mut map = mini_map();
        // Latch 0x4E then request 8 bytes: 0x4E + 8 > extent 0x51.
        map.write_phase(&[0x4E]).unwrap();
        let mut out = [0u8; 8];
        assert_eq!(map.read_phase(8, &mut out), Err(Error::Unmapped));
        assert_eq!(map.pointer(), 0x4E, "rejected reads must not move the pointer");
        // Reads may land exactly on the extent end.
        assert_eq!(map.read_phase(3, &mut out).unwrap(), 3);
        assert_eq!(map.pointer(), 0x51);
    }

    #[test]
    fn pointer_latch_then_read_autoincrements_left_at_reg_end() {
        let mut map = mini_map();
        // 16-byte burst-telemetry block 0x20..0x30 (upstream comm_i2c.cpp scenario).
        map.set(0x20, &le_f32(1.0)).unwrap();
        map.set(0x24, &le_f32(2.0)).unwrap();
        map.set(0x28, &le_f32(3.0)).unwrap();
        map.set(0x2C, &le_f32(4.0)).unwrap();
        map.write_phase(&[0x20]).unwrap(); // pointer latch, no state modification
        assert_eq!(map.pointer(), 0x20);
        let mut out = [0u8; 16];
        assert_eq!(map.read_phase(16, &mut out).unwrap(), 16);
        // Continuous little-endian byte stream, no per-register framing.
        assert_eq!(
            &out,
            &[
                1.0f32.to_le_bytes()[0], 1.0f32.to_le_bytes()[1], 1.0f32.to_le_bytes()[2], 1.0f32.to_le_bytes()[3],
                2.0f32.to_le_bytes()[0], 2.0f32.to_le_bytes()[1], 2.0f32.to_le_bytes()[2], 2.0f32.to_le_bytes()[3],
                3.0f32.to_le_bytes()[0], 3.0f32.to_le_bytes()[1], 3.0f32.to_le_bytes()[2], 3.0f32.to_le_bytes()[3],
                4.0f32.to_le_bytes()[0], 4.0f32.to_le_bytes()[1], 4.0f32.to_le_bytes()[2], 4.0f32.to_le_bytes()[3],
            ]
        );
        // The pointer is left at 0x30 (spec: *Pointer latch then read*).
        assert_eq!(map.pointer(), 0x30);
    }

    #[test]
    fn le_float_roundtrip_write_then_read() {
        let mut map = mini_map();
        // Bus write phase: [0x14, 12.5f32 LE] (spec: *Pointer latch then write*).
        map.write_phase(&[0x14]).unwrap();
        let mut out = [0u8; 4];
        assert_eq!(map.read_phase(4, &mut out).unwrap(), 4);
        // Fresh float write through the bus path.
        map.write_phase(&[0x14, le_f32(12.5)[0], le_f32(12.5)[1], le_f32(12.5)[2], le_f32(12.5)[3]])
            .unwrap();
        map.get(0x14, &mut out).unwrap();
        assert_eq!(f32::from_le_bytes(out), 12.5);
        // Read back through the wire path.
        map.write_phase(&[0x14]).unwrap();
        let mut wire = [0u8; 4];
        map.read_phase(4, &mut wire).unwrap();
        assert_eq!(f32::from_le_bytes(wire), 12.5);
        // And through the application path as well.
        map.set(0x3C, &le_f32(5.0)).unwrap(); // REG_POS_KP default (control_table.cpp)
        map.get(0x3C, &mut out).unwrap();
        assert_eq!(f32::from_le_bytes(out), 5.0);
    }

    #[test]
    fn partial_multi_byte_write_buffers_then_completes_hook_once() {
        let mut map: MiniMap = MiniMap::new(Handler::default())
            .register(RegEntry::rw_hooked(0x14, 4, vel_hook))
            .unwrap();
        let b = le_f32(3.25);
        // Fewer bytes than the declared width: buffered in the register's storage,
        // register not applied yet (no hook).
        map.write_phase(&[0x14, b[0], b[1]]).unwrap();
        assert!(map.handler().log.len == 0, "hook must not fire before completion");
        // Completion phase starting mid-register (continuation).
        map.write_phase(&[0x16, b[2], b[3]]).unwrap();
        assert_eq!(map.handler().log.len, 1, "hook fires exactly once on completion");
        assert_eq!(map.handler().log.items[0], 0xEE);
        let mut out = [0u8; 4];
        map.get(0x14, &mut out).unwrap();
        assert_eq!(f32::from_le_bytes(out), 3.25);
    }

    #[test]
    fn payload_spanning_register_boundary_writes_both_and_fires_both_hooks() {
        let mut map: MiniMap = MiniMap::new(Handler::default())
            .register(RegEntry::rw_hooked(0x20, 4, vel_hook))
            .unwrap()
            .register(RegEntry::rw_hooked(0x24, 4, vel_hook))
            .unwrap();
        // One continuous write payload crossing both registers.
        let mut frame = [0u8; 9];
        frame[0] = 0x20;
        frame[1..5].copy_from_slice(&le_f32(1.5));
        frame[5..9].copy_from_slice(&le_f32(2.5));
        map.write_phase(&frame).unwrap();
        assert_eq!(map.handler().log.len, 2, "each completed register fires once");
        let mut out = [0u8; 4];
        map.get(0x20, &mut out).unwrap();
        assert_eq!(f32::from_le_bytes(out), 1.5);
        map.get(0x24, &mut out).unwrap();
        assert_eq!(f32::from_le_bytes(out), 2.5);
    }

    #[test]
    fn read_only_write_rejected_without_state_change() {
        let mut map = mini_map();
        map.set(0x20, &le_f32(7.0)).unwrap(); // Ro telemetry published app-side
        map.write_phase(&[0x14, 0, 0, 0, 0]).unwrap(); // now pointer at 0x18
        // Direct Ro write: rejected, value intact.
        assert_eq!(
            map.write_phase(&[0x20, 1, 2, 3, 4]),
            Err(Error::ReadOnly)
        );
        let mut out = [0u8; 4];
        map.get(0x20, &mut out).unwrap();
        assert_eq!(f32::from_le_bytes(out), 7.0);
        // Mixed phase [0x1C..0x24] spans an RW byte range and the Ro register 0x20:
        // the WHOLE phase must be rejected and the RW bytes untouched too.
        map.add_register(RegEntry::rw(0x1C, 4)).unwrap();
        map.write_phase(&[0x1C, 0xAA, 0xBB, 0xCC, 0xDD, 0, 0, 0, 0])
            .unwrap_err();
        map.get(0x1C, &mut out).unwrap();
        assert_eq!(out, [0, 0, 0, 0], "rejected mixed phase must not touch any register");
        // But Ro pointer latches are legal (that is how hosts read telemetry).
        map.write_phase(&[0x20]).unwrap();
        assert_eq!(map.pointer(), 0x20);
    }

    #[test]
    fn self_clearing_system_command_hook() {
        let mut map = mini_map();
        // CMD_REBOOT_BOOTLOADER = 5 (upstream control_table.h SystemCmd).
        map.write_phase(&[0x50, 5]).unwrap();
        assert_eq!(map.handler().log.len, 1);
        assert_eq!(map.handler().log.items[0], 5);
        let mut out = [0u8; 1];
        map.get(0x50, &mut out).unwrap();
        assert_eq!(out[0], 0, "REG_SYSTEM_CMD self-clears after the hook runs");
        // Writing CMD_NONE (0) afterwards is a normal write, hook still runs.
        map.write_phase(&[0x50, 0]).unwrap();
        assert_eq!(map.handler().log.len, 2);
        assert_eq!(map.handler().log.items[1], 0);
    }

    #[test]
    fn hook_cannot_reenter_the_map_structural_guarantee() {
        // Compile-level guarantee (documented contract): the hook signature
        // `fn(&mut Handler, &[u8]) -> HookResult` offers no reference into the RegMap,
        // so hooks are structurally unable to mutate or read the table. This test
        // only pins the published hook type.
        let hook: WriteHookFn<Handler> = sys_cmd_hook;
        let mut map: MiniMap = MiniMap::new(Handler::default())
            .register(RegEntry::rw_hooked(0x50, 1, hook))
            .unwrap();
        map.write_phase(&[0x50, 1]).unwrap();
        assert_eq!(map.handler().log.len, 1);
    }

    #[test]
    fn app_side_set_get_width_matching() {
        let mut map = mini_map();
        // Width mismatch on direct register access.
        assert_eq!(map.set(0x14, &[1, 2, 3]), Err(Error::WidthMismatch));
        // Offset that is not a register start (inside the f32 at 0x14).
        assert_eq!(map.set(0x15, &[1, 2, 3, 4]), Err(Error::Unmapped));
        // Unmapped offset entirely.
        assert_eq!(map.get(0x60, &mut [0u8; 4]), Err(Error::Unmapped));
        // Buffer too small.
        let mut tiny = [0u8; 2];
        assert_eq!(map.get(0x14, &mut tiny), Err(Error::BufferTooSmall));
        let mut out = [0u8; 4];
        map.get(0x14, &mut out).unwrap();
        assert_eq!(out, [0, 0, 0, 0]);
        // Ro registers are writable from the application side (telemetry publisher).
        map.set(0x2C, &le_f32(0.5)).unwrap();
        map.get(0x2C, &mut out).unwrap();
        assert_eq!(f32::from_le_bytes(out), 0.5);
    }

    #[test]
    fn padding_bytes_are_harmless_plain_memory() {
        // Map with a hole at 0x06..0x08 (upstream reserved_06).
        let mut map: MiniMap = MiniMap::new(Handler::default())
            .register(RegEntry::rw(0x00, 1))
            .unwrap()
            .register(RegEntry::rw(0x01, 1))
            .unwrap()
            .register(RegEntry::rw(0x08, 4))
            .unwrap();
        // Write into the padding hole: harmless, no hook, stored.
        map.write_phase(&[0x06, 0xA5, 0x5A]).unwrap();
        // Read across boundary 0x05..0x09: continuous stream over pad + register.
        map.write_phase(&[0x05]).unwrap();
        let mut out = [0u8; 4];
        map.read_phase(4, &mut out).unwrap();
        assert_eq!(out[0], 0x00); // untouched mapped byte
        assert_eq!(out[1], 0xA5); // padding byte, stored by the plain map write
        assert_eq!(out[2], 0x5A);
        assert_eq!(out[3], 0x00); // register start byte
    }

    #[test]
    fn empty_write_phase_is_noop() {
        let mut map = mini_map();
        assert_eq!(map.write_phase(&[]), Ok(()));
        assert_eq!(map.pointer(), 0);
        assert_eq!(map.read_phase(0, &mut []), Ok(0));
    }
}
