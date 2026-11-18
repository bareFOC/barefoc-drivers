//! Host unit tests for the pure BareBoot decoders and layout checks.
//!
//! No test touches real flash or peripherals: presence is evaluated from a
//! fake two-word vector table via the crate-private `is_present_words` helper.

use super::*;

const FLASH_BASE: u32 = 0x1000_4000;
const APP_BASE: u32 = 0x1010_4000;
const SRAM_TOP: u32 = 0x2000_8000;
const GOOD_SP: u32 = 0x2000_7F00;
const GOOD_RESET_VEC: u32 = 0x1000_4101;

fn target() -> BootTarget {
    BootTarget::new(FLASH_BASE, APP_BASE, SRAM_TOP)
}

/// Fake flash buffer holding the first two vector-table words.
struct FakeFlash {
    words: [u32; 2],
}

impl FakeFlash {
    fn new(initial_sp: u32, reset_vec: u32) -> Self {
        Self {
            words: [initial_sp, reset_vec],
        }
    }

    fn is_present(&self, target: &BootTarget) -> bool {
        is_present_words(
            target.flash_base,
            target.app_base,
            self.words[0],
            self.words[1],
        )
    }
}

#[test]
fn constants_match_bareboot_contract() {
    assert_eq!(REQUEST_CANBOOT, REQUEST_BOOTLOADER);
    assert_eq!(REQUEST_BOOTLOADER, 0x5984_E3FA_6CA1_589B);
    assert_eq!(REQUEST_START_APP, 0x7B06_EC45_A9A8_243D);
    assert_eq!(BAREBOOT_MAGIC, 0x2174_6F6F_426E_6143);
    assert_eq!(BAREBOOT_MAGIC.to_le_bytes(), *b"CanBoot!");
    assert_eq!(SERIAL_BL_REQ, b"~ \x1c Request Serial Bootloader!! ~");
    assert_eq!(I2C_CMD_ENTER_BOOTLOADER, 0xF0);
    assert_eq!(CAN_ADMIN_REBOOT_ID, 0x3F0);
    assert_eq!(CAN_CMD_REBOOT, 0x02);
    assert_eq!(SPI_SYNC, [0x5A, 0xA5, 0xF0, 0x59, 0x84, 0xE3, 0xFA, 0x6C]);
    assert_eq!(I3C_CCC_RSTACT_BCAST, 0x2A);
    assert_eq!(I3C_CCC_RSTACT_DIRECT, 0x9A);
    assert_eq!(I3C_RSTACT_TARGET_RESET, 0x02);
    assert_eq!(USB_VENDOR_BREQUEST, 0xBB);
    assert_eq!(USB_CMD_ENTER_BOOTLOADER, 0x0001);
    assert_eq!(BLE_OTA_CONTROL_WRITE, 0x01);
}

#[test]
fn boot_target_accessors() {
    let t = target();
    assert_eq!(t.flash_base, FLASH_BASE);
    assert_eq!(t.app_base, APP_BASE);
    assert_eq!(t.sram_top, SRAM_TOP);
    assert_eq!(t.fallback_mailbox_addr(), (SRAM_TOP - 8) as usize);

    let tiny = BootTarget::new(0, 0, 4);
    assert_eq!(tiny.fallback_mailbox_addr(), 0);
}

#[test]
fn is_present_accepts_valid_vector_table() {
    let t = target();
    let flash = FakeFlash::new(GOOD_SP, GOOD_RESET_VEC);
    assert!(flash.is_present(&t));

    // SP bounds are inclusive; reset-vector lower bound is inclusive.
    assert!(FakeFlash::new(0x2000_0000, FLASH_BASE).is_present(&t));
    assert!(FakeFlash::new(0x2008_0000, APP_BASE - 1).is_present(&t));
}

#[test]
fn is_present_rejects_bad_sp() {
    let t = target();
    assert!(!FakeFlash::new(0x1FFF_FFFF, GOOD_RESET_VEC).is_present(&t));
    assert!(!FakeFlash::new(0x2008_0001, GOOD_RESET_VEC).is_present(&t));
    assert!(!FakeFlash::new(0, GOOD_RESET_VEC).is_present(&t));
    assert!(!FakeFlash::new(u32::MAX, GOOD_RESET_VEC).is_present(&t));
}

#[test]
fn is_present_rejects_bad_reset_vector() {
    let t = target();
    assert!(!FakeFlash::new(GOOD_SP, FLASH_BASE - 1).is_present(&t));
    assert!(!FakeFlash::new(GOOD_SP, APP_BASE).is_present(&t));
    assert!(!FakeFlash::new(GOOD_SP, 0).is_present(&t));
    assert!(!FakeFlash::new(GOOD_SP, u32::MAX).is_present(&t));
}

#[test]
fn decode_i2c_requires_exactly_one_byte() {
    assert!(trigger_i2c(&[0xF0]));
    assert!(!trigger_i2c(&[]));
    assert!(!trigger_i2c(&[0xF0, 0x00]));
    assert!(!trigger_i2c(&[0x00, 0xF0]));
    assert!(!trigger_i2c(&[0xEF]));
}

#[test]
fn decode_i3c_ccc_requires_rstact_target_reset() {
    assert!(trigger_i3c_ccc(
        I3C_CCC_RSTACT_BCAST,
        I3C_RSTACT_TARGET_RESET
    ));
    assert!(trigger_i3c_ccc(
        I3C_CCC_RSTACT_DIRECT,
        I3C_RSTACT_TARGET_RESET
    ));

    // Other RSTACT defining bytes mean no action / peripheral or debug reset.
    for defining in [0x00, 0x01, 0x03, 0xFF] {
        assert!(!trigger_i3c_ccc(I3C_CCC_RSTACT_BCAST, defining));
        assert!(!trigger_i3c_ccc(I3C_CCC_RSTACT_DIRECT, defining));
    }
    assert!(!trigger_i3c_ccc(0x00, I3C_RSTACT_TARGET_RESET));
    assert!(!trigger_i3c_ccc(0x2B, I3C_RSTACT_TARGET_RESET));
}

#[test]
fn decode_i3c_byte_accepts_raw_f0_only() {
    assert!(trigger_i3c_byte(0xF0));
    assert!(!trigger_i3c_byte(0x00));
    assert!(!trigger_i3c_byte(0x02));
    assert!(!trigger_i3c_byte(0xEF));
}

#[test]
fn decode_can_matches_id_command_and_uid() {
    let uid: [u8; 6] = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
    let frame = [0x02, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
    assert!(trigger_can(CAN_ADMIN_REBOOT_ID, &frame, &uid));

    assert!(!trigger_can(0x3F1, &frame, &uid));
    let wrong_cmd = [0x01, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
    assert!(!trigger_can(CAN_ADMIN_REBOOT_ID, &wrong_cmd, &uid));
    let wrong_uid = [0x02, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBD];
    assert!(!trigger_can(CAN_ADMIN_REBOOT_ID, &wrong_uid, &uid));

    // DLC is fixed at seven bytes; no short or extended frames.
    assert!(!trigger_can(CAN_ADMIN_REBOOT_ID, &frame[..6], &uid));
    assert!(!trigger_can(CAN_ADMIN_REBOOT_ID, &[0x02], &uid));
    let mut extended = [0u8; 8];
    extended[..7].copy_from_slice(&frame);
    assert!(!trigger_can(CAN_ADMIN_REBOOT_ID, &extended, &uid));
}

#[test]
fn decode_uart_detects_magic_substring() {
    assert!(trigger_uart(SERIAL_BL_REQ));

    let mut buf = [0u8; 64];
    buf[0..3].copy_from_slice(b">> ");
    buf[3..3 + SERIAL_BL_REQ.len()].copy_from_slice(SERIAL_BL_REQ);
    buf[3 + SERIAL_BL_REQ.len()..5 + SERIAL_BL_REQ.len()].copy_from_slice(b"\r\n");
    assert!(trigger_uart(&buf[..5 + SERIAL_BL_REQ.len()]));

    assert!(!trigger_uart(&[]));
    assert!(!trigger_uart(&SERIAL_BL_REQ[..SERIAL_BL_REQ.len() - 1]));
    assert!(!trigger_uart(b"~ \x1c Request Serial Bootloader! ~"));
    assert!(!trigger_uart(b"console ready"));
}

#[test]
fn decode_spi_matches_sync_prefix() {
    assert!(trigger_spi(&SPI_SYNC));
    let mut with_tail = [0u8; 12];
    with_tail[..8].copy_from_slice(&SPI_SYNC);
    with_tail[8..].copy_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);
    assert!(trigger_spi(&with_tail));

    assert!(!trigger_spi(&[]));
    assert!(!trigger_spi(&SPI_SYNC[..7]));
    let mut corrupt = SPI_SYNC;
    corrupt[2] = 0xF1;
    assert!(!trigger_spi(&corrupt));
    assert!(!trigger_spi(&[0x00; 8]));
}

#[test]
fn decode_usb_vendor_requires_brequest_and_wvalue() {
    assert!(trigger_usb_vendor(
        USB_VENDOR_BREQUEST,
        USB_CMD_ENTER_BOOTLOADER
    ));
    assert!(!trigger_usb_vendor(0xBA, USB_CMD_ENTER_BOOTLOADER));
    assert!(!trigger_usb_vendor(USB_VENDOR_BREQUEST, 0x0000));
    assert!(!trigger_usb_vendor(USB_VENDOR_BREQUEST, 0x0002));
}

#[test]
fn decode_usb_1200_baud() {
    assert!(trigger_usb_1200_baud(1200));
    assert!(!trigger_usb_1200_baud(0));
    assert!(!trigger_usb_1200_baud(115_200));
    assert!(!trigger_usb_1200_baud(1201));
}

#[test]
fn decode_usb_dfu_detach() {
    assert!(trigger_usb_dfu_detach(0x00));
    assert!(!trigger_usb_dfu_detach(0x01));
    assert!(!trigger_usb_dfu_detach(0xBB));
}

#[test]
fn decode_ble_ctrl() {
    assert!(trigger_ble_ctrl(BLE_OTA_CONTROL_WRITE));
    assert!(!trigger_ble_ctrl(0x00));
    assert!(!trigger_ble_ctrl(0x02));
}
