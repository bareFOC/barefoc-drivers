//! Integration and unit tests for TMAG5273 driver using embedded-hal-mock.

use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use tmag_rs::registers::*;
use tmag_rs::*;

const ADDR: u8 = 0x35;

fn init_expectations(
    address: u8,
    angle_en: AngleEnable,
    read_mode: I2cReadMode,
) -> Vec<I2cTransaction> {
    let mut sc2 = 0x03; // xy_range=High, z_range=High
    sc2 = insert(sc2, ANGLE_EN_MASK, ANGLE_EN_SHIFT, angle_en as u8);
    let dc1 = insert(0, I2C_RD_MASK, I2C_RD_SHIFT, read_mode as u8);
    vec![
        I2cTransaction::write_read(address, vec![MANUFACTURER_ID_LSB], vec![0x49, 0x54]),
        I2cTransaction::write_read(address, vec![DEVICE_ID], vec![VER_V1]),
        I2cTransaction::write(address, vec![CONV_STATUS, POR_MASK]),
        I2cTransaction::write(address, vec![DEVICE_STATUS, FAULT_CLEAR_MASK]),
        I2cTransaction::write(address, vec![DEVICE_CONFIG_1, dc1]),
        I2cTransaction::write(address, vec![DEVICE_CONFIG_2, 0x1A]),
        I2cTransaction::write(address, vec![SENSOR_CONFIG_1, 0x70]),
        I2cTransaction::write(address, vec![SENSOR_CONFIG_2, sc2]),
        I2cTransaction::write(address, vec![T_CONFIG, 0x00]),
    ]
}

#[test]
fn test_init_success() {
    let expectations = init_expectations(ADDR, AngleEnable::None, I2cReadMode::Standard);
    let i2c = I2cMock::new(&expectations);

    let sensor = Tmag5273::builder(i2c, ADDR).build().unwrap();
    assert_eq!(sensor.address(), ADDR);
    assert_eq!(sensor.variant(), DeviceVariant::A1);
    assert_eq!(sensor.read_mode(), I2cReadMode::Standard);

    let mut i2c = sensor.into_inner();
    i2c.done();
}

#[test]
fn test_init_invalid_mfg_id() {
    let expectations = [I2cTransaction::write_read(
        ADDR,
        vec![MANUFACTURER_ID_LSB],
        vec![0x00, 0x00],
    )];
    let mut i2c = I2cMock::new(&expectations);

    let err = Tmag5273::new(i2c.clone(), ADDR).unwrap_err();
    assert_eq!(err, Error::InvalidManufacturerId(0x0000));
    i2c.done();
}

#[test]
fn test_init_version_mismatch() {
    let expectations = [
        I2cTransaction::write_read(ADDR, vec![MANUFACTURER_ID_LSB], vec![0x49, 0x54]),
        I2cTransaction::write_read(ADDR, vec![DEVICE_ID], vec![VER_V2]), // version 2 when A1 expects 1
    ];
    let mut i2c = I2cMock::new(&expectations);

    let err = Tmag5273::new(i2c.clone(), ADDR).unwrap_err();
    assert_eq!(
        err,
        Error::VersionMismatch {
            expected: VER_V1,
            got: VER_V2,
        }
    );
    i2c.done();
}

#[test]
fn test_register_read_write() {
    let mut expectations = init_expectations(ADDR, AngleEnable::None, I2cReadMode::Standard);
    // Write register 0x04 (X_THR_CONFIG)
    expectations.push(I2cTransaction::write(ADDR, vec![X_THR_CONFIG, 0x42]));
    // Read register 0x04
    expectations.push(I2cTransaction::write_read(
        ADDR,
        vec![X_THR_CONFIG],
        vec![0x42],
    ));
    // Read 2 registers starting at 0x10
    expectations.push(I2cTransaction::write_read(
        ADDR,
        vec![T_MSB_RESULT],
        vec![0x12, 0x34],
    ));

    let i2c = I2cMock::new(&expectations);
    let mut sensor = Tmag5273::new(i2c, ADDR).unwrap();

    sensor.write_register(X_THR_CONFIG, 0x42).unwrap();
    let val = sensor.read_register(X_THR_CONFIG).unwrap();
    assert_eq!(val, 0x42);

    let mut buf = [0u8; 2];
    sensor.read_registers(T_MSB_RESULT, &mut buf).unwrap();
    assert_eq!(buf, [0x12, 0x34]);

    sensor.into_inner().done();
}

#[test]
fn test_clear_faults_and_por() {
    let mut expectations = init_expectations(ADDR, AngleEnable::None, I2cReadMode::Standard);
    // clear_por -> write 0x10 to CONV_STATUS (0x18)
    expectations.push(I2cTransaction::write(ADDR, vec![CONV_STATUS, 0x10]));
    // clear_faults -> write 0x0F to DEVICE_STATUS (0x1C)
    expectations.push(I2cTransaction::write(ADDR, vec![DEVICE_STATUS, 0x0F]));

    let i2c = I2cMock::new(&expectations);
    let mut sensor = Tmag5273::new(i2c, ADDR).unwrap();

    sensor.clear_por().unwrap();
    sensor.clear_faults().unwrap();

    sensor.into_inner().done();
}

#[test]
fn test_read_status_and_diagnostics() {
    let mut expectations = init_expectations(ADDR, AngleEnable::None, I2cReadMode::Standard);
    // DEVICE_STATUS with VCC_UV and OSC_ER flags set: 0x01 | 0x08 = 0x09
    expectations.push(I2cTransaction::write_read(
        ADDR,
        vec![DEVICE_STATUS],
        vec![0x09],
    ));
    // CONV_STATUS with RESULT_READY and SET_COUNT=3: 0x01 | (3 << 5 = 0x60) = 0x61
    expectations.push(I2cTransaction::write_read(
        ADDR,
        vec![CONV_STATUS],
        vec![0x61],
    ));

    let i2c = I2cMock::new(&expectations);
    let mut sensor = Tmag5273::new(i2c, ADDR).unwrap();

    let dev_status = sensor.read_status().unwrap();
    assert!(dev_status.vcc_undervoltage);
    assert!(dev_status.osc_error);
    assert!(!dev_status.otp_crc_error);
    assert!(dev_status.has_fault());

    let conv_status = sensor.read_conv_status().unwrap();
    assert!(conv_status.result_ready);
    assert_eq!(conv_status.set_count, 3);
    assert!(!conv_status.por_occurred);

    sensor.into_inner().done();
}

#[test]
fn test_cordic_angle_extraction() {
    let mut expectations = init_expectations(ADDR, AngleEnable::XY, I2cReadMode::Standard);
    // Read ANGLE_RESULT_MSB (0x19): 180 degrees -> 180 * 16 = 2880 = 0x0B40
    expectations.push(I2cTransaction::write_read(
        ADDR,
        vec![ANGLE_RESULT_MSB],
        vec![0x0B, 0x40],
    ));

    let i2c = I2cMock::new(&expectations);
    let mut sensor = Tmag5273::builder(i2c, ADDR)
        .angle_enabled(AngleEnable::XY)
        .build()
        .unwrap();

    let angle = sensor.read_angle().unwrap();
    assert!((angle.degrees() - 180.0).abs() < 1e-4);
    assert!((angle.radians() - _PI).abs() < 1e-4);

    sensor.into_inner().done();
}

#[test]
fn test_magnetic_burst_6byte_and_plane_angles() {
    let mut expectations = init_expectations(ADDR, AngleEnable::None, I2cReadMode::Standard);
    // Range is High (80 mT).
    // Let raw_x = 16384 (40.0 mT), raw_y = 0 (0.0 mT), raw_z = 16384 (40.0 mT).
    // Big-endian:
    // raw_x = 0x4000
    // raw_y = 0x0000
    // raw_z = 0x4000
    expectations.push(I2cTransaction::write_read(
        ADDR,
        vec![X_MSB_RESULT],
        vec![0x40, 0x00, 0x00, 0x00, 0x40, 0x00],
    ));

    let i2c = I2cMock::new(&expectations);
    let mut sensor = Tmag5273::new(i2c, ADDR).unwrap();

    let mag = sensor.read_magnetic().unwrap();
    assert!((mag.x - 40.0).abs() < 1e-3);
    assert!((mag.y - 0.0).abs() < 1e-3);
    assert!((mag.z - 40.0).abs() < 1e-3);

    // Plane angles: alpha = atan2(40, 40) = pi / 4, beta = atan2(0, 40) = 0
    let [alpha, beta] = mag.plane_angles();
    assert!((alpha - core::f32::consts::FRAC_PI_4).abs() < 1e-5);
    assert!((beta - 0.0).abs() < 1e-5);

    sensor.into_inner().done();
}

#[test]
fn test_1byte_stream_mode() {
    let mut expectations = init_expectations(ADDR, AngleEnable::None, I2cReadMode::OneByte16Bit);
    // 1-byte fast stream read: directly reads 6 bytes without writing register address!
    expectations.push(I2cTransaction::read(
        ADDR,
        vec![0x40, 0x00, 0x20, 0x00, 0x10, 0x00],
    ));

    let i2c = I2cMock::new(&expectations);
    let mut sensor = Tmag5273::builder(i2c, ADDR)
        .read_mode(I2cReadMode::OneByte16Bit)
        .build()
        .unwrap();

    let mut buf = [0u8; 6];
    sensor.read_stream(&mut buf).unwrap();
    assert_eq!(buf, [0x40, 0x00, 0x20, 0x00, 0x10, 0x00]);

    sensor.into_inner().done();
}

#[test]
fn test_read_magnetic_8bit_stream() {
    let mut expectations = init_expectations(ADDR, AngleEnable::None, I2cReadMode::OneByte8Bit);
    // 8-bit stream returns [X_MSB, Y_MSB, Z_MSB, CONV_STATUS]
    // 64 -> 40 mT, -64 -> -40 mT, 127 -> ~79.375 mT, 0x01
    expectations.push(I2cTransaction::read(
        ADDR,
        vec![64, (-64i8) as u8, 127, 0x01],
    ));

    let i2c = I2cMock::new(&expectations);
    let mut sensor = Tmag5273::builder(i2c, ADDR)
        .read_mode(I2cReadMode::OneByte8Bit)
        .build()
        .unwrap();

    let mag = sensor.read_magnetic_8bit().unwrap();
    assert!((mag.x - 40.0).abs() < 1e-3);
    assert!((mag.y - (-40.0)).abs() < 1e-3);
    assert!((mag.z - 79.375).abs() < 1e-3);

    sensor.into_inner().done();
}

#[cfg(feature = "barefoc")]
#[test]
fn test_barefoc_sensor_1d() {
    use ::barefoc::common::time::Clock;
    use ::barefoc::traits::sensor::Sensor;

    struct TestClock(u32);
    impl Clock for TestClock {
        fn now_us(&self) -> u32 {
            self.0
        }
    }

    let mut expectations = init_expectations(ADDR, AngleEnable::XY, I2cReadMode::Standard);
    // Read angle: 180 degrees -> 0x0B40
    expectations.push(I2cTransaction::write_read(
        ADDR,
        vec![ANGLE_RESULT_MSB],
        vec![0x0B, 0x40],
    ));

    let i2c = I2cMock::new(&expectations);
    let driver = Tmag5273::builder(i2c, ADDR)
        .angle_enabled(AngleEnable::XY)
        .build()
        .unwrap();

    let mut sensor = Tmag5273Sensor::new(driver);
    let clock = TestClock(1000);

    sensor.update(&clock);
    let angle = Sensor::<TestClock>::get_mechanical_angle(&sensor);
    assert!((angle - _PI).abs() < 1e-4);

    sensor.driver.into_inner().into_inner().done();
}

#[cfg(feature = "barefoc")]
#[test]
fn test_barefoc_sensor_2d() {
    use ::barefoc::common::time::Clock;
    use barefoc_kinematics::sensor::Sensor2D;

    struct TestClock(u32);
    impl Clock for TestClock {
        fn now_us(&self) -> u32 {
            self.0
        }
    }

    let mut expectations = init_expectations(ADDR, AngleEnable::None, I2cReadMode::Standard);
    // Read 6 bytes magnetic burst: Bx = 40 mT, By = 0 mT, Bz = 40 mT
    expectations.push(I2cTransaction::write_read(
        ADDR,
        vec![X_MSB_RESULT],
        vec![0x40, 0x00, 0x00, 0x00, 0x40, 0x00],
    ));

    let i2c = I2cMock::new(&expectations);
    let driver = Tmag5273::new(i2c, ADDR).unwrap();

    let mut sensor_2d = Tmag5273Sensor2D::new(driver);
    let clock = TestClock(1000);

    sensor_2d.update_2d(&clock);
    let angles = Sensor2D::<TestClock>::get_angles(&sensor_2d);
    assert!((angles[0] - core::f32::consts::FRAC_PI_4).abs() < 1e-5);
    assert!((angles[1] - 0.0).abs() < 1e-5);
    assert_eq!(sensor_2d.angles(), angles);

    sensor_2d.driver.into_inner().done();
}
