//! # drv8311::filter — TI DRV8311 substrate cross-talk decoupling filter
//!
//! References: Texas Instruments DRV8311 Datasheet (SLVSFN2B), Section 8.3.10.2:
//! "Current Sense Amplifier Offset Correction".
//!
//! CSA output has an offset induced due to silicon substrate ground differences between
//! the sense FETs and output FETs across phases. When running sinusoidal or FOC control,
//! applying these decoupling matrices removes inter-phase cross-talk and distortion.

/// Substrate cross-talk decoupling filter for DRV8311 internal Current Sense Amplifiers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Drv8311CurrentSenseFilter;

impl Drv8311CurrentSenseFilter {
    /// Corrects 3-phase raw sensed currents using the TI 3x3 substrate cross-talk matrix.
    ///
    /// Datasheet Equations (3)-(5):
    /// - $i_a = 1.001152 \cdot i_{a\_sensed} - 0.003375 \cdot i_{b\_sensed} - 0.003103 \cdot i_{c\_sensed}$
    /// - $i_b = 0.002369 \cdot i_{a\_sensed} + 1.000665 \cdot i_{b\_sensed} - 0.019126 \cdot i_{c\_sensed}$
    /// - $i_c = 0.001234 \cdot i_{a\_sensed} + 0.001595 \cdot i_{b\_sensed} + 0.998166 \cdot i_{c\_sensed}$
    #[inline]
    pub fn correct_3phase(ia_sensed: f32, ib_sensed: f32, ic_sensed: f32) -> (f32, f32, f32) {
        let ia = 1.001152 * ia_sensed - 0.003375 * ib_sensed - 0.003103 * ic_sensed;
        let ib = 0.002369 * ia_sensed + 1.000665 * ib_sensed - 0.019126 * ic_sensed;
        let ic = 0.001234 * ia_sensed + 0.001595 * ib_sensed + 0.998166 * ic_sensed;
        (ia, ib, ic)
    }

    /// Corrects 2-phase sensed currents when phases A & B are measured.
    ///
    /// Datasheet Equations (6)-(8):
    /// - $i_a = 1.004346 \cdot i_{a\_sensed} - 0.000199 \cdot i_{b\_sensed}$
    /// - $i_b = 0.022060 \cdot i_{a\_sensed} + 1.020405 \cdot i_{b\_sensed}$
    /// - $i_c = -(i_a + i_b)$
    #[inline]
    pub fn correct_2phase_ab(ia_sensed: f32, ib_sensed: f32) -> (f32, f32, f32) {
        let ia = 1.004346 * ia_sensed - 0.000199 * ib_sensed;
        let ib = 0.022060 * ia_sensed + 1.020405 * ib_sensed;
        let ic = -(ia + ib);
        (ia, ib, ic)
    }

    /// Corrects 2-phase sensed currents when phases B & C are measured.
    ///
    /// Datasheet Equations (9)-(11):
    /// - $i_b = 0.998309 \cdot i_{b\_sensed} - 0.021427 \cdot i_{c\_sensed}$
    /// - $i_c = 0.000368 \cdot i_{b\_sensed} + 0.996967 \cdot i_{c\_sensed}$
    /// - $i_a = -(i_b + i_c)$
    #[inline]
    pub fn correct_2phase_bc(ib_sensed: f32, ic_sensed: f32) -> (f32, f32, f32) {
        let ib = 0.998309 * ib_sensed - 0.021427 * ic_sensed;
        let ic = 0.000368 * ib_sensed + 0.996967 * ic_sensed;
        let ia = -(ib + ic);
        (ia, ib, ic)
    }

    /// Corrects 2-phase sensed currents when phases C & A are measured.
    ///
    /// Datasheet Equations (12)-(14):
    /// - $i_a = 1.004547 \cdot i_{a\_sensed} + 0.000195 \cdot i_{c\_sensed}$
    /// - $i_c = 0.000371 \cdot i_{a\_sensed} + 0.996975 \cdot i_{c\_sensed}$
    /// - $i_b = -(i_a + i_c)$
    #[inline]
    pub fn correct_2phase_ca(ic_sensed: f32, ia_sensed: f32) -> (f32, f32, f32) {
        let ia = 1.004547 * ia_sensed + 0.000195 * ic_sensed;
        let ic = 0.000371 * ia_sensed + 0.996975 * ic_sensed;
        let ib = -(ia + ic);
        (ia, ib, ic)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_correct_3phase_zero() {
        let (ia, ib, ic) = Drv8311CurrentSenseFilter::correct_3phase(0.0, 0.0, 0.0);
        assert_eq!(ia, 0.0);
        assert_eq!(ib, 0.0);
        assert_eq!(ic, 0.0);
    }

    #[test]
    fn test_correct_3phase_unity() {
        let (ia, ib, ic) = Drv8311CurrentSenseFilter::correct_3phase(1.0, 0.0, 0.0);
        assert!((ia - 1.001152).abs() < 1e-6);
        assert!((ib - 0.002369).abs() < 1e-6);
        assert!((ic - 0.001234).abs() < 1e-6);
    }

    #[test]
    fn test_correct_2phase_kirchhoff() {
        let (ia, ib, ic) = Drv8311CurrentSenseFilter::correct_2phase_ab(1.0, -0.5);
        assert!((ia + ib + ic).abs() < 1e-6);

        let (ia2, ib2, ic2) = Drv8311CurrentSenseFilter::correct_2phase_bc(1.0, -0.5);
        assert!((ia2 + ib2 + ic2).abs() < 1e-6);

        let (ia3, ib3, ic3) = Drv8311CurrentSenseFilter::correct_2phase_ca(1.0, -0.5);
        assert!((ia3 + ib3 + ic3).abs() < 1e-6);
    }
}
