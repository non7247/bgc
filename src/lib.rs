mod geo;

const DEFAULT_TOLERANCE_CALCULATION: f64 = 1.0e-8;
const DEFAULT_TOLERANCE_CONVERGENCE: f64 = 1.0e-6;
const DEFAULT_TOLERANCE_POINT: f64 = 1.0e-4;
const DEFAULT_TOLERANCE_VECTOR: f64 = 1.0e-6;

enum ErrorStatus {
    Ok,
    InvalidInput,
    OutOfRange,
    MustBePositive,
    MustBeNoNegative,
    MustBeNonZero,
    Deivergence,
    Empty,
    NotInplemented,
}

mod math {
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_length() {
        let l = geo::Line { start_point: geo::Point { x: 0.0, y: 0.0, z: 0.0},
                            end_point: geo::Point { x: 1.0, y: 1.0, z: 0.0} };
        assert!((l.length() - 2.0_f64.sqrt()).abs() < crate::DEFAULT_TOLERANCE_POINT);
    }

    #[test]
    fn line_get_closest_point() {
        let l = geo::Line { start_point: geo::Point { x: 1379.591836, y: 1159.400383, z: 0.0 },
                            end_point: geo::Point { x: 3079.683229, y: 2067.058311, z: 0.0 } };

        let p = l.get_closest_point(&geo::Point { x: 3908.885031, y: 1901.285447, z: 0.0 },
                                    false, crate::DEFAULT_TOLERANCE_POINT);
        assert!(p.is_equal_to(&geo::Point { x: 3079.683229, y: 2067.058311, z: 0.0 },
                              crate::DEFAULT_TOLERANCE_POINT));
        let p = l.get_closest_point(&geo::Point { x: 3908.885031, y: 1901.285447, z: 0.0 },
                                    true, crate::DEFAULT_TOLERANCE_POINT);
        assert!(p.is_equal_to(&geo::Point { x: 3656.085482, y: 2374.792398, z: 0.0 },
                              crate::DEFAULT_TOLERANCE_POINT));

        let p = l.get_closest_point(&geo::Point { x: 569.433291, y: 1366.238184, z: 0.0 },
                                    false, crate::DEFAULT_TOLERANCE_POINT);
        assert!(p.is_equal_to(&geo::Point { x: 1379.591836, y: 1159.400383, z: 0.0 },
                              crate::DEFAULT_TOLERANCE_POINT));
        let p = l.get_closest_point(&geo::Point { x: 569.433291, y: 1366.238184, z: 0.0 },
                                    true, crate::DEFAULT_TOLERANCE_POINT);
        assert!(p.is_equal_to(&geo::Point { x: 835.069873, y: 868.686791, z: 0.0 },
                              crate::DEFAULT_TOLERANCE_POINT));
    }
}
