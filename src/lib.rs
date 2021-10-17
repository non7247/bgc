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
}
