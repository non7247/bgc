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
    fn point_is_equal_to() {
        let lhs = geo::Point { x: 1.0, y: 2.0, z: 3.0 };
        let rhs = geo::Point { x: 1.0, y: 2.0, z: 3.0 };

        assert!(lhs.is_equal_to(&rhs, crate::DEFAULT_TOLERANCE_POINT));

        let rhs = geo::Point { x: 1.1, y: 2.1, z: 3.1 };

        assert!(!lhs.is_equal_to(&rhs, crate::DEFAULT_TOLERANCE_POINT));
    }

    #[test]
    fn vector_is_equal_to() {
        let lhs = geo::Vector { x: 1.0, y: 2.0, z: 3.0 };
        let rhs = geo::Vector { x: 1.0, y: 2.0, z: 3.0 };

        assert!(lhs.is_equal_to(&rhs, crate::DEFAULT_TOLERANCE_VECTOR));

        let rhs = geo::Vector { x: 1.1, y: 2.1, z: 3.1 };

        assert!(!lhs.is_equal_to(&rhs, crate::DEFAULT_TOLERANCE_VECTOR));
    }
}
