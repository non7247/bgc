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

mod geo {
    #[derive(Debug)]
    pub struct Point {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    impl Point {
        pub fn origin() -> Point {
            Point { x: 0.0, y: 0.0, z: 0.0 }
        }

        pub fn distance_to(&self, rhs: &Self) -> f64 {
            let dx = self.x - rhs.x;
            let dy = self.y - rhs.y;
            let dz = self.z - rhs.z;

            (dx * dx + dy * dy + dz * dz).sqrt()
        }

        pub fn is_equal_to(&self, rhs: &Self) -> bool {
            self.distance_to(rhs) < crate::DEFAULT_TOLERANCE_POINT
        }
    }

    #[derive(Debug)]
    pub struct Vector {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    impl Vector {
        pub fn x_axis() -> Vector {
            Vector { x: 1.0, y: 0.0, z: 0.0 }
        }

        pub fn y_axis() -> Vector {
            Vector { x: 0.0, y: 1.0, z: 0.0 }
        }

        pub fn z_axis() -> Vector {
            Vector { x: 0.0, y: 0.0, z: 1.0 }
        }
    }
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

        assert!(lhs.is_equal_to(&rhs));
    }
}
