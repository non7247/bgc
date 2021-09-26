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
    struct Point {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    impl Point {
        fn origin() -> Point {
            Point { x: 0.0, y: 0.0, z: 0.0 }
        }
    }

    #[derive(Debug)]
    struct Vector {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    impl Vector {
        fn x_axis() -> Vector {
            Vector { x: 1.0, y: 0.0, z: 0.0 }
        }
        fn y_axis() -> Vector {
            Vector { x: 0.0, y: 1.0, z: 0.0 }
        }
        fn z_axis() -> Vector {
            Vector { x: 0.0, y: 0.0, z: 1.0 }
        }
    }
}

mod math {
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
