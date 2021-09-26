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

    #[derive(Debug)]
    struct Vector {
        pub x: f64,
        pub y: f64,
        pub z: f64,
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
