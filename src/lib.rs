pub mod geo;

pub const DEFAULT_TOLERANCE_CALCULATION: f64 = 1.0e-8;
pub const DEFAULT_TOLERANCE_CONVERGENCE: f64 = 1.0e-6;
pub const DEFAULT_TOLERANCE_POINT: f64 = 1.0e-4;
pub const DEFAULT_TOLERANCE_VECTOR: f64 = 1.0e-6;

#[derive(PartialOrd, PartialEq, Debug)]
pub enum ErrorStatus {
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

pub struct Tolerance {
    pub equal_point: f64,
    pub equal_vector: f64,
    pub convergence: f64,
    pub calculation: f64,
}

impl Default for Tolerance {
    fn default() -> Self {
        Self { equal_point: DEFAULT_TOLERANCE_POINT,
               equal_vector: DEFAULT_TOLERANCE_VECTOR,
               convergence: DEFAULT_TOLERANCE_CONVERGENCE,
               calculation: DEFAULT_TOLERANCE_CALCULATION }
    }
}
