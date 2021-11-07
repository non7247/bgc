pub mod geo;

pub const DEFAULT_TOLERANCE_CALCULATION: f64 = 1.0e-8;
pub const DEFAULT_TOLERANCE_CONVERGENCE: f64 = 1.0e-6;
pub const DEFAULT_TOLERANCE_POINT: f64 = 1.0e-4;
pub const DEFAULT_TOLERANCE_VECTOR: f64 = 1.0e-6;

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
