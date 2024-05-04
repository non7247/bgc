use std::error;
use std::fmt;

pub mod geo;

pub const DEFAULT_TOLERANCE_CALCULATION: f64 = 1.0e-8;
pub const DEFAULT_TOLERANCE_CONVERGENCE: f64 = 1.0e-6;
pub const DEFAULT_TOLERANCE_POINT: f64 = 1.0e-4;
pub const DEFAULT_TOLERANCE_VECTOR: f64 = 1.0e-6;

#[derive(Debug, PartialEq)]
pub enum BgcError {
    InvalidInput,
    OutOfRange,
    MustBePositive,
    MustBeNoNegative,
    MustBeNonZero,
    Deivergence,
    Empty,
    NotInplemented,
}

impl error::Error for BgcError {}

impl fmt::Display for BgcError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            BgcError::InvalidInput => write!(f, "invalid input"),
            BgcError::OutOfRange => write!(f, "out of range"),
            BgcError::MustBePositive => write!(f, "must be positive"),
            BgcError::MustBeNoNegative => write!(f, "must be non negative"),
            BgcError::MustBeNonZero => write!(f, "must be non zero"),
            BgcError::Deivergence => write!(f, "deivergence"),
            BgcError::Empty => write!(f, "empty"),
            BgcError::NotInplemented => write!(f, "not inplemented"),
        }
    }
}

mod math {
}

#[derive(Debug)]
pub struct Tolerance {
    equal_point: f64,
    equal_vector: f64,
    convergence: f64,
    calculation: f64,
}

impl Default for Tolerance {
    fn default() -> Self {
        Self { equal_point: DEFAULT_TOLERANCE_POINT,
               equal_vector: DEFAULT_TOLERANCE_VECTOR,
               convergence: DEFAULT_TOLERANCE_CONVERGENCE,
               calculation: DEFAULT_TOLERANCE_CALCULATION }
    }
}

impl Tolerance {
    pub fn equal_point(&self) -> f64 {
        self.equal_point
    }

    pub fn equal_vector(&self) -> f64 {
        self.equal_vector
    }

    pub fn convergence(&self) -> f64 {
        self.convergence
    }

    pub fn calculation(&self) -> f64 {
        self.calculation
    }

    pub fn set_equal_point(&mut self, tol: f64) {
        self.equal_point = if tol < 0.0 {
            DEFAULT_TOLERANCE_POINT
        } else {
            tol
        };
    }

    pub fn set_equal_vector(&mut self, tol: f64) {
        self.equal_vector = if tol < 0.0 {
            DEFAULT_TOLERANCE_VECTOR
        } else {
            tol
        };
    }

    pub fn set_convergnece(&mut self, tol: f64) {
        self.convergence = if tol < 0.0 {
            DEFAULT_TOLERANCE_CONVERGENCE
        } else {
            tol
        };
    }

    pub fn set_calculation(&mut self, tol: f64) {
        self.calculation = if tol < 0.0 {
            DEFAULT_TOLERANCE_CALCULATION
        } else {
            tol
        };
    }
}
