pub mod geo;
pub mod math;

pub const DEFAULT_TOLERANCE_CALCULATION: f64 = 1.0e-6;
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
    NotImplemented,
}

impl std::error::Error for BgcError {}

impl std::fmt::Display for BgcError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            BgcError::InvalidInput => write!(f, "invalid input"),
            BgcError::OutOfRange => write!(f, "out of range"),
            BgcError::MustBePositive => write!(f, "must be positive"),
            BgcError::MustBeNoNegative => write!(f, "must be non negative"),
            BgcError::MustBeNonZero => write!(f, "must be non zero"),
            BgcError::Deivergence => write!(f, "deivergence"),
            BgcError::Empty => write!(f, "empty"),
            BgcError::NotImplemented => write!(f, "not implemented"),
        }
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tolerance_setters() {
        let mut tol = Tolerance::default();
        let epsilon = 1.0e-9; // 比較用の微小な値

        // 有効な値を設定するテスト
        tol.set_equal_point(0.1);
        assert!((tol.equal_point() - 0.1).abs() < epsilon);

        // 無効な値（負の値）を設定するテスト
        tol.set_equal_point(-0.1);
        assert!((tol.equal_point() - DEFAULT_TOLERANCE_POINT).abs() < epsilon);

        // 他も同様にテスト
        tol.set_equal_vector(0.001);
        assert!((tol.equal_vector() - 0.001).abs() < epsilon);
        tol.set_equal_vector(-0.001);
        assert!((tol.equal_vector() - DEFAULT_TOLERANCE_VECTOR).abs() < epsilon);

        tol.set_convergnece(0.0001);
        assert!((tol.convergence() - 0.0001).abs() < epsilon);
        tol.set_convergnece(-0.0001);
        assert!((tol.convergence() - DEFAULT_TOLERANCE_CONVERGENCE).abs() < epsilon);

        tol.set_calculation(0.00001);
        assert!((tol.calculation() - 0.00001).abs() < epsilon);
        tol.set_calculation(-0.00001);
        assert!((tol.calculation() - DEFAULT_TOLERANCE_CALCULATION).abs() < epsilon);
    }
}