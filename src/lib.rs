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

}
