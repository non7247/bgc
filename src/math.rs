use crate::{ BgcError, Tolerance };

/// Solve a quadratic equation using the quadratic formula.
///
/// The formula is:
/// $$
/// x = \frac{-b \pm \sqrt{b^2 - 4ac}}{2a}
/// $$
///
/// # Arguments
///
/// * `a` - The coefficient of $x^2$.
/// * `b` - The convergence of $x$.
/// * `c` - The constant term.
/// * `tol` - The tolerance used for comparisons.
///
/// # Returns
///
/// * `Ok((f64, f64))` - solutions of the equation
/// * `Err(BgcError::InvalidInput)` - |a| < tolerance
/// * `Err(BgcError::MustBeNoNegative)` - b^2 - 4ac < 0.0
///
pub fn quadratic_equation(a: f64, b: f64, c: f64, tol: &Tolerance) -> Result<(f64, f64), BgcError> {
    if a.abs() <= tol.calculation() { return Err(BgcError::InvalidInput); }

    let mut discriminant = b * b - 4.0 * a * c;
    if discriminant.abs() <= tol.calculation() {
        discriminant = 0.0;
    }

    if discriminant < 0.0 {
        dbg!(discriminant);
        return Err(BgcError::MustBeNoNegative);
    }

    let result1 = (-b + discriminant.sqrt()) / (2.0 * a);
    let result2 = (-b - discriminant.sqrt()) / (2.0 * a);

    Ok((result1, result2))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quadratic_equation() {
        let tol = Tolerance::default();

        let r = quadratic_equation(1.0, -9.3, -262.3, &tol);
        match r {
            Ok((x1, x2)) => {
                assert!((x1 - 21.50).abs() < 0.01);
                assert!((x2 + 12.20).abs() < 0.01);
            },
            Err(err) => { panic!("error in test_quadratic_equation: {:?}", err); },
        }

        let r = quadratic_equation(0.0, 2.0, 4.0, &tol);
        match r {
            Ok(_) => { panic!("this test should be error."); },
            Err(err) => { assert_eq!(err, BgcError::InvalidInput); },
        }

        let r = quadratic_equation(23.2, 18.5, 97.6, &tol);
        match r {
            Ok(_) => { panic!("this test should be error."); },
            Err(err) => { assert_eq!(err, BgcError::MustBeNoNegative); },
        }

        let r = quadratic_equation(12.3, 0.2, -10256.8, &tol);
        match r {
            Ok((x1, x2)) => {
                assert!((x1 - 28.87).abs() < 0.01);
                assert!((x2 + 28.89).abs() < 0.01);
            },
            Err(err) => { panic!("error in test_quadratic_equation: {:?}", err); },
        }

        let r = quadratic_equation(739.84, -47474.88, 761605.29, &tol);
        match r {
            Ok((x1, x2)) => {
                assert!((x1 - 32.085).abs() < 0.01);
                assert!((x2 - 32.085).abs() < 0.01);
            },
            Err(err) => { panic!("error in test_quadratic_equation: {:?}", err); },
        }

        let r = quadratic_equation(1.0, 0.0, 0.0, &tol);
        match r {
            Ok((x1, x2)) => {
                assert!((x1 - 0.00).abs() < 0.01);
                assert!((x2 - 0.00).abs() < 0.01);
            },
            Err(err) => { panic!("error in test_quadratic_equation: {:?}", err); },
        }

        let r = quadratic_equation(0.2, 9987.6, -16.4, &tol);
        match r {
            Ok((x1, x2)) => {
                assert!((x1 - 0.00).abs() < 0.01);
                assert!((x2 + 49938.00).abs() < 0.01);
            },
            Err(err) => { panic!("error in test_quadratic_equation: {:?}", err); },
        }
    }
}
