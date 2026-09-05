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

/// Solves an equation using Newton's method.
///
/// $$
/// x_{n} = x_{n-1} - \frac{f(x_{n-1})}{f'(x_{n-1})}
/// $$
///
/// # Arguments
///
/// * `init_value` - The initial guess.
/// * `max_iterations` - The maximum number of iterations.
/// * `func` - The function to solve: $f(x)$.
/// * `dfunc` - The derivative of the function: $f'(x)$.
/// * `tol` - The tolerance configuration.
///
/// # Returns
///
/// * `Ok((result, func_value))` - The found root and the function value at the root.
/// * `Err(BgcError::InvalidInput)` - If `max_iterations <= 0`.
/// * `Err(BgcError::Deivergence)` - If the iteration count exceeds `max_iterations`.
/// * `Err(BgcError::MustBeNonZero)` - If the derivative value is zero or too small.
pub fn newton<F, DF>(
    init_value: f64,
    max_iterations: i32,
    func: F,
    dfunc: DF,
    tol: &Tolerance,
) -> Result<(f64, f64), BgcError>
where
    F: Fn(f64) -> f64,
    DF: Fn(f64) -> f64,
{
    if max_iterations <= 0 {
        return Err(BgcError::InvalidInput);
    }

    let mut pp = init_value;

    let fd = dfunc(init_value);
    if fd.abs() <= tol.calculation() {
        return Err(BgcError::MustBeNonZero);
    }
    let mut pn = pp - func(init_value) / fd;

    let mut it = 0;
    while (pp - pn).abs() > tol.convergence() {
        pp = pn;
        let fd = dfunc(pp);
        if fd.abs() <= tol.calculation() {
            return Err(BgcError::MustBeNonZero);
        }
        pn = pp - func(pp) / fd;

        it += 1;
        if it > max_iterations {
            return Err(BgcError::Deivergence);
        }
    }

    let val = func(pn);
    Ok((pn, val))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quadratic_equation() {
        let tol = Tolerance::default();

        let r = quadratic_equation(1.0, -9.3, -262.3, &tol);
        let Ok((x1, x2)) = r else {
            panic!("error in test_quadratic_equation: {:?}", r.unwrap_err());
        };
        assert!((x1 - 21.50).abs() < 0.01);
        assert!((x2 + 12.20).abs() < 0.01);

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
        let Ok((x1, x2)) = r else {
            panic!("error in test_quadratic_equation: {:?}", r.unwrap_err());
        };
        assert!((x1 - 28.87).abs() < 0.01);
        assert!((x2 + 28.89).abs() < 0.01);

        let r = quadratic_equation(739.84, -47474.88, 761605.29, &tol);
        let Ok((x1, x2)) = r else {
            panic!("error in test_quadratic_equation: {:?}", r.unwrap_err());
        };
        assert!((x1 - 32.085).abs() < 0.01);
        assert!((x2 - 32.085).abs() < 0.01);

        let r = quadratic_equation(1.0, 0.0, 0.0, &tol);
        let Ok((x1, x2)) = r else {
            panic!("error in test_quadratic_equation: {:?}", r.unwrap_err());
        };
        assert!((x1 - 0.00).abs() < 0.01);
        assert!((x2 - 0.00).abs() < 0.01);

        let r = quadratic_equation(0.2, 9987.6, -16.4, &tol);
        let Ok((x1, x2)) = r else {
            panic!("error in test_quadratic_equation: {:?}", r.unwrap_err());
        };
        assert!((x1 - 0.00).abs() < 0.01);
        assert!((x2 + 49938.00).abs() < 0.01);
    }

    #[test]
    fn test_newton() {
        let tol = Tolerance::default();

        // 1. Basic quadratic: Solve f(x) = x^2 - 4 = 0, root is 2.0 (and -2.0)
        let func = |x: f64| x * x - 4.0;
        let dfunc = |x: f64| 2.0 * x;

        // Try from 3.0 -> should converge to 2.0
        let r = newton(3.0, 100, func, dfunc, &tol);
        let Ok((root, val)) = r else {
            panic!("newton failed: {:?}", r.unwrap_err());
        };
        assert!((root - 2.0).abs() <= tol.convergence());
        assert!(val.abs() <= tol.convergence());

        // Try from negative guess -> should converge to -2.0
        let r_neg = newton(-3.0, 100, func, dfunc, &tol);
        let Ok((root_neg, val_neg)) = r_neg else {
            panic!("newton failed: {:?}", r_neg.unwrap_err());
        };
        assert!((root_neg - (-2.0)).abs() <= tol.convergence());
        assert!(val_neg.abs() <= tol.convergence());

        // 2. Trigonometric function: f(x) = sin(x) = 0. Root near 3.0 is pi (~3.14159265)
        let sin_func = |x: f64| x.sin();
        let cos_func = |x: f64| x.cos();
        let r_trig = newton(3.0, 100, sin_func, cos_func, &tol);
        let Ok((root_trig, val_trig)) = r_trig else {
            panic!("newton trig failed: {:?}", r_trig.unwrap_err());
        };
        assert!((root_trig - std::f64::consts::PI).abs() <= tol.convergence());
        assert!(val_trig.abs() <= tol.convergence());

        // 3. Exponential function: f(x) = e^x - 2 = 0. Root is ln(2) (~0.693147)
        let exp_func = |x: f64| x.exp() - 2.0;
        let dexp_func = |x: f64| x.exp();
        let r_exp = newton(1.0, 100, exp_func, dexp_func, &tol);
        let Ok((root_exp, val_exp)) = r_exp else {
            panic!("newton exp failed: {:?}", r_exp.unwrap_err());
        };
        assert!((root_exp - 2.0f64.ln()).abs() <= tol.convergence());
        assert!(val_exp.abs() <= tol.convergence());

        // 4. Large scales: f(x) = x - 1e6 = 0. Root is 1e6.
        let large_func = |x: f64| x - 1_000_000.0;
        let dlarge_func = |_x: f64| 1.0;
        let r_large = newton(0.0, 100, large_func, dlarge_func, &tol);
        let Ok((root_large, val_large)) = r_large else {
            panic!("newton large failed: {:?}", r_large.unwrap_err());
        };
        assert!((root_large - 1_000_000.0).abs() <= tol.convergence());
        assert!(val_large.abs() <= tol.convergence());

        // 5. Small scales: f(x) = x - 1e-6 = 0. Root is 1e-6.
        let small_func = |x: f64| x - 1e-6;
        let dsmall_func = |_x: f64| 1.0;
        let r_small = newton(0.0, 100, small_func, dsmall_func, &tol);
        let Ok((root_small, val_small)) = r_small else {
            panic!("newton small failed: {:?}", r_small.unwrap_err());
        };
        assert!((root_small - 1e-6).abs() <= tol.convergence());
        assert!(val_small.abs() <= tol.convergence());

        // 6. Divergence / No real root case: f(x) = x^2 + 1 = 0
        let no_root_func = |x: f64| x * x + 1.0;
        let dno_root_func = |x: f64| 2.0 * x;
        let r_no_root = newton(2.0, 100, no_root_func, dno_root_func, &tol);
        assert_eq!(r_no_root.unwrap_err(), BgcError::Deivergence);

        // 7. Error condition checks
        // Try with 0 max iterations -> should fail with InvalidInput
        let r_err1 = newton(3.0, 0, func, dfunc, &tol);
        assert_eq!(r_err1.unwrap_err(), BgcError::InvalidInput);

        // Try with too few iterations -> should fail with Deivergence
        let r_err2 = newton(3.0, 1, func, dfunc, &tol);
        assert_eq!(r_err2.unwrap_err(), BgcError::Deivergence);

        // Try starting at 0.0 -> derivative is 0.0 -> should fail with MustBeNonZero
        let r_err3 = newton(0.0, 100, func, dfunc, &tol);
        assert_eq!(r_err3.unwrap_err(), BgcError::MustBeNonZero);
    }
}
