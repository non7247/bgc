use super::*;
use crate::{BgcError, Tolerance};

/// Non-Uniform Rational B-Spline (NURBS) Curve.
#[derive(Debug, Clone)]
pub struct NurbsCurve {
    degree: usize,
    control_points: Vec<Point>,
    weights: Vec<f64>,
    knots: Vec<f64>,
}

impl NurbsCurve {
    /// Creates a new NURBS curve.
    ///
    /// # Validation Rules
    /// - `degree` must be >= 1.
    /// - `control_points.len()` must be >= `degree + 1`.
    /// - `weights.len()` must equal `control_points.len()`.
    /// - `weights` elements must be positive.
    /// - `knots.len()` must equal `control_points.len() + degree + 1`.
    /// - `knots` must be monotonically non-decreasing.
    pub fn new(
        degree: usize,
        control_points: Vec<Point>,
        weights: Vec<f64>,
        knots: Vec<f64>,
        tol: &Tolerance,
    ) -> Result<Self, BgcError> {
        if degree < 1 {
            return Err(BgcError::InvalidInput);
        }
        if control_points.len() < degree + 1 {
            return Err(BgcError::InvalidInput);
        }
        if weights.len() != control_points.len() {
            return Err(BgcError::InvalidInput);
        }
        for &w in &weights {
            if w <= 0.0 {
                return Err(BgcError::MustBePositive);
            }
        }
        if knots.len() != control_points.len() + degree + 1 {
            return Err(BgcError::InvalidInput);
        }
        for i in 0..knots.len() - 1 {
            if knots[i] > knots[i + 1] + tol.calculation() {
                return Err(BgcError::InvalidInput);
            }
        }

        Ok(Self {
            degree,
            control_points,
            weights,
            knots,
        })
    }

    /// Returns the degree of the NURBS curve.
    pub fn degree(&self) -> usize {
        self.degree
    }

    /// Returns the control points.
    pub fn control_points(&self) -> &[Point] {
        &self.control_points
    }

    /// Returns the weights of the control points.
    pub fn weights(&self) -> &[f64] {
        &self.weights
    }

    /// Returns the knot vector.
    pub fn knots(&self) -> &[f64] {
        &self.knots
    }

    /// Finds the knot span index `k` such that `knots[k] <= u < knots[k+1]`.
    pub fn find_span(&self, u: f64, tol: &Tolerance) -> Result<usize, BgcError> {
        let p = self.degree;
        let n = self.control_points.len() - 1;

        let low = self.knots[p];
        let high = self.knots[n + 1];

        // Snap u to domain boundaries if it's within tolerance
        let mut u = u;
        if (u - low).abs() <= tol.calculation() {
            u = low;
        } else if (u - high).abs() <= tol.calculation() {
            //u = high;
            return Ok(n);
        }

        if u < low || u > high {
            return Err(BgcError::OutOfRange);
        }

        // Use Rust's standard slice::partition_point to find k such that knots[k] <= u < knots[k+1]
        let k = self.knots.partition_point(|&x| x <= u) - 1;

        Ok(k)
    }

    /// Evaluates the curve at parameter `u`.
    ///
    /// Uses De Boor's algorithm extended for rational B-splines.
    pub fn evaluate(&self, u: f64, tol: &Tolerance) -> Result<Point, BgcError> {
        let k = self.find_span(u, tol)?;
        let p = self.degree;

        // Initialize 4D points for the active control points: k-p ..= k
        let mut d = Vec::with_capacity(p + 1);
        for i in (k - p)..=k {
            let pt = self.control_points[i];
            let w = self.weights[i];
            d.push([pt.x * w, pt.y * w, pt.z * w, w]);
        }

        // De Boor's algorithm recursion
        for r in 1..=p {
            for j in (r..=p).rev() {
                let i = k - p + j;
                let denom = self.knots[i + p - r + 1] - self.knots[i];
                let alpha = if denom.abs() <= tol.calculation() {
                    0.0
                } else {
                    (u - self.knots[i]) / denom
                };

                // Interpolate in 4D
                for coord in 0..4 {
                    d[j][coord] = (1.0 - alpha) * d[j - 1][coord] + alpha * d[j][coord];
                }
            }
        }

        // Project back to 3D
        let w = d[p][3];
        if w.abs() <= tol.calculation() {
            return Err(BgcError::MustBeNonZero);
        }

        Ok(Point::new(d[p][0] / w, d[p][1] / w, d[p][2] / w))
    }

    /// Returns the start point of the NURBS curve.
    pub fn start_point(&self, tol: &Tolerance) -> Result<Point, BgcError> {
        self.evaluate(self.knots[self.degree], tol)
    }

    /// Returns the end point of the NURBS curve.
    pub fn end_point(&self, tol: &Tolerance) -> Result<Point, BgcError> {
        let n = self.control_points.len() - 1;
        self.evaluate(self.knots[n + 1], tol)
    }

    /// Evaluates the curve and its derivatives up to `max_derivatives` order at parameter `u`.
    ///
    /// # Returns
    /// - `Ok((Point, Vec<Vector>))` where:
    ///   - The first element is the point on the curve (0-th derivative).
    ///   - The second element is a vector containing the derivative vectors (1st, 2nd, etc.).
    pub fn evaluate_derivatives(
        &self,
        u: f64,
        max_derivatives: usize,
        tol: &Tolerance,
    ) -> Result<(Point, Vec<Vector>), BgcError> {
        let k = self.find_span(u, tol)?;
        let p = self.degree;
        // Clamp higher-order derivatives exceeding the degree, as they become zero
        let n_ders = max_derivatives.min(p);
        
        // ------------------------------------------------------------------
        // Step 1: Compute 0-th through n_ders-th derivatives of the B-Spline 
        //         in homogeneous coordinates (4D)
        // ------------------------------------------------------------------
        // ders_4d[k][0..4] : (x*w, y*w, z*w, w) of the k-th derivative
        let ders_4d = self.evaluate_ders_4d(k, u, n_ders)?;
        
        // ------------------------------------------------------------------
        // Step 2: Apply the rational quotient rule to convert to 3D derivatives
        // ------------------------------------------------------------------
        // A[k] : k-th derivative of the numerator (x*w, y*w, z*w)
        // w[k] : k-th derivative of the denominator (w)
        // CK[k]: Resulting k-th derivative vector in 3D space (C^(k)(u))
        let mut ck = vec![Vector::new(0.0, 0.0, 0.0); n_ders + 1];
        
        for k in 0..=n_ders {
            // Extract A_k (3D Vector) and w_k (Scalar)
            let a_k = Vector::new(ders_4d[k][0], ders_4d[k][1], ders_4d[k][2]);
            
            // Recursive term of quotient rule: sum_{i=1}^{k} binom(k, i) * w_i * C^(k-i)
            let mut sum = Vector::new(0.0, 0.0, 0.0);
            let mut binom = 1.0;    //nCr
            for i in 1..=k {
                binom = binom * (k - i + 1) as f64 / i as f64;
                sum += ck[k - i] * (binom * ders_4d[i][3]);
            }
            
            // Divide by w_0 (current weight) to get C^(k)
            let w_0 = ders_4d[0][3];
            if w_0.abs() <= tol.calculation() {
                return Err(BgcError::MustBeNonZero);
            }
            
            ck[k] = (a_k - sum) / w_0;
        }
        
        // 0-th derivative is a Point (a point on the curve); 1st and higher are Vectors
        // (derivative vectors)
        let point = Point::new(ck[0].x, ck[0].y, ck[0].z);
        let mut derivatives = Vec::with_capacity(max_derivatives);
        
        // Store derivatives from 1st to n_ders-th
        for d in 1..=n_ders {
            derivatives.push(ck[d]);
        }
        
        // Fill remaining higher-order derivatives with zero vectors if max_derivatives > degree
        for _ in (n_ders + 1)..=max_derivatives {
            derivatives.push(Vector::new(0.0, 0.0, 0.0));
        }
        
        Ok((point, derivatives))
    }

    /// Helper function: B-Spline derivative algorithm in homogeneous space (4D)
    fn evaluate_ders_4d(
        &self,
        span: usize,
        u: f64,
        n_ders: usize,
    ) -> Result<Vec<[f64; 4]>, BgcError> {
        let p = self.degree;
        let mut ders = vec![[0.0; 4]; n_ders + 1];
        
        // Table to compute derivatives of non-zero basis functions
        // (Equivalent to The NURBS Book Alg A2.3)
        // Efficiently evaluated using tables such as ndu, left, and right
        let ndu = self.calc_basis_functions_derivatives(span, u, n_ders)?;
        
        for k in 0..=n_ders {
            for j in 0..=p {
                let idx = span - p + j;
                let pt = self.control_points[idx];
                let w = self.weights[idx];
                let basis_der = ndu[k][j];
                
                ders[k][0] += basis_der * pt.x * w;
                ders[k][1] += basis_der * pt.y * w;
                ders[k][2] += basis_der * pt.z * w;
                ders[k][3] += basis_der * w;
            }
        }
        
        Ok(ders)
    }
        
    /// Calculates the B-Spline basis functions and their higher-order derivatives.
    /// (Based on Algorithm A2.2 from The NURBS Book)
    ///
    /// # Returns
    /// `ders[k][j]` : Value of the basis function corresponding to knot span `span - degree + j` 
    ///                for the `k`-th derivative.
    /// - `k` : 0 <= k <= n_ders
    /// - `j` : 0 <= j <= degree
    fn calc_basis_functions_derivatives(
        &self,
        span: usize,
        u: f64,
        n_ders: usize,
    ) -> Result<Vec<Vec<f64>>, BgcError> {
        let p = self.degree;
        let n = n_ders.min(p);
        
        // 2D array storing results ders[k][j]
        let mut ders = vec![vec![0.0; p + 1]; n_ders + 1];
        
        // Working table
        // ndu[j][r] : Upper triangular table of basis functions N_{j,r}
        let mut ndu = vec![vec![0.0; p + 1]; p + 1];
        let mut left = vec![0.0; p + 1];
        let mut right = vec![0.0; p + 1];
        
        ndu[0][0] = 1.0;
        
        // ------------------------------------------------------------------
        // Step 1: Compute basis functions N_{i,p}(u) (Equivalent to Algorithm A2.1)
        // ------------------------------------------------------------------
        for j in 1..=p {
            left[j] = u - self.knots[span + 1 - j];
            right[j] = self.knots[span + j] - u;
            let mut saved = 0.0;
            
            for r in 0..j {
                // update ndu table
                ndu[j][r] = right[r + 1] + left[j - r];
                let temp = ndu[r][j - 1] / ndu[j][r];
                
                ndu[r][j] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            ndu[j][j] = saved;
        }
        
        // Store 0-th derivative (i.e., the value of the basis function itself)
        for j in 0..=p {
            ders[0][j] = ndu[j][p];
        }
        
        // ------------------------------------------------------------------
        // Step 2: Compute derivatives (Algorithm A2.2)
        // ------------------------------------------------------------------
        // a[s1][s2] : Blending table for computing derivative coefficients
        let mut a = vec![vec![0.0; p + 1]; 2];
        
        for j in 0..=p {
            let mut s1 = 0;
            let mut s2 = 1;
            a[0][0] = 1.0;
            
            // Compute k-th derivatives in order
            for k in 1..=n {
                let mut d = 0.0;
                let pk = p - k;
                
                if j >= k {
                    let rk = j - k;
                    a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                    d = a[s2][0] * ndu[rk][pk];
                }

                let j1 = if j + 1 >= k { 1 } else { k - j };
                let j2 = if j <= pk + 1 { k - 1 } else { p - j };                
                
                for r in j1..=j2 {
                    let rk_plus_r = j + r - k;
                    a[s2][r] = (a[s1][r] - a[s1][r - 1]) / ndu[pk + 1][rk_plus_r];
                    d += a[s2][r] * ndu[rk_plus_r][pk];
                }
                
                if j <= pk {
                    a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][j];
                    d += a[s2][k] * ndu[j][pk];
                }
                
                ders[k][j] = d;
                
                // Swap s1 and s2 to reuse the table
                std::mem::swap(&mut s1, &mut s2);
            }
        }
        
        // ------------------------------------------------------------------
        // Step 3: Multiply by degree factor (factorial factor: p! / (p-k)!)
        // ------------------------------------------------------------------
        let mut r = p as f64;
        for k in 1..=n {
            for j in 0..=p {
                ders[k][j] *= r;
            }
            r *= (p -k) as f64;
        }
        
        Ok(ders)
    }
}

impl Curve for NurbsCurve {
    fn intersect_with_line(
        &self,
        _line: &Line,
        _extends: bool,
        _tol: &Tolerance
    ) -> Result<Vec<Point>, BgcError> {
        // TODO: Implement line-NURBS curve intersection
        Err(BgcError::NotImplemented)
    }

    fn intersect_with_arc(
        &self,
        _arc: &Arc,
        _extends: bool,
        _tol: &Tolerance
    ) -> Result<Vec<Point>, BgcError> {
        // TODO: Implement arc-NURBS curve intersection
        Err(BgcError::NotImplemented)
    }

    fn intersect_with_plane(
        &self,
        _plane: &Plane,
        _extends: bool,
        _tol: &Tolerance
    ) -> Result<Vec<Point>, BgcError> {
        // TODO: Implement plane-NURBS curve intersection
        Err(BgcError::NotImplemented)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nurbs_new_validation() {
        let tol = Tolerance::default();
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];
        let weights = vec![1.0, 1.0, 1.0];
        let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];

        // Valid quadratic NURBS/B-spline
        let curve = NurbsCurve::new(2, pts.clone(), weights.clone(), knots.clone(), &tol);
        assert!(curve.is_ok());

        // Invalid: degree = 0
        let curve_deg_0 = NurbsCurve::new(0, pts.clone(), weights.clone(), knots.clone(), &tol);
        assert_eq!(curve_deg_0.unwrap_err(), BgcError::InvalidInput);

        // Invalid: too few control points
        let curve_few_pts = NurbsCurve::new(3, pts.clone(), weights.clone(), knots.clone(), &tol);
        assert_eq!(curve_few_pts.unwrap_err(), BgcError::InvalidInput);

        // Invalid: mismatched weight length
        let curve_weights = NurbsCurve::new(2, pts.clone(), vec![1.0, 1.0], knots.clone(), &tol);
        assert_eq!(curve_weights.unwrap_err(), BgcError::InvalidInput);

        // Invalid: negative weight
        let curve_neg_weight = NurbsCurve::new(2, pts.clone(), vec![1.0, -1.0, 1.0], knots.clone(), &tol);
        assert_eq!(curve_neg_weight.unwrap_err(), BgcError::MustBePositive);
    }

    #[test]
    fn test_nurbs_evaluation_bezier() {
        let tol = Tolerance::default();
        // A quadratic Bezier curve as a B-spline
        // Control points: (0,0,0), (1,2,0), (2,0,0)
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];
        let weights = vec![1.0, 1.0, 1.0];
        let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];

        let curve = NurbsCurve::new(2, pts, weights, knots, &tol).unwrap();

        // Evaluate at u = 0.0 -> should be start point (0,0,0)
        let p_start = curve.evaluate(0.0, &tol).unwrap();
        assert!(p_start.is_equal_to(&Point::new(0.0, 0.0, 0.0), &tol));
        assert!(curve.start_point(&tol).unwrap().is_equal_to(&Point::new(0.0, 0.0, 0.0), &tol));

        // Evaluate at u = 1.0 -> should be end point (2,0,0)
        let p_end = curve.evaluate(1.0, &tol).unwrap();
        assert!(p_end.is_equal_to(&Point::new(2.0, 0.0, 0.0), &tol));
        assert!(curve.end_point(&tol).unwrap().is_equal_to(&Point::new(2.0, 0.0, 0.0), &tol));

        // Evaluate at u = 0.5 -> should be (1.0, 1.0, 0.0)
        // B(0.5) = 0.25*(0,0,0) + 0.5*(1,2,0) + 0.25*(2,0,0) = (1.0, 1.0, 0.0)
        let p_mid = curve.evaluate(0.5, &tol).unwrap();
        assert!(p_mid.is_equal_to(&Point::new(1.0, 1.0, 0.0), &tol));
    }

    #[test]
    fn test_nurbs_find_span() {
        let tol = Tolerance::default();
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];
        let weights = vec![1.0, 1.0, 1.0];
        let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let curve = NurbsCurve::new(2, pts, weights, knots, &tol).unwrap();

        // knots[2] = 0.0 (low), knots[3] = 1.0 (high)
        // degree = 2, n = 2.
        
        // Exact low boundary
        assert_eq!(curve.find_span(0.0, &tol).unwrap(), 2);
        // Near low boundary (within calculation tolerance)
        assert_eq!(curve.find_span(-1e-12, &tol).unwrap(), 2);
        
        // Inside domain
        assert_eq!(curve.find_span(0.5, &tol).unwrap(), 2);

        // Exact high boundary (should return n = 2)
        assert_eq!(curve.find_span(1.0, &tol).unwrap(), 2);
        // Near high boundary (within calculation tolerance)
        assert_eq!(curve.find_span(1.0 + 1e-12, &tol).unwrap(), 2);
        
        // Out of range (beyond tolerance)
        assert!(curve.find_span(-1e-5, &tol).is_err());
        assert!(curve.find_span(1.0 + 1e-5, &tol).is_err());
    }

    #[test]
    fn test_nurbs_derivatives_line() {
        let tol = Tolerance::default();
        // A straight line: degree 1
        // P0 = (0, 0, 0), P1 = (3, 4, 12)
        let pts = vec![Point::new(0.0, 0.0, 0.0), Point::new(3.0, 4.0, 12.0)];
        let weights = vec![1.0, 1.0];
        let knots = vec![0.0, 0.0, 1.0, 1.0]; // degree 1

        let curve = NurbsCurve::new(1, pts, weights, knots, &tol).unwrap();

        // Evaluate at u = 0.4
        // C(0.4) = (1.2, 1.6, 4.8)
        // C'(0.4) = (3, 4, 12)
        // C''(0.4) = (0, 0, 0)
        let (pt, derivs) = curve.evaluate_derivatives(0.4, 2, &tol).unwrap();
        assert!(pt.is_equal_to(&Point::new(1.2, 1.6, 4.8), &tol));
        assert_eq!(derivs.len(), 2);
        assert!(derivs[0].is_equal_to(&Vector::new(3.0, 4.0, 12.0), &tol));
        assert!(derivs[1].is_equal_to(&Vector::new(0.0, 0.0, 0.0), &tol));
    }

    #[test]
    fn test_nurbs_derivatives_bezier() {
        let tol = Tolerance::default();
        // Quadratic Bezier: C(u) = (1-u)^2 P0 + 2u(1-u) P1 + u^2 P2
        // C'(u) = 2(1-u)(P1-P0) + 2u(P2-P1)
        // C''(u) = 2(P2 - 2P1 + P0)
        // Let P0 = (0,0,0), P1 = (1,2,0), P2 = (2,0,0)
        // C(u) = (2u, 4u(1-u), 0)
        // C'(u) = (2, 4 - 8u, 0)
        // C''(u) = (0, -8, 0)
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];
        let weights = vec![1.0, 1.0, 1.0];
        let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let curve = NurbsCurve::new(2, pts, weights, knots, &tol).unwrap();

        // Test at u = 0.0
        let (p0, d0) = curve.evaluate_derivatives(0.0, 2, &tol).unwrap();
        assert!(p0.is_equal_to(&Point::new(0.0, 0.0, 0.0), &tol));
        assert!(d0[0].is_equal_to(&Vector::new(2.0, 4.0, 0.0), &tol));
        assert!(d0[1].is_equal_to(&Vector::new(0.0, -8.0, 0.0), &tol));

        // Test at u = 0.5
        let (p5, d5) = curve.evaluate_derivatives(0.5, 2, &tol).unwrap();
        assert!(p5.is_equal_to(&Point::new(1.0, 1.0, 0.0), &tol));
        assert!(d5[0].is_equal_to(&Vector::new(2.0, 0.0, 0.0), &tol));
        assert!(d5[1].is_equal_to(&Vector::new(0.0, -8.0, 0.0), &tol));

        // Test at u = 1.0
        let (p1, d1) = curve.evaluate_derivatives(1.0, 2, &tol).unwrap();
        assert!(p1.is_equal_to(&Point::new(2.0, 0.0, 0.0), &tol));
        assert!(d1[0].is_equal_to(&Vector::new(2.0, -4.0, 0.0), &tol));
        assert!(d1[1].is_equal_to(&Vector::new(0.0, -8.0, 0.0), &tol));
    }

    #[test]
    fn test_nurbs_derivatives_rational_circle() {
        let tol = Tolerance::default();
        // Quarter circle: R = 1.
        // P0 = (1, 0, 0), P1 = (1, 1, 0), P2 = (0, 1, 0)
        // w0 = 1, w1 = 1/sqrt(2), w2 = 1
        let w1 = 1.0 / 2.0f64.sqrt();
        let pts = vec![
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let weights = vec![1.0, w1, 1.0];
        let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let curve = NurbsCurve::new(2, pts, weights, knots, &tol).unwrap();

        // Evaluate at u = 0.5.
        // The point should be on the unit circle: x^2 + y^2 = 1.
        // The tangent vector should be orthogonal to the radius vector.
        let (p, d) = curve.evaluate_derivatives(0.5, 1, &tol).unwrap();
        
        let dist_from_origin = (p.x * p.x + p.y * p.y + p.z * p.z).sqrt();
        assert!((dist_from_origin - 1.0).abs() <= tol.convergence());

        let radius_vec = Vector::new(p.x, p.y, p.z);
        let tangent = d[0];
        let dot = radius_vec.x * tangent.x + radius_vec.y * tangent.y + radius_vec.z * tangent.z;
        assert!(dot.abs() <= tol.convergence());
    }

    #[test]
    fn test_nurbs_derivatives_numerical() {
        let tol = Tolerance::default();
        // A complex cubic NURBS curve
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 3.0, -1.0),
            Point::new(2.0, -1.0, 4.0),
            Point::new(4.0, 2.0, 1.0),
            Point::new(5.0, 0.0, 0.0),
        ];
        let weights = vec![1.0, 1.2, 0.8, 1.1, 0.9];
        let knots = vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0];
        let curve = NurbsCurve::new(3, pts, weights, knots, &tol).unwrap();

        // Compare analytic derivative with central differences at u = 0.3
        let u = 0.3;
        let (pt, d) = curve.evaluate_derivatives(u, 2, &tol).unwrap();

        let h = 1e-6;
        let pt_prev = curve.evaluate(u - h, &tol).unwrap();
        let pt_next = curve.evaluate(u + h, &tol).unwrap();
        
        // C'(u) approx (C(u+h) - C(u-h)) / 2h
        let fd_1_x = (pt_next.x - pt_prev.x) / (2.0 * h);
        let fd_1_y = (pt_next.y - pt_prev.y) / (2.0 * h);
        let fd_1_z = (pt_next.z - pt_prev.z) / (2.0 * h);
        let fd_1 = Vector::new(fd_1_x, fd_1_y, fd_1_z);
        let mut tol_fd1 = Tolerance::default();
        tol_fd1.set_equal_vector(1e-5);
        assert!(d[0].is_equal_to(&fd_1, &tol_fd1));

        // C''(u) approx (C(u+h) - 2C(u) + C(u-h)) / h^2
        let fd_2_x = (pt_next.x - 2.0 * pt.x + pt_prev.x) / (h * h);
        let fd_2_y = (pt_next.y - 2.0 * pt.y + pt_prev.y) / (h * h);
        let fd_2_z = (pt_next.z - 2.0 * pt.z + pt_prev.z) / (h * h);
        let fd_2 = Vector::new(fd_2_x, fd_2_y, fd_2_z);
        let mut tol_fd2 = Tolerance::default();
        tol_fd2.set_equal_vector(1e-4);
        assert!(d[1].is_equal_to(&fd_2, &tol_fd2));
    }
}
