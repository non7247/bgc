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
        derivatives.extend(ck.iter().skip(1).take(n_ders).copied());

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

    /// Calculates the arc length of the NURBS curve.
    pub fn length(&self, tol: &Tolerance) -> Result<f64, BgcError> {
        self.knot_spans(tol)
            .iter()
            .try_fold(0.0, |acc, &(a, b)| {
                let span_len = self.integrate_span_length(a, b, tol)?;
                Ok(acc + span_len)
            })
    }

    /// Returns a list of valid knot intervals (spans of non-zero length) where the curve is
    /// defined.
    ///
    /// # Returns
    /// `Vec<(f64, f64)>` : Pairs of (start parameter, end parameter) for each interval.
    fn knot_spans(&self, tol: &Tolerance) -> Vec<(f64, f64)> {
        let p = self.degree;
        let n = self.control_points.len();

        let mut spans = Vec::new();
        let mut current_u = self.knots[p];

        for &next_u in &self.knots[(p + 1)..=n] {
            if (next_u - current_u).abs() > tol.calculation() {
                spans.push((current_u, next_u));
                current_u = next_u;
            }
        }

        spans
    }

    /// (Evaluation point x_i, weight w_i) for 5-point Gauss-Legendre quadrature.
    /// Domain: [-1, 1]
    const GAUSS_POINTS_5: &[(f64, f64)] = &[
        (-0.9061798459386640, 0.2369268850561891),
        (-0.5384693101056831, 0.4786286704993665),
        ( 0.0000000000000000, 0.5688888888888889),
        ( 0.5384693101056831, 0.4786286704993665),
        ( 0.9061798459386640, 0.2369268850561891),
    ];

    /// Calculates curve length over the given knot interval [a, b] via Gauss integration.
    fn integrate_span_length(&self, a: f64, b: f64, tol: &Tolerance) -> Result<f64, BgcError> {
        self.integrate_span_length_adaptive(a, b, 0, tol)
    }

    fn integrate_span_length_adaptive(
        &self,
        a: f64,
        b: f64,
        depth: usize,
        tol: &Tolerance
    ) -> Result<f64, BgcError> {
        let val_single = self.gauss_5_span_length(a, b, tol)?;
        if depth >= 10 {
            return Ok(val_single);
        }
        let mid = (a + b) / 2.0;
        let val_left = self.gauss_5_span_length(a, mid, tol)?;
        let val_right = self.gauss_5_span_length(mid, b, tol)?;
        let val_double = val_left + val_right;

        if (val_single - val_double).abs() <= tol.calculation() * (1.0 + val_double.abs()) {
            Ok(val_double)
        } else {
            let left_res = self.integrate_span_length_adaptive(a, mid, depth + 1, tol)?;
            let right_res = self.integrate_span_length_adaptive(mid, b, depth + 1, tol)?;
            Ok(left_res + right_res)
        }
    }

    fn gauss_5_span_length(&self, a: f64, b: f64, tol: &Tolerance) -> Result<f64, BgcError> {
        let half_length = (b - a) / 2.0;
        let mid_point = (a + b) / 2.0;

        let mut sum = 0.0;

        for &(x_i, w_i) in Self::GAUSS_POINTS_5 {
            let u = mid_point + half_length * x_i;

            // Get 1st derivative vector (velocity vector C'(u)) at parameter u
            let (_, ders) = self.evaluate_derivatives(u, 1, tol)?;
            let v = ders[0];

            // Calculate ||C'(u)||, the magnitude of derivative vector
            let speed = v.length();
            sum += w_i * speed;
        }

        Ok(half_length * sum)
    }
}

impl Curve for NurbsCurve {
    fn intersect_with_line(
        &self,
        line: &Line,
        extends: bool,
        tol: &Tolerance
    ) -> Result<Vec<Point>, BgcError> {
        let line_dir = line.end_point - line.start_point;
        let line_len_sq = line_dir.length_squared();

        if line_len_sq <= tol.equal_point() * tol.equal_point() {
            return Err(BgcError::InvalidInput);
        }

        // Geometric search for each knot span
        let spans = self.knot_spans(tol);
        let mut intersection_points: Vec<Point> = Vec::new();
        let mut found_params: Vec<f64> = Vec::new();

        for (u_min, u_max) in spans {
            // Subdivide within the span to search for an initial solution (seed value)
            let samples = 10;
            for i in 0..samples {
                let u0 = u_min + (u_max - u_min) * (i as f64) / (samples as f64);
                let u1 = u_min + (u_max - u_min) * ((i + 1) as f64) / (samples as f64);

                let p0 = self.evaluate(u0, tol)?;
                let p1 = self.evaluate(u1, tol)?;

                // Compute minimum distance between line and line segment (p0-p1)
                let dist_sq0 = line.distance_squared_to(&p0, true, tol);
                let dist_sq1 = line.distance_squared_to(&p1, true, tol);

                // Skip if both points p0 and p1 are sufficiently far from the line
                // (no intersection)
                let max_dist_sq = 100.0 * tol.equal_point() * tol.equal_point();
                if dist_sq0 > max_dist_sq && dist_sq1 > max_dist_sq {
                    continue;
                }

                // Run Newton's method refinement only for intervals close to the line
                let mut u_guess = (u0 + u1) / 2.0;

                for _ in 0..15 {
                    let (pt, ders) = self.evaluate_derivatives(u_guess, 1, tol)?;
                    let v = pt - line.start_point;

                    // Vector rejection obtained by subtracting the parallel projection onto
                    // the line
                    let proj = line_dir * (v.inner_product(&line_dir) / line_len_sq);
                    let dist_vec = v - proj;

                    if dist_vec.length() <= tol.equal_point() {
                        // Check for duplicate solutions
                        if !found_params.iter().any(|&p| (p - u_guess).abs() <= tol.calculation()) {
                            // Line segment range check (when extends == false)
                            let t = v.inner_product(&line_dir) / line_len_sq;
                            if extends
                                || (-tol.calculation() ..= 1.0 + tol.calculation()).contains(&t)
                            {
                                found_params.push(u_guess);
                                intersection_points.push(pt);
                            }
                        }
                        break;
                    }

                    // Update parameter u step using the first derivative
                    let der = ders[0];
                    let der_proj = line_dir * (der.inner_product(&line_dir) / line_len_sq);
                    let dist_der = der - der_proj;

                    let denom = dist_der.length_squared();
                    if denom <= tol.calculation() {
                        break;
                    }

                    let delta = -dist_vec.inner_product(&line_dir) / denom;
                    u_guess += delta;

                    if u_guess < u_min - tol.calculation() || u_guess > u_max + tol.calculation() {
                        break;
                    }

                    u_guess = u_guess.clamp(u_min, u_max);
                }
            }
        }

        Ok(intersection_points)
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

        // C'(u) approx (C(u+h1) - C(u-h1)) / 2h1
        let h1 = 1e-6;
        let pt_prev1 = curve.evaluate(u - h1, &tol).unwrap();
        let pt_next1 = curve.evaluate(u + h1, &tol).unwrap();
        let fd_1_x = (pt_next1.x - pt_prev1.x) / (2.0 * h1);
        let fd_1_y = (pt_next1.y - pt_prev1.y) / (2.0 * h1);
        let fd_1_z = (pt_next1.z - pt_prev1.z) / (2.0 * h1);
        let fd_1 = Vector::new(fd_1_x, fd_1_y, fd_1_z);
        let mut tol_fd1 = Tolerance::default();
        tol_fd1.set_equal_vector(1e-5);
        assert!(d[0].is_equal_to(&fd_1, &tol_fd1));

        // C''(u) approx (C(u+h2) - 2C(u) + C(u-h2)) / h2^2
        // Optimal h for 2nd order central difference is O(eps^(1/4)) ~ 1e-4
        let h2 = 1e-4;
        let pt_prev2 = curve.evaluate(u - h2, &tol).unwrap();
        let pt_next2 = curve.evaluate(u + h2, &tol).unwrap();
        let fd_2_x = (pt_next2.x - 2.0 * pt.x + pt_prev2.x) / (h2 * h2);
        let fd_2_y = (pt_next2.y - 2.0 * pt.y + pt_prev2.y) / (h2 * h2);
        let fd_2_z = (pt_next2.z - 2.0 * pt.z + pt_prev2.z) / (h2 * h2);
        let fd_2 = Vector::new(fd_2_x, fd_2_y, fd_2_z);
        let mut tol_fd2 = Tolerance::default();
        tol_fd2.set_equal_vector(1e-4);
        assert!(d[1].is_equal_to(&fd_2, &tol_fd2));
    }

    #[test]
    fn test_nurbs_length_line() {
        let tol = Tolerance::default();
        // Straight line P0=(0,0,0) to P1=(3,4,12), length = 13.0
        let pts = vec![Point::new(0.0, 0.0, 0.0), Point::new(3.0, 4.0, 12.0)];
        let weights = vec![1.0, 1.0];
        let knots = vec![0.0, 0.0, 1.0, 1.0];
        let curve = NurbsCurve::new(1, pts, weights, knots, &tol).unwrap();

        let len = curve.length(&tol).unwrap();
        assert!((len - 13.0).abs() <= tol.equal_point());
    }

    #[test]
    fn test_nurbs_length_straight_bezier() {
        let tol = Tolerance::default();
        // Straight quadratic Bezier P0=(0,0,0), P1=(1,0,0), P2=(2,0,0), length = 2.0
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];
        let weights = vec![1.0, 1.0, 1.0];
        let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let curve = NurbsCurve::new(2, pts, weights, knots, &tol).unwrap();

        let len = curve.length(&tol).unwrap();
        assert!((len - 2.0).abs() <= tol.equal_point());
    }

    #[test]
    fn test_nurbs_length_quarter_circle() {
        let tol = Tolerance::default();
        // Quarter circle R=1. Length = PI/2 = 1.5707963267948966
        let w1 = 1.0 / 2.0f64.sqrt();
        let pts = vec![
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let weights = vec![1.0, w1, 1.0];
        let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let curve = NurbsCurve::new(2, pts, weights, knots, &tol).unwrap();

        let len = curve.length(&tol).unwrap();
        let expected_len = std::f64::consts::FRAC_PI_2;
        assert!((len - expected_len).abs() <= tol.equal_point());
    }

    #[test]
    fn test_nurbs_length_full_circle() {
        let tol = Tolerance::default();
        // Full circle R=2. Length = 2 * PI * 2 = 4 * PI
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let pts = vec![
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
            Point::new(-2.0, 2.0, 0.0),
            Point::new(-2.0, 0.0, 0.0),
            Point::new(-2.0, -2.0, 0.0),
            Point::new(0.0, -2.0, 0.0),
            Point::new(2.0, -2.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ];
        let weights = vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
        let knots = vec![
            0.0, 0.0, 0.0,
            0.25, 0.25,
            0.5, 0.5,
            0.75, 0.75,
            1.0, 1.0, 1.0,
        ];
        let curve = NurbsCurve::new(2, pts, weights, knots, &tol).unwrap();

        let len = curve.length(&tol).unwrap();
        let expected_len = 4.0 * std::f64::consts::PI;
        assert!((len - expected_len).abs() <= tol.equal_point());
    }

    #[test]
    fn test_nurbs_length_medium_scale_line() {
        let tol = Tolerance::default();
        // Line from (0,0,0) to (300, 400, 1200), length = 1300.0
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(300.0, 400.0, 1200.0),
        ];
        let weights = vec![1.0, 1.0];
        let knots = vec![0.0, 0.0, 1.0, 1.0];
        let curve = NurbsCurve::new(1, pts, weights, knots, &tol).unwrap();

        let len = curve.length(&tol).unwrap();
        assert!((len - 1300.0).abs() <= tol.equal_point());
    }

    #[test]
    fn test_nurbs_length_circle_r500() {
        let tol = Tolerance::default();
        // Circle R = 500.0. Length = 2 * PI * 500 = 1000 * PI
        let r = 500.0;
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let pts = vec![
            Point::new(r, 0.0, 0.0),
            Point::new(r, r, 0.0),
            Point::new(0.0, r, 0.0),
            Point::new(-r, r, 0.0),
            Point::new(-r, 0.0, 0.0),
            Point::new(-r, -r, 0.0),
            Point::new(0.0, -r, 0.0),
            Point::new(r, -r, 0.0),
            Point::new(r, 0.0, 0.0),
        ];
        let weights = vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
        let knots = vec![
            0.0, 0.0, 0.0,
            0.25, 0.25,
            0.5, 0.5,
            0.75, 0.75,
            1.0, 1.0, 1.0,
        ];
        let curve = NurbsCurve::new(2, pts, weights, knots, &tol).unwrap();

        let len = curve.length(&tol).unwrap();
        let expected_len = 2.0 * std::f64::consts::PI * r;
        assert!((len - expected_len).abs() <= tol.equal_point());
    }

    #[test]
    fn test_nurbs_length_translated_offset() {
        let tol = Tolerance::default();
        // Quarter circle R = 50.0 offset by (500.0, -1200.0, 300.0). Length = (PI/2) * 50 = 25 * PI
        let r = 50.0;
        let ox = 500.0;
        let oy = -1200.0;
        let oz = 300.0;
        let w1 = 1.0 / 2.0f64.sqrt();
        let pts = vec![
            Point::new(ox + r, oy, oz),
            Point::new(ox + r, oy + r, oz),
            Point::new(ox, oy + r, oz),
        ];
        let weights = vec![1.0, w1, 1.0];
        let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let curve = NurbsCurve::new(2, pts, weights, knots, &tol).unwrap();

        let len = curve.length(&tol).unwrap();
        let expected_len = std::f64::consts::FRAC_PI_2 * r;
        assert!((len - expected_len).abs() <= tol.equal_point());
    }

    #[test]
    fn test_nurbs_intersect_with_line_bezier_two_points() {
        let tol = Tolerance::default();
        // Quadratic Bezier: P0=(0,0,0), P1=(50,100,0), P2=(100,0,0)
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(50.0, 100.0, 0.0),
            Point::new(100.0, 0.0, 0.0),
        ];
        let weights = vec![1.0, 1.0, 1.0];
        let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let curve = NurbsCurve::new(2, pts, weights, knots, &tol).unwrap();

        // Line y = 37.5 from x = -50 to x = 150
        let line = Line::new(Point::new(-50.0, 37.5, 0.0), Point::new(150.0, 37.5, 0.0));
        let pts_intersect = curve.intersect_with_line(&line, false, &tol).unwrap();

        assert_eq!(pts_intersect.len(), 2);
        // Intersections expected at x = 25.0 and x = 75.0, y = 37.5, z = 0.0
        let p1 = Point::new(25.0, 37.5, 0.0);
        let p2 = Point::new(75.0, 37.5, 0.0);
        assert!(
            (pts_intersect[0].is_equal_to(&p1, &tol) && pts_intersect[1].is_equal_to(&p2, &tol))
                || (pts_intersect[0].is_equal_to(&p2, &tol) && pts_intersect[1].is_equal_to(&p1, &tol))
        );
    }

    #[test]
    fn test_nurbs_intersect_with_line_extends() {
        let tol = Tolerance::default();
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(50.0, 100.0, 0.0),
            Point::new(100.0, 0.0, 0.0),
        ];
        let weights = vec![1.0, 1.0, 1.0];
        let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let curve = NurbsCurve::new(2, pts, weights, knots, &tol).unwrap();

        // Line segment from (0, 37.5, 0) to (50, 37.5, 0)
        let line = Line::new(Point::new(0.0, 37.5, 0.0), Point::new(50.0, 37.5, 0.0));

        // extends = false: only (25, 37.5, 0) should be included
        let res_no_extend = curve.intersect_with_line(&line, false, &tol).unwrap();
        assert_eq!(res_no_extend.len(), 1);
        assert!(res_no_extend[0].is_equal_to(&Point::new(25.0, 37.5, 0.0), &tol));

        // extends = true: both (25, 37.5, 0) and (75, 37.5, 0) should be included
        let res_extend = curve.intersect_with_line(&line, true, &tol).unwrap();
        assert_eq!(res_extend.len(), 2);
    }

    #[test]
    fn test_nurbs_intersect_with_line_no_intersection() {
        let tol = Tolerance::default();
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(50.0, 100.0, 0.0),
            Point::new(100.0, 0.0, 0.0),
        ];
        let weights = vec![1.0, 1.0, 1.0];
        let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let curve = NurbsCurve::new(2, pts, weights, knots, &tol).unwrap();

        // Line y = 150.0 is above the curve's peak (y_max = 50.0)
        let line = Line::new(Point::new(-50.0, 150.0, 0.0), Point::new(150.0, 150.0, 0.0));
        let res = curve.intersect_with_line(&line, true, &tol).unwrap();
        assert!(res.is_empty());
    }

    #[test]
    fn test_nurbs_intersect_with_line_circle() {
        let tol = Tolerance::default();
        // Circle R = 100.0 offset by (100.0, 200.0, 50.0)
        let r = 100.0;
        let ox = 100.0;
        let oy = 200.0;
        let oz = 50.0;
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let pts = vec![
            Point::new(ox + r, oy, oz),
            Point::new(ox + r, oy + r, oz),
            Point::new(ox, oy + r, oz),
            Point::new(ox - r, oy + r, oz),
            Point::new(ox - r, oy, oz),
            Point::new(ox - r, oy - r, oz),
            Point::new(ox, oy - r, oz),
            Point::new(ox + r, oy - r, oz),
            Point::new(ox + r, oy, oz),
        ];
        let weights = vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
        let knots = vec![
            0.0, 0.0, 0.0,
            0.25, 0.25,
            0.5, 0.5,
            0.75, 0.75,
            1.0, 1.0, 1.0,
        ];
        let curve = NurbsCurve::new(2, pts, weights, knots, &tol).unwrap();

        // Horizontal line through circle center (y = 200.0, z = 50.0)
        let line = Line::new(Point::new(-100.0, 200.0, 50.0), Point::new(300.0, 200.0, 50.0));
        let res = curve.intersect_with_line(&line, false, &tol).unwrap();
        assert_eq!(res.len(), 2);
        let p1 = Point::new(0.0, 200.0, 50.0);
        let p2 = Point::new(200.0, 200.0, 50.0);
        assert!(
            (res[0].is_equal_to(&p1, &tol) && res[1].is_equal_to(&p2, &tol))
                || (res[0].is_equal_to(&p2, &tol) && res[1].is_equal_to(&p1, &tol))
        );
    }

    #[test]
    fn test_nurbs_intersect_with_line_tangent() {
        let tol = Tolerance::default();
        // Circle R = 100.0 offset by (100.0, 200.0, 50.0)
        let r = 100.0;
        let ox = 100.0;
        let oy = 200.0;
        let oz = 50.0;
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let pts = vec![
            Point::new(ox + r, oy, oz),
            Point::new(ox + r, oy + r, oz),
            Point::new(ox, oy + r, oz),
            Point::new(ox - r, oy + r, oz),
            Point::new(ox - r, oy, oz),
            Point::new(ox - r, oy - r, oz),
            Point::new(ox, oy - r, oz),
            Point::new(ox + r, oy - r, oz),
            Point::new(ox + r, oy, oz),
        ];
        let weights = vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
        let knots = vec![
            0.0, 0.0, 0.0,
            0.25, 0.25,
            0.5, 0.5,
            0.75, 0.75,
            1.0, 1.0, 1.0,
        ];
        let curve = NurbsCurve::new(2, pts, weights, knots, &tol).unwrap();

        // Tangent line at y = 300.0 (top of circle), z = 50.0
        let line = Line::new(Point::new(-50.0, 300.0, 50.0), Point::new(250.0, 300.0, 50.0));
        let res = curve.intersect_with_line(&line, false, &tol).unwrap();
        assert_eq!(res.len(), 1);
        assert!(res[0].is_equal_to(&Point::new(100.0, 300.0, 50.0), &tol));
    }

    #[test]
    fn test_nurbs_intersect_with_line_skew_3d() {
        let tol = Tolerance::default();
        // Quadratic Bezier in XY plane z = 0
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(50.0, 100.0, 0.0),
            Point::new(100.0, 0.0, 0.0),
        ];
        let weights = vec![1.0, 1.0, 1.0];
        let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let curve = NurbsCurve::new(2, pts, weights, knots, &tol).unwrap();

        // Line parallel to XY plane but offset in Z by 10.0 (skew line)
        let line = Line::new(Point::new(-50.0, 37.5, 10.0), Point::new(150.0, 37.5, 10.0));
        let res = curve.intersect_with_line(&line, true, &tol).unwrap();
        assert!(res.is_empty());
    }

    #[test]
    fn test_nurbs_intersect_with_line_multispan_cubic() {
        let tol = Tolerance::default();
        // Multi-span cubic NURBS curve
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(100.0, 200.0, 0.0),
            Point::new(200.0, -100.0, 0.0),
            Point::new(300.0, 200.0, 0.0),
            Point::new(400.0, 0.0, 0.0),
        ];
        let weights = vec![1.0, 1.0, 1.0, 1.0, 1.0];
        let knots = vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0];
        let curve = NurbsCurve::new(3, pts, weights, knots, &tol).unwrap();

        // Line y = 50.0 across the curve
        let line = Line::new(Point::new(-50.0, 50.0, 0.0), Point::new(450.0, 50.0, 0.0));
        let res = curve.intersect_with_line(&line, false, &tol).unwrap();
        // The wave C(u) crosses y = 50 multiple times
        assert!(res.len() >= 2);
        for pt in &res {
            assert!((pt.y - 50.0).abs() <= tol.equal_point());
        }
    }

    #[test]
    fn test_nurbs_intersect_with_line_large_scale_10k() {
        let tol = Tolerance::default();
        // Quadratic Bezier at ~10,000 scale: (0,0,5000), (10000, 20000, 5000), (20000, 0, 5000)
        let pts = vec![
            Point::new(0.0, 0.0, 5000.0),
            Point::new(10000.0, 20000.0, 5000.0),
            Point::new(20000.0, 0.0, 5000.0),
        ];
        let weights = vec![1.0, 1.0, 1.0];
        let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let curve = NurbsCurve::new(2, pts, weights, knots, &tol).unwrap();

        // Line y = 7500.0, z = 5000.0
        let line = Line::new(Point::new(-5000.0, 7500.0, 5000.0), Point::new(25000.0, 7500.0, 5000.0));
        let res = curve.intersect_with_line(&line, false, &tol).unwrap();

        assert_eq!(res.len(), 2);
        let p1 = Point::new(5000.0, 7500.0, 5000.0);
        let p2 = Point::new(15000.0, 7500.0, 5000.0);
        assert!(
            (res[0].is_equal_to(&p1, &tol) && res[1].is_equal_to(&p2, &tol))
                || (res[0].is_equal_to(&p2, &tol) && res[1].is_equal_to(&p1, &tol))
        );
    }
}