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
        Err(BgcError::NotImplemented)
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
