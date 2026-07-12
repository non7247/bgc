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

        // Evaluate at u = 1.0 -> should be end point (2,0,0)
        let p_end = curve.evaluate(1.0, &tol).unwrap();
        assert!(p_end.is_equal_to(&Point::new(2.0, 0.0, 0.0), &tol));

        // Evaluate at u = 0.5 -> should be (1.0, 1.0, 0.0)
        // B(0.5) = 0.25*(0,0,0) + 0.5*(1,2,0) + 0.25*(2,0,0) = (1.0, 1.0, 0.0)
        let p_mid = curve.evaluate(0.5, &tol).unwrap();
        assert!(p_mid.is_equal_to(&Point::new(1.0, 1.0, 0.0), &tol));
    }
}
