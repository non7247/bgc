use super::*;

#[derive(Debug)]
pub struct Plane {
    /// 3D plane
    /// Ax + By + Cz + D = 0
    pub param_a: f64,
    pub param_b: f64,
    pub param_c: f64,
    pub param_d: f64,
}

impl Plane {
    pub fn from(point: &Point, vec: &Vector, tol: &Tolerance) -> Self {
        let normal_vec = vec.normal(tol);

        Self {
            param_a: normal_vec.x,
            param_b: normal_vec.y,
            param_c: normal_vec.z,
            param_d: -(normal_vec.x * point.x + normal_vec.y * point.y + normal_vec.z * point.z),
        }
    }

    /// Calculates the distance from a point to this plane.
    ///
    /// p0(x0, y0, z0) -> Ax + By + Cz + D = 0
    ///
    /// d = (Ax0 + By0 + Cz0 + D) / S
    /// S^2 = A^2 + B^2 + C^2
    pub fn distance_to(&self, point: &Point) -> f64 {
        let s = (self.param_a.powi(2) + self.param_b.powi(2) + self.param_c.powi(2)).sqrt();

        (point.x * self.param_a + point.y * self.param_b + point.z * self.param_c + self.param_d)
            .abs() / s
    }

    /// Calculates the closest point on this plane from a point.
    ///
    /// p0(x0, y0, z0) -> Ax + By + Cz + D = 0
    ///
    /// xa = x0 + At
    /// ya = y0 + Bt
    /// za = z0 + Ct
    /// t = -(Ax0 + By0 + Cz0 + D) / (A^2 + B^2 + C^2)
    pub fn closest_point(&self, point: & Point) -> Point {
        let t = -(self.param_a * point.x
            + self.param_b * point.y
            + self.param_c * point.z
            + self.param_d)
            / (self.param_a.powi(2) + self.param_b.powi(2) + self.param_c.powi(2));

        Point::new(
            point.x + self.param_a * t,
            point.y + self.param_b * t,
            point.z + self.param_c * t
        )
    }

    pub fn contains(&self, point: &Point, tol: &Tolerance) -> bool {
        self.distance_to(point) <= tol.equal_point()
    }

    pub fn get_normal_vector(&self, tol: &Tolerance) -> Vector {
        Vector::new(self.param_a, self.param_b, self.param_c).normal(tol)
    }

    pub fn is_parallel_to(&self, other: &Plane, tol: &Tolerance) -> bool {
        let self_normal = self.get_normal_vector(tol);
        let other_normal = other.get_normal_vector(tol);
        self_normal.is_parallel_to(&other_normal, tol)
    }

    pub fn is_coplanar_with(&self, other: &Plane, tol: &Tolerance) -> bool {
        self.is_parallel_to(other, tol) && (self.param_d - other.param_d).abs() < tol.equal_point()
    }

    pub fn intersect_with_plane(
        &self,
        other: &Plane,
        tol: &Tolerance
    ) -> Result<Line, crate::BgcError> {
        let self_normal = self.get_normal_vector(tol);
        let other_normal = other.get_normal_vector(tol);

        let line_dir = self_normal.outer_product(&other_normal);
        if line_dir.length() < tol.equal_vector() {
            return Err(crate::BgcError::InvalidInput); // Planes are parallel
        }

        // To find a point on the line, we can solve a system of 3 linear equations.
        // The equations are the two plane equations and a third plane equation that is
        // perpendicular to the line of intersection and passes through the origin.
        let third_plane_normal = line_dir.normal(tol);
        let third_plane = Plane::from(&Point::origin(), &third_plane_normal, tol);

        let a1 = self.param_a;
        let b1 = self.param_b;
        let c1 = self.param_c;
        let d1 = -self.param_d;

        let a2 = other.param_a;
        let b2 = other.param_b;
        let c2 = other.param_c;
        let d2 = -other.param_d;

        let a3 = third_plane.param_a;
        let b3 = third_plane.param_b;
        let c3 = third_plane.param_c;
        let d3 = -third_plane.param_d;

        let det = a1 * (b2 * c3 - b3 * c2) - b1 * (a2 * c3 - a3 * c2) + c1 * (a2 * b3 - a3 * b2);

        if det.abs() < tol.calculation() {
            return Err(crate::BgcError::InvalidInput);
        }

        let x = (d1 * (b2 * c3 - b3 * c2)
            - b1 * (d2 * c3 - d3 * c2)
            + c1 * (d2 * b3 - d3 * b2)) / det;
        let y = (a1 * (d2 * c3 - d3 * c2)
            - d1 * (a2 * c3 - a3 * c2)
            + c1 * (a2 * d3 - a3 * d2)) / det;
        let z = (a1 * (b2 * d3 - b3 * d2)
            - b1 * (a2 * d3 - a3 * d2)
            + d1 * (a2 * b3 - a3 * b2)) / det;

        let intersection_point = Point::new(x, y, z);

        Ok(Line::new(intersection_point, intersection_point + line_dir))
    }
}
