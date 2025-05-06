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
}
