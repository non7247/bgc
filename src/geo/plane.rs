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
    pub fn from(point: &Point, normal_vec: &Vector) -> Self {
        Self { param_a: normal_vec.x,
               param_b: normal_vec.y,
               param_c: normal_vec.z,
               param_d: -(normal_vec.x * point.x
                          + normal_vec.y * point.y
                          + normal_vec.z * point.z)}
    }

    /// Calculates the distance from a point to this plane.
    ///
    /// p0(x0, y0, z0) -> Ax + By + Cz + D = 0
    ///
    /// d = (Ax0 + By0 + Cz0 + D) / S
    /// S^2 = A^2 + B^2 + C^2
    pub fn distance_to(&self, point: &Point) -> f64 {
        let s = (self.param_a.powi(2) + self.param_b.powi(2) + self.param_c.powi(2)).sqrt();

        (point.x * self.param_a + point.y * self.param_b + point.z * self.param_c
         + self.param_d) / s
    }

    /// Calculates the closest point on this plane from a point.
    ///
    /// p0(x0, y0, z0) -> Ax + By + Cz + D = 0
    ///
    /// xa = x0 + At
    /// ya = y0 + Bt
    /// za = z0 + Ct
    /// t = -(Ax0 + By0 + Cz0 + D) / (A^2 + B^2 + C^2)
    pub fn get_closest_point(&self, point: & Point) -> Point {
        let t = -(self.param_a * point.x + self.param_b * point.y + self.param_c * point.z
                  + self.param_d)
                / (self.param_a.powi(2) + self.param_b.powi(2) + self.param_c.powi(2));

        Point { x: point.x + self.param_a * t,
                y: point.y + self.param_b * t,
                z: point.z + self.param_c * t }
    }

    pub fn is_on(&self, point: &Point, tol: &Tolerance) -> bool {
        self.distance_to(point) <= tol.equal_point()
    }
}
