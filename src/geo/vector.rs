use std::ops;
use super::*;
use crate::Tolerance;

#[derive(Debug, Copy, Clone)]
pub struct Vector {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn x_axis() -> Self {
        Self::new(1.0, 0.0, 0.0)
    }

    pub fn y_axis() -> Self {
        Self::new(0.0, 1.0, 0.0)
    }

    pub fn z_axis() -> Self {
        Self::new(0.0, 0.0, 1.0)
    }

    pub fn length_squared(&self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn length(&self) -> f64 {
        self.length_squared().sqrt()
    }

    pub fn is_equal_to(&self, rhs: &Self, tol: &Tolerance) -> bool {
        let diff = Self::new(
            self.x - rhs.x,
            self.y - rhs.y,
            self.z - rhs.z
        );
        diff.length() < tol.equal_vector()
    }

    pub fn is_parallel_to(&self, other: &Self, tol: &Tolerance) -> bool {
        if self.length() < tol.equal_vector() || other.length() < tol.equal_vector() {
            return false;
        }

        let ip = self.normal(tol).inner_product(&other.normal(tol));
        if (ip.abs() - 1.0).abs() < tol.equal_vector() {
            return true;
        }

        false
    }

    pub fn normal(&self, tol: &Tolerance) -> Self {
        let l = self.length();

        if l < tol.equal_vector() {
            Self::new(self.x, self.y, self.z)
        } else {
            Self::new(self.x / l, self.y / l, self.z / l)
        }
    }

    pub fn inner_product(&self, rhs: &Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    pub fn outer_product(&self, rhs: &Self) -> Self {
        Self::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x
        )
    }

    /// Calculates the angle of XY relative to the positive X-axis.
    ///
    /// # Returns
    /// The angle of the vector in radians, in the range `[0.0, 2 * PI)`.
    pub fn angle_xy(&self, tol: &Tolerance) -> f64 {
        if self.x.abs() < tol.equal_vector() && self.y.abs() < tol.equal_vector() {
            return 0.0;
        }

        let angle = self.y.atan2(self.x);
        if angle.abs() < tol.calculation() {
            0.0
        } else if angle < 0.0 {
            angle + std::f64::consts::PI * 2.0
        } else {
            angle
        }
    }

    /// Transforms this vector by the 3x3 linear submatrix of the transformation matrix.
    ///
    /// Unlike points, vectors represent direction and magnitude, so translation
    /// (the 4th column of the matrix) is not applied.
    pub fn transform(&self, mat: &Matrix3d) -> Self {
        Self::new(
            mat.get(0, 0) * self.x + mat.get(0, 1) * self.y + mat.get(0, 2) * self.z,
            mat.get(1, 0) * self.x + mat.get(1, 1) * self.y + mat.get(1, 2) * self.z,
            mat.get(2, 0) * self.x + mat.get(2, 1) * self.y + mat.get(2, 2) * self.z,
        )
    }
}

impl ops::AddAssign for Vector {
    fn add_assign(&mut self, rhs: Vector) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl ops::AddAssign<&Vector> for Vector {
    fn add_assign(&mut self, rhs: &Vector) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl ops::SubAssign for Vector {
    fn sub_assign(&mut self, rhs: Vector) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl ops::SubAssign<&Vector> for Vector {
    fn sub_assign(&mut self, rhs: &Vector) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl ops::MulAssign<f64> for Vector {
    fn mul_assign(&mut self, rhs: f64) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl ops::DivAssign<f64> for Vector {
    fn div_assign(&mut self, rhs: f64) {
        self.x /= rhs;
        self.y /= rhs;
        self.z /= rhs;
    }
}

impl ops::Add for Vector {
    type Output = Self;

    fn add(self, rhs: Vector) -> Self::Output {
        Self::new(
            self.x + rhs.x,
            self.y + rhs.y,
            self.z + rhs.z
        )
    }
}

impl ops::Add<&Vector> for Vector {
    type Output = Self;

    fn add(self, rhs: &Vector) -> Self::Output {
        self + *rhs
    }
}

impl ops::Add<Vector> for &Vector {
    type Output = Vector;

    fn add(self, rhs: Vector) -> Self::Output {
        *self + rhs
    }
}

impl ops::Add for &Vector {
    type Output = Vector;

    fn add(self, rhs: Self) -> Self::Output {
        *self + *rhs
    }
}

impl ops::Sub for Vector {
    type Output = Self;

    fn sub(self, rhs: Vector) -> Self::Output {
        Self::new(
            self.x - rhs.x,
            self.y - rhs.y,
            self.z - rhs.z
        )
    }
}

impl ops::Sub<&Vector> for Vector {
    type Output = Self;

    fn sub(self, rhs: &Vector) -> Self::Output {
        self - *rhs
    }
}

impl ops::Sub<Vector> for &Vector {
    type Output = Vector;

    fn sub(self, rhs: Vector) -> Self::Output {
        *self - rhs
    }
}

impl ops::Sub for &Vector {
    type Output = Vector;

    fn sub(self, rhs: Self) -> Self::Output {
        *self - *rhs
    }
}

impl ops::Mul<f64> for Vector {
    type Output = Self;

    fn mul(self, m: f64) -> Self::Output {
        Self::new(self.x * m, self.y * m, self.z * m)
    }
}

impl ops::Div<f64> for Vector {
    type Output = Self;
    fn div(self, rhs: f64) -> Self::Output {
        Self::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl From<Point> for Vector {
    fn from(p: Point) -> Self {
        Self::new(p.x, p.y, p.z)
    }
}

impl From<&Point> for Vector {
    fn from(p: &Point) -> Self {
        Self::new(p.x, p.y, p.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vector_is_equal_to() {
        let tol = Tolerance::default();

        let lhs = Vector::new(1.0, 2.0, 3.0);
        let rhs = Vector::new(1.0, 2.0, 3.0);

        assert!(lhs.is_equal_to(&rhs, &tol), "lhs: {:?}, rhs: {:?}", lhs, rhs);

        let rhs = Vector::new(1.1, 2.1, 3.1);

        assert!(!lhs.is_equal_to(&rhs, &tol), "lhs: {:?}, rhs: {:?}", lhs, rhs);
    }

    #[test]
    fn vector_is_parallel_to() {
        let tol = Tolerance::default();

        let lhs = Vector::new(0.0, 3.0, 4.0);
        let rhs = Vector::new(0.0, 6.0, 8.0);
        assert!(lhs.is_parallel_to(&rhs, &tol), "lhs: {:?}, rhs: {:?}", lhs, rhs);

        let lhs = Vector::new(5.0, 0.0, -2.0);
        let rhs = Vector::new(-10.0, 0.0, 4.0);
        assert!(lhs.is_parallel_to(&rhs, &tol), "lhs: {:?}, rhs: {:?}", lhs, rhs);

        let lhs = Vector::new(1.0, 2.0, 3.0);
        let rhs = Vector::new(2.0, 4.0, 6.0);
        assert!(lhs.is_parallel_to(&rhs, &tol), "lhs: {:?}, rhs: {:?}", lhs, rhs);

        let lhs = Vector::new(1.0, 2.0, 3.0);
        let rhs = Vector::new(-2.0, -4.0, -6.0);
        assert!(lhs.is_parallel_to(&rhs, &tol), "lhs: {:?}, rhs: {:?}", lhs, rhs);

        let lhs = Vector::new(7.0, 8.0, 9.0);
        let rhs = Vector::new(7.0, 8.0, 9.0);
        assert!(lhs.is_parallel_to(&rhs, &tol), "lhs: {:?}, rhs: {:?}", lhs, rhs);

        let lhs = Vector::new(0.0, 1.0, 0.0);
        let rhs = Vector::new(0.0, 0.0, 1.0);
        assert!(!lhs.is_parallel_to(&rhs, &tol), "lhs: {:?}, rhs: {:?}", lhs, rhs);

        let lhs = Vector::new(1.0, 0.0, 0.0);
        let rhs = Vector::new(0.0, 0.0, 1.0);
        assert!(!lhs.is_parallel_to(&rhs, &tol), "lhs: {:?}, rhs: {:?}", lhs, rhs);

        let lhs = Vector::new(1.0, 0.0, 0.0);
        let rhs = Vector::new(0.0, 1.0, 0.0);
        assert!(!lhs.is_parallel_to(&rhs, &tol), "lhs: {:?}, rhs: {:?}", lhs, rhs);

        let lhs = Vector::new(1.0, 2.0, 3.0);
        let rhs = Vector::new(2.000001, 4.000002, 6.000003);
        assert!(lhs.is_parallel_to(&rhs, &tol), "lhs: {:?}, rhs: {:?}", lhs, rhs);

        let lhs = Vector::new(0.0, 0.0, 0.0);
        let rhs = Vector::new(1.0, 2.0, 3.0);
        assert!(!lhs.is_parallel_to(&rhs, &tol), "lhs: {:?}, rhs: {:?}", lhs, rhs);

        let lhs = Vector::new(0.0, 0.0, 0.0);
        let rhs = Vector::new(0.0, 0.0, 0.0);
        assert!(!lhs.is_parallel_to(&rhs, &tol), "lhs: {:?}, rhs: {:?}", lhs, rhs);

        // If the dot product of normalized vectors is very close to
        // 1.0 - tol.equal_vector(), they should be parallel.
        let lhs = Vector::new(1.0, 0.0, 0.0);
        let cos_val = 1.0 - tol.equal_vector() + 1.0e-12;
        let sin_val = (1.0 - cos_val.powi(2)).sqrt();
        let rhs = Vector::new(cos_val, sin_val, 0.0);
        assert!(lhs.is_parallel_to(&rhs, &tol), "lhs: {:?}, rhs: {:?}", lhs, rhs);

        // If the dot product of normalized vectors is very close to
        // 1.0 - tol.equal_vector(), they should be non-parallel.
        let lhs = Vector::new(1.0, 0.0, 0.0);
        let cos_val = 1.0 - tol.equal_vector() - 1.0e-12;
        let sin_val = (1.0 - cos_val.powi(2)).sqrt();
        let rhs = Vector::new(cos_val, sin_val, 0.0);
        assert!(!lhs.is_parallel_to(&rhs, &tol), "lhs: {:?}, rhs: {:?}", lhs, rhs);
    }

    #[test]
    fn vector_operators() {
        let tol = Tolerance::default();

        let mut lhs = Vector::new(10.0, 10.0, 10.0);
        let rhs = Vector::new(5.0, -5.0, 0.0);

        lhs += rhs;

        assert!(lhs.is_equal_to(&Vector::new(15.0, 5.0, 10.0), &tol), "+=: {:?}", lhs);

        lhs -= rhs;

        assert!(lhs.is_equal_to(&Vector::new(10.0, 10.0, 10.0), &tol), "-=: {:?}", lhs);

        let result = lhs + rhs;
        assert!(result.is_equal_to(&Vector::new(15.0, 5.0, 10.0), &tol), "+: {:?}", result);

        let result = lhs - rhs;
        assert!(result.is_equal_to(&Vector::new(5.0, 15.0, 10.0), &tol), "-: {:?}", result);
    }

    #[test]
    fn vector_from_trait() {
        let tol = Tolerance::default();

        let mut lhs = Point::new(1.0, 2.0, 3.0);
        let rhs = Point::new(1.0, 2.0, 3.0);

        lhs += Into::<Vector>::into(rhs);

        assert!(lhs.is_equal_to(&Point::new(2.0, 4.0, 6.0), &tol));
    }

    #[test]
    fn vector_axes() {
        let tol = Tolerance::default();

        assert!(Vector::x_axis().is_equal_to(&Vector::new(1.0, 0.0, 0.0), &tol));
        assert!(Vector::y_axis().is_equal_to(&Vector::new(0.0, 1.0, 0.0), &tol));
        assert!(Vector::z_axis().is_equal_to(&Vector::new(0.0, 0.0, 1.0), &tol));
    }

    #[test]
    fn vector_length() {
        let v = Vector::new(3.0, 4.0, 0.0);
        assert!((v.length() - 5.0).abs() < 1e-12);

        let v = Vector::new(1.0, 1.0, 1.0);
        assert!((v.length() - 3.0f64.sqrt()).abs() < 1e-12);

        let v = Vector::new(0.0, 0.0, 0.0);
        assert!(v.length() == 0.0);
    }

    #[test]
    fn vector_scalar_multiplication() {
        let tol = Tolerance::default();
        let v = Vector::new(1.0, 2.0, 3.0);

        let result = v * 2.0;
        assert!(result.is_equal_to(&Vector::new(2.0, 4.0, 6.0), &tol));

        let mut v = Vector::new(1.0, 2.0, 3.0);
        v *= 0.5;
        assert!(v.is_equal_to(&Vector::new(0.5, 1.0, 1.5), &tol));
    }

    #[test]
    fn vector_normal() {
        let tol = Tolerance::default();

        let v = Vector::new(3.0, 0.0, 0.0);
        let n = v.normal(&tol);
        assert!(n.is_equal_to(&Vector::new(1.0, 0.0, 0.0), &tol));
        assert!((n.length() - 1.0).abs() < 1e-12);

        let v = Vector::new(0.0, 0.0, 0.0);
        let n = v.normal(&tol);
        assert!(n.is_equal_to(&Vector::new(0.0, 0.0, 0.0), &tol));
    }

    #[test]
    fn vector_products() {
        let tol = Tolerance::default();

        let v1 = Vector::new(1.0, 2.0, 3.0);
        let v2 = Vector::new(4.0, 5.0, 6.0);

        // Inner product: 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
        assert!((v1.inner_product(&v2) - 32.0).abs() < 1e-12);

        // Outer product: (2*6-3*5, 3*4-1*6, 1*5-2*4) = (12-15, 12-6, 5-8) = (-3, 6, -3)
        let op = v1.outer_product(&v2);
        assert!(op.is_equal_to(&Vector::new(-3.0, 6.0, -3.0), &tol));

        // Orthogonality check
        assert!(v1.inner_product(&op).abs() < 1e-12);
        assert!(v2.inner_product(&op).abs() < 1e-12);
    }

    #[test]
    fn vector_angle_xy() {
        let tol = Tolerance::default();

        let v = Vector::new(1.0, 0.0, 0.0);
        assert!((v.angle_xy(&tol) - 0.0).abs() < 1e-12);

        let v = Vector::new(0.0, 1.0, 0.0);
        assert!((v.angle_xy(&tol) - std::f64::consts::PI / 2.0).abs() < 1e-12);

        let v = Vector::new(-1.0, 0.0, 0.0);
        assert!((v.angle_xy(&tol) - std::f64::consts::PI).abs() < 1e-12);

        let v = Vector::new(0.0, -1.0, 0.0);
        assert!((v.angle_xy(&tol) - std::f64::consts::PI * 1.5).abs() < 1e-12);

        let v = Vector::new(0.0, 0.0, 1.0);
        assert!(v.angle_xy(&tol) == 0.0);
    }

    #[test]
    fn vector_scale_robustness() {
        let tol = Tolerance::default();

        // 1. Large vectors (planetary scale)
        let v1 = Vector::new(1.0e9, 0.0, 0.0);
        let v2 = Vector::new(0.0, 1.0e9, 0.0);

        assert!((v1.length() - 1.0e9).abs() < 1.0); // Absolute error might be large, but relative is small
        assert!(v1.normal(&tol).is_equal_to(&Vector::new(1.0, 0.0, 0.0), &tol));

        // Inner product of perpendicular large vectors should be near zero
        assert!(v1.inner_product(&v2).abs() < tol.calculation());

        // Outer product
        let op = v1.outer_product(&v2);
        assert!(op.is_equal_to(&Vector::new(0.0, 0.0, 1.0e18), &tol));

        // 2. Tiny vectors (sub-atomic scale)
        let v_tiny = Vector::new(1.0e-12, 1.0e-12, 1.0e-12);
        // Normalizing a vector smaller than calculation tolerance should return zero vector
        // (This depends on the implementation of normal())
        let n_tiny = v_tiny.normal(&tol);
        assert!(n_tiny.is_equal_to(&Vector::new(0.0, 0.0, 0.0), &tol));
    }

    #[test]
    fn vector_transform() {
        let tol = Tolerance::default();
        let v = Vector::new(12.0, 9.5, 4.8);

        // Matrix with rotation, scale, and translation
        let mut mat = Matrix3d::new();
        mat.set(0, 0, 0.9238);
        mat.set(0, 1, 0.2156);
        mat.set(0, 2, -0.5856);
        mat.set(0, 3, 7.2); // translation should be ignored
        mat.set(1, 0, 0.2514);
        mat.set(1, 1, -0.1132);
        mat.set(1, 2, 0.3126);
        mat.set(1, 3, 3.6); // translation should be ignored
        mat.set(2, 0, 0.1113);
        mat.set(2, 1, -0.4286);
        mat.set(2, 2, 0.8156);
        mat.set(2, 3, 4.0); // translation should be ignored
        mat.set(3, 0, 0.0);
        mat.set(3, 1, 0.0);
        mat.set(3, 2, 0.0);
        mat.set(3, 3, 1.0);

        let transformed = v.transform(&mat);
        assert!(transformed.is_equal_to(
            &Vector::new(10.32292, 3.44188, 1.17878),
            &tol
        ));

        // Identity transform preserves the vector
        let identity = Matrix3d::identity();
        assert!(v.transform(&identity).is_equal_to(&v, &tol));

        // Consistency between point difference and vector transform under local/world transforms
        let origin = Point::new(10.0, 20.0, 30.0);
        let uaxis = Vector::new(1.0, 1.0, 0.0).normal(&tol);
        let vaxis = Vector::new(-1.0, 1.0, 2.0).normal(&tol);

        let to_local = Matrix3d::transform_to_local(&origin, &uaxis, &vaxis, &tol);
        let to_world = Matrix3d::transform_to_world(&origin, &uaxis, &vaxis, &tol);

        let p1 = Point::new(14.0, 25.0, 32.0);
        let p2 = Point::new(20.0, 18.0, 45.0);
        let disp = p2 - p1;

        let disp_local = disp.transform(&to_local);
        let p1_local = p1.transform(&to_local, &tol).unwrap();
        let p2_local = p2.transform(&to_local, &tol).unwrap();

        assert!(disp_local.is_equal_to(&(p2_local - p1_local), &tol));

        let disp_world = disp_local.transform(&to_world);
        assert!(disp_world.is_equal_to(&disp, &tol));
    }
}
