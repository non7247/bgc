use std::ops;
use super::*;
use crate::Tolerance;

#[derive(Debug, Copy, Clone)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point {
    pub fn origin() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0 }
    }

    pub fn distance_to(&self, rhs: &Self) -> f64 {
        let dx = self.x - rhs.x;
        let dy = self.y - rhs.y;
        let dz = self.z - rhs.z;

        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    pub fn is_equal_to(&self, rhs: &Self, tol: &Tolerance) -> bool {
        self.distance_to(rhs) < tol.equal_point()
    }

    pub fn calc_middle_point(&self, rhs: &Self) -> Self {
        Self { x: (self.x + rhs.x) / 2.0,
               y: (self.y + rhs.y) / 2.0,
               z: (self.z + rhs.z) / 2.0 }
    }

    /// Transforms this point to the coordinate system of the transformation matrix
    ///
    /// \[p\] * \[M\] = \[p'\]
    pub fn transform(&self, rhs: &Matrix3d) -> Self {
        let x = self.x * rhs.get(0, 0) + self.y * rhs.get(1, 0) + self.z * rhs.get(2, 0)
              + rhs.get(3, 0);
        let y = self.x * rhs.get(0, 1) + self.y * rhs.get(1, 1) + self.z * rhs.get(2, 1)
              + rhs.get(3, 1);
        let z = self.x * rhs.get(0, 2) + self.y * rhs.get(1, 2) + self.z * rhs.get(2, 2)
              + rhs.get(3, 2);

        Self { x, y, z }
    }
}

impl ops::AddAssign<Vector> for Point {
    fn add_assign(&mut self, rhs: Vector) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl ops::SubAssign<Vector> for Point {
    fn sub_assign(&mut self, rhs: Vector) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl ops::Add<Vector> for Point {
    type Output = Self;

    fn add(self, rhs: Vector) -> Self {
        Self { x: self.x + rhs.x, y: self.y + rhs.y, z: self.z + rhs.z, }
    }
}

impl ops::Sub for Point {
    type Output = Vector;

    fn sub(self, rhs: Self) -> Vector {
        Vector { x: self.x - rhs.x, y: self.y - rhs.y, z: self.z - rhs.z }
    }
}

impl ops::Sub<Vector> for Point {
    type Output = Self;

    fn sub(self, rhs: Vector) -> Self {
        Self { x: self.x - rhs.x, y: self.y - rhs.y, z: self.z - rhs.z }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_is_equal_to() {
        let lhs = Point { x: 1.0, y: 2.0, z: 3.0 };
        let rhs = Point { x: 1.0, y: 2.0, z: 3.0 };

        assert!(lhs.is_equal_to(&rhs, &Tolerance::default()));

        let rhs = Point { x: 1.1, y: 2.1, z: 3.1 };

        assert!(!lhs.is_equal_to(&rhs, &Tolerance::default()));
    }

    #[test]
    fn point_operators() {
        let mut lhs = Point { x: 10.0, y: 10.0, z: 10.0 };
        let rhs = Vector { x: 5.0, y: -5.0, z: 0.0 };

        lhs += rhs;
        assert!(lhs.is_equal_to(&Point { x: 15.0, y: 5.0, z: 10.0 }, &Tolerance::default()));

        lhs -= rhs;
        assert!(lhs.is_equal_to(&Point { x: 10.0, y: 10.0, z: 10.0 }, &Tolerance::default()));

        let result = lhs + rhs;
        assert!(result.is_equal_to(&Point { x: 15.0, y: 5.0, z: 10.0 }, &Tolerance::default()));

        let result = lhs - rhs;
        assert!(result.is_equal_to(&Point { x: 5.0, y: 15.0, z: 10.0 }, &Tolerance::default()));

        let p = Point { x: 5.0, y: -5.0, z: 0.0 };
        let result = lhs - p;
        assert!(result.is_equal_to(&Vector { x: 5.0, y: 15.0, z: 10.0 }, &Tolerance::default()));
    }

    #[test]
    fn point_transform() {
        let  p = Point { x: 12.0, y: 9.5, z: 4.8 };

        let mut mat = Matrix3d::new();
        mat.set(0, 0, 0.9238);
        mat.set(0, 1, 0.2514);
        mat.set(0, 2, 0.1113);
        mat.set(0, 3, 0.0);
        mat.set(1, 0, 0.2156);
        mat.set(1, 1, -0.1132);
        mat.set(1, 2, -0.4286);
        mat.set(1, 3, 0.0);
        mat.set(2, 0, -0.5856);
        mat.set(2, 1, 0.3126);
        mat.set(2, 2, 0.8156);
        mat.set(2, 3, 0.0);
        mat.set(3, 0, 7.2);
        mat.set(3, 1, 3.6);
        mat.set(3, 2, 4.0);
        mat.set(3, 3, 1.0);

        let transformed = p.transform(&mat);
        assert!(transformed.is_equal_to(&Point { x: 17.5229, y: 7.0419, z: 5.1788 },
                                        &Tolerance::default()));
    }
}
