use std::ops;
use super::Vector;
use crate::Tolerance;

#[derive(Debug, Copy, Clone)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point {
    pub fn origin() -> Self {
        Point { x: 0.0, y: 0.0, z: 0.0 }
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
        Self { x: self.x - rhs.x, y: self.y - rhs.y, z: self.z - rhs.z, }
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
}
