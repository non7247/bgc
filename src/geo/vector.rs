use std::ops;
use crate::Tolerance;

#[derive(Debug, Copy, Clone)]
pub struct Vector {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector {
    pub fn x_axis() -> Self {
        Self { x: 1.0, y: 0.0, z: 0.0 }
    }

    pub fn y_axis() -> Self {
        Self { x: 0.0, y: 1.0, z: 0.0 }
    }

    pub fn z_axis() -> Self {
        Self { x: 0.0, y: 0.0, z: 1.0 }
    }

    pub fn length(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn is_equal_to(&self, rhs: &Self, tol: &Tolerance) -> bool {
        let diff = Self { x: self.x - rhs.x, y: self.y - rhs.y, z: self.z - rhs.z };
        diff.length() < tol.equal_vector()
    }

    pub fn normal(&self, tol: &Tolerance) -> Self {
        let l = self.length();

        if l < tol.equal_vector() {
            Self { x: self.x, y: self.y, z: self.z }
        } else {
            Self { x: self.x / l, y: self.y / l, z: self.z / l }
        }
    }

    pub fn inner_product(&self, rhs: &Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    pub fn outer_product(&self, rhs: &Self) -> Self {
        Self { x: self.y * rhs.z - self.z * rhs.y,
               y: self.z * rhs.x - self.x * rhs.z,
               z: self.x * rhs.y - self.y * rhs.x }
    }
}

impl ops::AddAssign for Vector {
    fn add_assign(&mut self, rhs: Vector) {
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

impl ops::MulAssign<f64> for Vector {
    fn mul_assign(&mut self, rhs: f64) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl ops::Add for Vector {
    type Output = Self;

    fn add(self, rhs: Vector) -> Self {
        Self { x: self.x + rhs.x, y: self.y + rhs.y, z: self.z + rhs.z }
    }
}

impl ops::Sub for Vector {
    type Output = Self;

    fn sub(self, rhs: Vector) -> Self {
        Self { x: self.x - rhs.x, y: self.y - rhs.y, z: self.z - rhs.z }
    }
}

impl ops::Mul<f64> for Vector {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        Self { x: self.x * rhs, y: self.y * rhs, z: self.z * rhs }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vector_is_equal_to() {
        let lhs = Vector { x: 1.0, y: 2.0, z: 3.0 };
        let rhs = Vector { x: 1.0, y: 2.0, z: 3.0 };

        assert!(lhs.is_equal_to(&rhs, &Tolerance::default()));

        let rhs = Vector { x: 1.1, y: 2.1, z: 3.1 };

        assert!(!lhs.is_equal_to(&rhs, &Tolerance::default()));
    }

    #[test]
    fn vector_operators() {
        let mut lhs = Vector { x: 10.0, y: 10.0, z: 10.0 };
        let rhs = Vector { x: 5.0, y: -5.0, z: 0.0 };

        lhs += rhs;

        assert!(lhs.is_equal_to(&Vector { x: 15.0, y: 5.0, z: 10.0 }, &Tolerance::default()));

        lhs -= rhs;

        assert!(lhs.is_equal_to(&Vector { x: 10.0, y: 10.0, z: 10.0 }, &Tolerance::default()));

        let result = lhs + rhs;
        assert!(result.is_equal_to(&Vector { x: 15.0, y: 5.0, z: 10.0 }, &Tolerance::default()));

        let result = lhs - rhs;
        assert!(result.is_equal_to(&Vector { x: 5.0, y: 15.0, z: 10.0 }, &Tolerance::default()));
    }
}
