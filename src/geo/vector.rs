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

    pub fn length(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn is_equal_to(&self, rhs: &Self, tol: &Tolerance) -> bool {
        let diff = Self::new(
            self.x - rhs.x,
            self.y - rhs.y,
            self.z - rhs.z
        );
        diff.length() < tol.equal_vector()
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
        Self::new(
            self.x + rhs.x,
            self.y + rhs.y,
            self.z + rhs.z
        )
    }
}

impl ops::Sub for Vector {
    type Output = Self;

    fn sub(self, rhs: Vector) -> Self {
        Self::new(
            self.x - rhs.x,
            self.y - rhs.y,
            self.z - rhs.z
        )
    }
}

impl ops::Mul<f64> for Vector {
    type Output = Self;

    fn mul(self, m: f64) -> Self {
        Self::new(self.x * m, self.y * m, self.z * m)
    }
}

impl From<Point> for Vector {
    fn from(p: Point) -> Self {
        Self::new(p.x, p.y, p.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vector_is_equal_to() {
        let lhs = Vector::new(1.0, 2.0, 3.0);
        let rhs = Vector::new(1.0, 2.0, 3.0);

        assert!(lhs.is_equal_to(&rhs, &Tolerance::default()));

        let rhs = Vector::new(1.1, 2.1, 3.1);

        assert!(!lhs.is_equal_to(&rhs, &Tolerance::default()));
    }

    #[test]
    fn vector_operators() {
        let mut lhs = Vector::new(10.0, 10.0, 10.0);
        let rhs = Vector::new(5.0, -5.0, 0.0);

        lhs += rhs;

        assert!(lhs.is_equal_to(&Vector::new(15.0, 5.0, 10.0), &Tolerance::default()));

        lhs -= rhs;

        assert!(lhs.is_equal_to(&Vector::new(10.0, 10.0, 10.0), &Tolerance::default()));

        let result = lhs + rhs;
        assert!(result.is_equal_to(&Vector::new(15.0, 5.0, 10.0), &Tolerance::default()));

        let result = lhs - rhs;
        assert!(result.is_equal_to(&Vector::new(5.0, 15.0, 10.0), &Tolerance::default()));
    }

    #[test]
    fn vector_from_trait() {
        let mut lhs = Point::new(1.0, 2.0, 3.0);
        let rhs = Point::new(1.0, 2.0, 3.0);

        lhs += rhs.into();

        assert!(lhs.is_equal_to(&Point::new(2.0, 4.0, 6.0), &Tolerance::default()));
    }
}
