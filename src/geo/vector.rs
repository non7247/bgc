use std::ops;

#[derive(Debug, Copy, Clone)]
pub struct Vector {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector {
    pub fn x_axis() -> Vector {
        Vector { x: 1.0, y: 0.0, z: 0.0 }
    }

    pub fn y_axis() -> Vector {
        Vector { x: 0.0, y: 1.0, z: 0.0 }
    }

    pub fn z_axis() -> Vector {
        Vector { x: 0.0, y: 0.0, z: 1.0 }
    }

    pub fn length(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn is_equal_to(&self, rhs: &Self, tol: f64) -> bool {
        let diff = Vector { x: self.x - rhs.x, y: self.y - rhs.y, z: self.z - rhs.z };
        diff.length() < tol
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vector_is_equal_to() {
        let lhs = Vector { x: 1.0, y: 2.0, z: 3.0 };
        let rhs = Vector { x: 1.0, y: 2.0, z: 3.0 };

        assert!(lhs.is_equal_to(&rhs, crate::DEFAULT_TOLERANCE_VECTOR));

        let rhs = Vector { x: 1.1, y: 2.1, z: 3.1 };

        assert!(!lhs.is_equal_to(&rhs, crate::DEFAULT_TOLERANCE_VECTOR));
    }

    #[test]
    fn vector_operators() {
        let mut lhs = Vector { x: 10.0, y: 10.0, z: 10.0 };
        let rhs = Vector { x: 5.0, y: -5.0, z: 0.0 };

        lhs += rhs;

        assert!(lhs.is_equal_to(&Vector { x: 15.0, y: 5.0, z: 10.0 }, crate::DEFAULT_TOLERANCE_VECTOR));

        lhs -= rhs;

        assert!(lhs.is_equal_to(&Vector { x: 10.0, y: 10.0, z: 10.0 }, crate::DEFAULT_TOLERANCE_VECTOR));

        let result = lhs + rhs;
        assert!(result.is_equal_to(&Vector { x: 15.0, y: 5.0, z: 10.0 }, crate::DEFAULT_TOLERANCE_POINT));

        let result = lhs - rhs;
        assert!(result.is_equal_to(&Vector { x: 5.0, y: 15.0, z: 10.0 }, crate::DEFAULT_TOLERANCE_POINT));
    }
}
