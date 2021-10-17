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
