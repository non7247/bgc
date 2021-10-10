#[derive(Debug)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point {
    pub fn origin() -> Point {
        Point { x: 0.0, y: 0.0, z: 0.0 }
    }

    pub fn distance_to(&self, rhs: &Self) -> f64 {
        let dx = self.x - rhs.x;
        let dy = self.y - rhs.y;
        let dz = self.z - rhs.z;

        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    pub fn is_equal_to(&self, rhs: &Self, tol: f64) -> bool {
        self.distance_to(rhs) < tol
    }
}
