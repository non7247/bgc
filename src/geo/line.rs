use super::Point;

#[derive(Debug)]
pub struct Line {
    pub start_point: Point,
    pub end_point: Point,
}

impl Line {
    pub fn length(&self) -> f64 {
        self.start_point.distance_to(&self.end_point)
    }
}
