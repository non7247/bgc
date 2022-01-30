mod point;
mod vector;
mod line;
mod arc;

use crate::{ ErrorStatus, Tolerance };

pub use point::Point as Point;
pub use vector::Vector as Vector;

pub use line::Line as Line;
pub use arc::Arc as Arc;

pub trait Curve {
    fn intersect_with_line(&self, line: &Line, extends: bool, tol: &Tolerance)
            -> Result<Vec<Point>, ErrorStatus>;
}
