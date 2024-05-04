mod point;
mod vector;
mod matrix3d;
mod line;
mod arc;
mod plane;

use crate::{ BgcError, Tolerance };

pub use point::Point as Point;
pub use vector::Vector as Vector;
pub use matrix3d::Matrix3d as Matrix3d;

pub use line::Line as Line;
pub use arc::Arc as Arc;

pub use plane::Plane as Plane;

pub trait Curve {
    fn intersect_with_line(
        &self,
        line: &Line,
        extends: bool,
        tol: &Tolerance
    ) -> Result<Vec<Point>, BgcError>;
}
