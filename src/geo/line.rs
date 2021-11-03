use super::Point;
use super::Vector;

#[derive(Debug)]
pub struct Line {
    pub start_point: Point,
    pub end_point: Point,
}

impl Line {
    pub fn length(&self) -> f64 {
        self.start_point.distance_to(&self.end_point)
    }

    pub fn direction(&self) -> Vector {
        (self.end_point - self.start_point).normal(crate::DEFAULT_TOLERANCE_VECTOR)
    }

    /// Calculates the closest point on this curve to input point.
    ///
    /// p0(x0, y0, z0) -> (x - x1)/l = (y - y1)/m = (z - z1)/n
    ///
    /// pa(xa, ya, za) <br>
    /// xa = x1 + lt <br>
    /// ya = y1 + mt <br>
    /// za = z1 + nt <br>
    /// t = (x0 - x1)l + (y0 - y1)m + (z0 - z1)n
    pub fn get_closest_point(&self, p0: &Point, extends: bool, tol: f64) -> Point {
        if p0.is_equal_to(&self.start_point, tol) {
            return self.start_point;
        }
        if p0.is_equal_to(&self.end_point, tol) {
            return self.end_point;
        }

        let uvec = self.direction();

        let t = (p0.x - self.start_point.x) * uvec.x
              + (p0.y - self.start_point.y) * uvec.y
              + (p0.z - self.start_point.z) * uvec.z;

        let mut closest = Point { x: self.start_point.x + uvec.x * t,
                                  y: self.start_point.y + uvec.y * t,
                                  z: self.start_point.z + uvec.z * t };

        if !extends {
            let to_closest = (closest - self.start_point).normal(crate::DEFAULT_TOLERANCE_VECTOR);
            
            if to_closest.is_equal_to(&(uvec * -1.0), crate::DEFAULT_TOLERANCE_VECTOR) {
                closest = self.start_point;
            } else {
                if self.length() < closest.distance_to(&self.start_point) {
                    closest = self.end_point;
                }
            }
        }

        closest
    }

    /// Determines if input point lies on this line.
    pub fn is_on(&self, p: &Point, extends: bool, tol: f64) -> bool {
        if p.is_equal_to(&self.start_point, tol) || p.is_equal_to(&self.end_point, tol) {
            return true;
        }

        let closest = self.get_closest_point(&p, extends, tol);
        
        closest.is_equal_to(&p, tol)
    }

    /// Determines if input line is parallel to this line.
    pub fn is_parallel(&self, l: &Self, tol: f64) -> bool {
        let closest_start = l.get_closest_point(&self.start_point, true, tol);
        let closest_end = l.get_closest_point(&self.end_point, true, tol);

        let dir_start = (closest_start - self.start_point).normal(crate::DEFAULT_TOLERANCE_VECTOR);
        let dir_end = (closest_end - self.end_point).normal(crate::DEFAULT_TOLERANCE_VECTOR);

        if !dir_start.is_equal_to(&dir_end, crate::DEFAULT_TOLERANCE_VECTOR) {
            return false;
        }

        let dist_start = self.start_point.distance_to(&closest_start);
        let dist_end = self.end_point.distance_to(&closest_end);

        let dir_self = self.direction();
        let dir_other = l.direction();

        if (dist_start - dist_end).abs() <= tol &&
                (dir_self.is_equal_to(&dir_other, crate::DEFAULT_TOLERANCE_VECTOR) ||
                 dir_self.is_equal_to(&(dir_other * -1.0), crate::DEFAULT_TOLERANCE_CONVERGENCE)) {
            return true;
        }

        false
    }
}
