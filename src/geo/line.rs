use super::Point;
use super::Vector;
use crate::{ ErrorStatus, Tolerance };

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
        (self.end_point - self.start_point).normal(&Tolerance::default())
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
    pub fn get_closest_point(&self, p0: &Point, extends: bool, tol: &Tolerance) -> Point {
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
            let to_closest = (closest - self.start_point).normal(tol);
            
            if to_closest.is_equal_to(&(uvec * -1.0), tol) {
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
    pub fn is_on(&self, p: &Point, extends: bool, tol: &Tolerance) -> bool {
        if p.is_equal_to(&self.start_point, tol) || p.is_equal_to(&self.end_point, tol) {
            return true;
        }

        let closest = self.get_closest_point(&p, extends, tol);
        
        closest.is_equal_to(&p, tol)
    }

    /// Determines if input line is parallel to this line.
    pub fn is_parallel(&self, l: &Self, tol: &Tolerance) -> bool {
        let closest_start = l.get_closest_point(&self.start_point, true, tol);
        let closest_end = l.get_closest_point(&self.end_point, true, tol);

        let dir_start = (closest_start - self.start_point).normal(tol);
        let dir_end = (closest_end - self.end_point).normal(tol);

        if !dir_start.is_equal_to(&dir_end, tol) {
            return false;
        }

        let dist_start = self.start_point.distance_to(&closest_start);
        let dist_end = self.end_point.distance_to(&closest_end);

        let dir_self = self.direction();
        let dir_other = l.direction();

        (dist_start - dist_end).abs() <= tol.equal_point() &&
        (dir_self.is_equal_to(&dir_other, tol) ||
         dir_self.is_equal_to(&(dir_other * -1.0), tol))
    }

    /// Calculates an intersection point of two lines
    /// 
    /// line1 = (x - x1)/l1 = (y - y1)/m1 = (z - z1)/n1 ... this line <br>
    /// line2 = (x - x2)/l2 = (y - y2)/m2 = (z - z2)/n2 ... other line
    ///
    /// ix1 = x1 + l1\*L1, iy1 = y1 + m1\*L1, iz1 = z1 + n1\*L1 ... intersection point on line1 <br>
    /// ix2 = x2 + l2\*L2, iy2 = y2 + m2\*L2, iz2 = z2 + n2\*L2 ... intersection point on line2
    ///
    /// L1 = (S2\*Q + S1)/(1 - Q^2) <br>
    /// L2 = (S1\*Q + S2)/(1 - Q^2) <br>
    /// Q = l1\*l2 + m1\*m2 + n1\*n2 <br>
    /// S1 = l1\*X + m1\*Y +n1\*Z <br>
    /// S2 = -(l2\*X + m2\*Y + n2\*Z) <br>
    /// X = x2 - x1, Y = y2 - y1, Z = z2 - z1
    pub fn intersect_with_line(&self, other: &Self, extends: bool, tol: &Tolerance) -> Result<Point, ErrorStatus> {
        if self.start_point.is_equal_to(&other.start_point, tol) ||
                self.start_point.is_equal_to(&other.end_point, tol) {
            return Ok(self.start_point);
        }
        if self.end_point.is_equal_to(&other.start_point, tol) ||
                self.end_point.is_equal_to(&other.end_point, tol) {
            return Ok(self.end_point);
        }

        if self.is_parallel(&other, tol) {
            return Err(ErrorStatus::MustBeNonZero);
        }

        let dir1 = self.direction();
        let dir2 = other.direction();

        let q = dir1.inner_product(&dir2);

        let start_to_start = other.start_point - self.start_point;

        let s1 = dir1.inner_product(&start_to_start);
        let s2 = -(dir2.inner_product(&start_to_start));

        let l1 = (s2 * q + s1) / (1.0 - q * q);
        let l2 = (s1 * q + s2) / (1.0 - q * q);

        let int_p1 = self.start_point + dir1 * l1;
        let int_p2 = other.start_point + dir2 * l2;

        if !self.is_on(&int_p1, extends, tol) || !other.is_on(&int_p2, extends, tol) {
            return Err(ErrorStatus::InvalidInput);
        }

        if int_p1.is_equal_to(&int_p2, tol) {
            return Ok(int_p1);
        }

        Err(ErrorStatus::InvalidInput)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn line_length() {
        let l = Line { start_point: Point { x: 0.0, y: 0.0, z: 0.0},
                       end_point: Point { x: 1.0, y: 1.0, z: 0.0} };
        assert!((l.length() - 2.0_f64.sqrt()).abs() < Tolerance::default().equal_point());
    }

    #[test]
    fn line_get_closest_point() {
        let l = Line { start_point: Point { x: 1379.591836, y: 1159.400383, z: 0.0 },
                       end_point: Point { x: 3079.683229, y: 2067.058311, z: 0.0 } };

        let p = l.get_closest_point(&Point { x: 3908.885031, y: 1901.285447, z: 0.0 },
                                    false, &Tolerance::default());
        assert!(p.is_equal_to(&Point { x: 3079.683229, y: 2067.058311, z: 0.0 },
                              &Tolerance::default()));
        let p = l.get_closest_point(&Point { x: 3908.885031, y: 1901.285447, z: 0.0 },
                                    true, &Tolerance::default());
        assert!(p.is_equal_to(&Point { x: 3656.085482, y: 2374.792398, z: 0.0 },
                              &Tolerance::default()));

        let p = l.get_closest_point(&Point { x: 569.433291, y: 1366.238184, z: 0.0 },
                                    false, &Tolerance::default());
        assert!(p.is_equal_to(&Point { x: 1379.591836, y: 1159.400383, z: 0.0 },
                              &Tolerance::default()));
        let p = l.get_closest_point(&Point { x: 569.433291, y: 1366.238184, z: 0.0 },
                                    true, &Tolerance::default());
        assert!(p.is_equal_to(&Point { x: 835.069873, y: 868.686791, z: 0.0 },
                              &Tolerance::default()));
    }

    #[test]
    fn line_is_on() {
        let l = Line { start_point: Point { x: -26.0564, y: -13.8449, z: 0.0 },
                       end_point: Point { x: 44.2176, y: 19.9981, z: 0.0 } };

        assert!(l.is_on(&Point { x: 0.2074, y: -1.1966, z: 0.0 }, true, &Tolerance::default()));
        assert!(l.is_on(&Point { x: -26.0564, y: -13.8449, z: 0.0 }, true, &Tolerance::default()));
        assert!(l.is_on(&Point { x: 44.2176, y: 19.9981, z: 0.0 }, true, &Tolerance::default()));

        assert!(!l.is_on(&Point { x: -35.0660, y: -18.1838, z: 0.0 }, false, &Tolerance::default()));
        assert!(l.is_on(&Point { x: -35.0660, y: -18.1838, z: 0.0 }, true, &Tolerance::default()));
        assert!(!l.is_on(&Point { x: 57.7321, y: 26.5065, z: 0.0 }, false, &Tolerance::default()));
        assert!(l.is_on(&Point { x: 57.7321, y: 26.5065, z: 0.0 }, true, &Tolerance::default()));

        assert!(!l.is_on(&Point { x: -12.6810, y: -2.9175, z: 0.0}, true, &Tolerance::default()));
        assert!(!l.is_on(&Point { x: 18.7406, y: 5.9941, z: 0.0}, true, &Tolerance::default()));
    }

    #[test]
    fn line_intersect_with_line_xy_axis_first_quadrant() {
        let l1 = Line { start_point: Point { x: 1.0, y: 1.0, z: 0.0 },
                        end_point: Point { x: 7.0, y: 7.0, z: 0.0 } };
        let l2 = Line { start_point: Point { x: 2.0, y: 6.0, z: 0.0 },
                        end_point: Point { x: 6.0, y: 1.0, z: 0.0 } };

        let p = l1.intersect_with_line(&l2, false, &Tolerance::default());
        
        match p {
            Ok(ip) => {
                assert!(ip.is_equal_to(&Point { x: 34.0/9.0, y: 34.0/9.0, z: 0.0 }, &Tolerance::default()));
            },
            Err(error) => {
                panic!("error in intersect_with_line: {:?}", error)
            },
        };
    }

    #[test]
    fn line_intersect_with_line_xy_axis_first_quadrant_fail() {
        let l1 = Line { start_point: Point { x: 1.0, y: 1.0, z: 0.0 },
                        end_point: Point { x: 7.0, y: 7.0, z: 0.0 } };
        let l2 = Line { start_point: Point { x: 6.0, y: 1.0, z: 0.0 },
                        end_point: Point { x: 6.0, y: 1.0, z: 0.0 } };

        let p = l1.intersect_with_line(&l2, false, &Tolerance::default());

        match p {
            Ok(_ip) => {
                panic!("this test should be error.");
            },
            Err(error) => {
                assert_eq!(error, ErrorStatus::InvalidInput);
            },
        };
    }

    #[test]
    fn line_intersect_with_line_xy_axis_second_quadrant() {
        let l1 = Line { start_point: Point { x: -4.0, y: 4.0, z: 0.0 },
                        end_point: Point { x: -1.0, y: 1.0, z: 0.0 } };
        let l2 = Line { start_point: Point { x: -3.0, y: 1.0, z: 0.0 },
                        end_point: Point { x: -1.0, y: 3.0, z: 0.0 } };

        let p = l1.intersect_with_line(&l2, false, &Tolerance::default());

        match p {
            Ok(ip) => {
                assert!(ip.is_equal_to(&Point { x: -2.0, y: 2.0, z: 0.0 }, &Tolerance::default()))
            },
            Err(error) => {
                panic!("error in intersect_with_line: {:?}", error)
            },
        };
    }

    #[test]
    fn line_intersect_with_line_xy_axis_third_quadrant() {
        let l1 = Line { start_point: Point { x: -4.0, y: -4.0, z: 0.0 },
                        end_point: Point { x: -1.0, y: -1.0, z: 0.0 } };
        let l2 = Line { start_point: Point { x: -3.0, y: -1.0, z: 0.0 },
                        end_point: Point { x: -1.0, y: -3.0, z: 0.0 } };

        let p = l1.intersect_with_line(&l2, false, &Tolerance::default());

        match p {
            Ok(ip) => {
                assert!(ip.is_equal_to(&Point { x: -2.0, y: -2.0, z: 0.0 }, &Tolerance::default()))
            },
            Err(error) => {
                panic!("error in intersect_with_line: {:?}", error)
            },
        };
    }

    #[test]
    fn line_intersect_with_line_xy_axis_fourth_quadrant() {
        let l1 = Line { start_point: Point { x: 4.0, y: -4.0, z: 0.0 },
                        end_point: Point { x: 1.0, y: -1.0, z: 0.0 } };
        let l2 = Line { start_point: Point { x: 3.0, y: -1.0, z: 0.0 },
                        end_point: Point { x: 1.0, y: -3.0, z: 0.0 } };

        let p = l1.intersect_with_line(&l2, false, &Tolerance::default());

        match p {
            Ok(ip) => {
                assert!(ip.is_equal_to(&Point { x: 2.0, y: -2.0, z: 0.0 }, &Tolerance::default()))
            },
            Err(error) => {
                panic!("error in intersect_with_line: {:?}", error)
            },
        };
    }

    #[test]
    fn line_intersect_with_line_yz_axis() {
        let l1 = Line { start_point: Point { x: 0.0, y: 4.0, z: -4.0 },
                        end_point: Point { x: 0.0, y: 1.0, z: -1.0 } };
        let l2 = Line { start_point: Point { x: 0.0, y: 3.0, z: -1.0 },
                        end_point: Point { x: 0.0, y: 1.0, z: -3.0 } };

        let p = l1.intersect_with_line(&l2, false, &Tolerance::default());

        match p {
            Ok(ip) => {
                assert!(ip.is_equal_to(&Point { x: 0.0, y: 2.0, z: -2.0 }, &Tolerance::default()))
            },
            Err(error) => {
                panic!("error in intersect_with_line: {:?}", error)
            },
        };
    }

    #[test]
    fn line_intersect_with_line_xz_axis() {
        let l1 = Line { start_point: Point { x: 4.0, y: 0.0, z: -4.0 },
                        end_point: Point { x: 1.0, y: 0.0, z: -1.0 } };
        let l2 = Line { start_point: Point { x: 3.0, y: 0.0, z: -1.0 },
                        end_point: Point { x: 1.0, y: 0.0, z: -3.0 } };

        let p = l1.intersect_with_line(&l2, false, &Tolerance::default());

        match p {
            Ok(ip) => {
                assert!(ip.is_equal_to(&Point { x: 2.0, y: 0.0, z: -2.0 }, &Tolerance::default()))
            },
            Err(error) => {
                panic!("error in intersect_with_line: {:?}", error)
            },
        };
    }

    #[test]
    fn line_intersect_with_line_entpoints() {
        let l1 = Line { start_point: Point { x: 0.0, y: 6.0, z: 5.0 },
                        end_point: Point { x: 8.0, y: 0.0, z: 3.0 } };
        let l2 = Line { start_point: Point { x: 1.0, y: 0.0, z: 10.0 },
                        end_point: Point { x: 8.0, y: 0.0, z: 3.0 } };

        let p = l1.intersect_with_line(&l2, false, &Tolerance::default());

        match p {
            Ok(ip) => {
                assert!(ip.is_equal_to(&Point { x: 8.0, y: 0.0, z: 3.0 }, &Tolerance::default()))
            },
            Err(error) => {
                panic!("error in intersect_with_line: {:?}", error)
            },
        };
    }

    #[test]
    fn line_intersect_with_line_with_extend() {
        let l1 = Line { start_point: Point { x: 268.3669, y: 445.9483, z: 0.0 },
                        end_point: Point { x: 1596.5413, y: 1349.3888, z: 0.0 } };
        let l2 = Line { start_point: Point { x: 1918.3457, y: 1355.2363, z: 0.0 },
                        end_point: Point { x: 2588.2839, y: 355.3119, z: 0.0 } };

        let p = l1.intersect_with_line(&l2, false, &Tolerance::default());

        match p {
            Ok(_ip) => {
                panic!("thie test should be error.");
            },
            Err(error) => {
                assert_eq!(error, ErrorStatus::InvalidInput);
            },
        };

        let p = l1.intersect_with_line(&l2, true, &Tolerance::default());

        match p {
            Ok(ip) => {
                assert!(ip.is_equal_to(&Point { x: 1820.2924, y: 1501.5870, z: 0.0 }, &Tolerance::default()));
            },
            Err(error) => {
                panic!("error in intersect_with_line: {:?}", error)
            },
        };

        let l3 = Line { start_point: Point { x: 268.3669, y: 445.9483, z: 10.0 },
                        end_point: Point { x: 1596.5413, y: 1349.3888, z: 10.0 } };

        let p = l1.intersect_with_line(&l3, true, &Tolerance::default());

        match p {
            Ok(_ip) => {
                panic!("thie test should be error.");
            },
            Err(error) => {
                assert_eq!(error, ErrorStatus::MustBeNonZero);
            },
        };

        let l4 = Line { start_point: Point { x: 1918.3457, y: 1355.2363, z: 10.0 },
                        end_point: Point { x: 2588.2839, y: 355.3119, z: 10.0 } };

        let p = l1.intersect_with_line(&l4, true, &Tolerance::default());

        match p {
            Ok(_ip) => {
                panic!("thie test should be error.");
            },
            Err(error) => {
                assert_eq!(error, ErrorStatus::InvalidInput);
            },
        };
    }
}
