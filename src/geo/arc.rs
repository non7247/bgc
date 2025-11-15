use super::*;
use crate::{ math::{self, quadratic_equation}, BgcError, Tolerance };

#[derive(Debug)]
pub struct Arc {
    pub center_point: Point,
    pub x_axis: Vector,
    pub y_axis: Vector,
    pub radius: f64,
    pub start_angle: f64,
    pub end_angle: f64,
}

impl Arc {
    /// Makes an arc from three points.
    pub fn from_three_points(
        start_point: &Point,
        end_point: &Point,
        on_arc: &Point,
        tol: &Tolerance
    ) -> Result<Self, BgcError> {
        let to_start = (start_point - on_arc).normal(tol);
        let to_end = (end_point - on_arc).normal(tol);

        if to_start.is_equal_to(&(to_end * -1.0), tol) {
            return Err(BgcError::InvalidInput);
        }

        let nrm_vec = to_start.outer_product(&to_end);

        let mid1 = start_point.calc_middle_point(on_arc);
        let mid2 = end_point.calc_middle_point(on_arc);

        let line1 = Line::new(mid1, mid1 + to_start.outer_product(&nrm_vec));
        let line2 = Line::new(mid2, mid2 + to_end.outer_product(&nrm_vec));

        let Ok(ip) = line1.intersect_with_line(&line2, true, tol) else {
            return Err(BgcError::InvalidInput);
        };
        let center = ip[0];

        let radius = center.distance_to(on_arc);
        let x_axis = (start_point - center).normal(tol);

        let to_on_arc = (on_arc - center).normal(tol);
        let ref_vec = x_axis.outer_product(&to_on_arc);
        let y_axis = ref_vec.outer_product(&x_axis).normal(tol);

        let local_end = end_point.transform(
            &Matrix3d::transform_to_local(
                &center,
                &x_axis,
                &y_axis,
                tol
            ),
            tol
        )?;
        let end_angle = Arc::calc_angle_at_local_point(&local_end);

        Ok(Self { center_point: center,
                  x_axis,
                  y_axis,
                  radius,
                  start_angle: 0.0,
                  end_angle })
    }

    pub fn length(&self) -> f64 {
        self.calc_length_at_param(self.end_angle)
    }

    pub fn start_point(&self) -> Point {
        self.calc_point_at_param(self.start_angle)
    }

    pub fn end_point(&self) -> Point {
        self.calc_point_at_param(self.end_angle)
    }

    /// Calculates the closest point on this arc to input point.
    pub fn closest_point(
        &self,
        point: &Point,
        extends: bool,
        tol: &Tolerance
    ) -> Result<Point, BgcError> {
        let mut local_point = point.transform(
            &Matrix3d::transform_to_local(
                &self.center_point,
                &self.x_axis,
                &self.y_axis,
                tol
            ),
            tol
        )?;
        local_point.z = 0.0;

        let line = Line::new(Point::origin(), local_point);
        let on_arc = line.point_at_dist(self.radius, true, tol)?;
        let angle = Arc::calc_angle_at_local_point(&on_arc);

        if !extends && !self.is_param_in_range(angle, tol) {
            let to_start = self.start_point().distance_to(&local_point);
            let to_end = self.end_point().distance_to(&local_point);

            if to_start < to_end {
                Ok(self.start_point())
            } else {
                Ok(self.end_point())
            }
        } else {
            Ok(on_arc.transform(
                &Matrix3d::transform_to_world(
                    &self.center_point,
                    &self.x_axis,
                    &self.y_axis,
                    tol
                ),
                tol
            )?)
        }
    }

    /// Determines if input point lies on this arc.
    pub fn contains(&self, point: &Point, extends: bool, tol: &Tolerance) -> bool {
        if self.start_point().is_equal_to(point, tol) || self.end_point().is_equal_to(point, tol) {
            return true;
        }

        if let Ok(closest) = self.closest_point(point, extends, tol) {
            closest.is_equal_to(point, tol)
        } else {
            false
        }
    }

    pub fn containing_plane(&self, tol: &Tolerance) -> Plane {
        let z_axis = self.x_axis.outer_product(&self.y_axis);
        Plane::from(&self.center_point, &z_axis, tol)
    }

    fn calc_angle_at_local_point(p: &Point) -> f64 {
        let angle = p.y.atan2(p.x);
        if angle < 0.0 {
            angle + std::f64::consts::PI * 2.0
        } else {
            angle
        }
    }

    fn calc_point_at_param(&self, param: f64) -> Point {
        self.center_point + (self.x_axis * param.cos() + self.y_axis * param.sin()) * self.radius
    }

    fn calc_length_at_param(&self, param: f64) -> f64 {
        (param - self.start_angle) * self.radius
    }

    fn is_param_in_range(&self, param: f64, tol: &Tolerance) -> bool {
        if (self.start_angle - param).abs() < tol.calculation() ||
                (self.end_angle - param).abs() < tol.calculation() {
            return true;
        }

        if param < self.start_angle || self.end_angle < param {
            return false;
        }

        true
    }

    fn intersect_with_line_in_local(
        &self,
        other: &Line,
        extends: bool,
        tol: &Tolerance
    ) -> Result<Vec<Point>, BgcError> {
        let start = other.start_point;
        let dir = other.direction(tol).normal(tol);

        let a = dir.x * dir.x + dir.y * dir.y;
        let b = 2.0 * (start.x * dir.x + start.y * dir.y);
        let c = start.x * start.x + start.y * start.y - self.radius * self.radius;

        let Ok(roots) = math::quadratic_equation(a, b, c, tol) else {
            return Err(BgcError::InvalidInput);
        };

        let p1 = other.point_at_dist(roots.0, true, tol)?;
        let p2 = other.point_at_dist(roots.1, true, tol)?;

        let mut points = Vec::new();
        if self.is_param_in_range(Arc::calc_angle_at_local_point(&p1), tol) &&
                (extends || other.contains(&p1, false, tol)) {
            points.push(p1);
        }

        if !p1.is_equal_to(&p2, tol) &&
                self.is_param_in_range(Arc::calc_angle_at_local_point(&p2), tol) &&
                (extends || other.contains(&p2, false, tol)) {
            points.push(p2);
        }

        if points.is_empty() {
            Err(BgcError::InvalidInput)
        } else {
            Ok(points)
        }
    }

    fn intersect_with_circle_in_local(
        &self,
        other_center: &Point,
        other_radius: f64,
        tol: &Tolerance
    ) -> Result<Vec<Point>, BgcError> {
        if self.center_point.is_equal_to(other_center, tol) {
            return Err(BgcError::InvalidInput);
        }

        let r1 = self.radius;
        let r2 = other_radius;

        let dist = self.center_point.distance_to(other_center);
        if (dist - r1 - r2).abs() < tol.equal_point()
                || (r1 - (dist + r2)).abs() < tol.equal_point()
                || (r2 - (dist + r1)).abs() < tol.equal_point() {
            // two circles are tangent
            return Err(BgcError::NotImplemented);
        } else if dist - r1 - r2 > 0.0 {
            // two circles are completely separate
            return Err(BgcError::InvalidInput);
        }

        // check one circle is entirely contained within the other
        if (r1 - r2).abs() > tol.equal_point() {
            if (r1 > r2 && r1 > r2 + dist) || (r2 > r1 && r2 > r1 + dist) {
                return Err(BgcError::InvalidInput);
            }
        }

        if (self.center_point.x - other_center.x).abs() < tol.equal_point() {
            let a = other_center.x;
            let x = (a * a + r1 * r1 - r2 * r2) / (2.0 * a);

            let y = (r1 * r1 - x * x).sqrt();
            if y.abs() < tol.equal_point() {
                return Ok(vec![Point::new(x, 0.0, 0.0)]);
            } else {
                return Ok(vec![Point::new(x, y, 0.0), Point::new(x, -y, 0.0)]);
            }
        } else if (self.center_point.y - other_center.y).abs() < tol.equal_point() {
            let b = other_center.y;
            let y = (b * b + r1 * r1 - r2 * r2) / (2.0 * b);

            let x = (r1 * r1 - y * y).sqrt();
            if x.abs() < tol.equal_point() {
                return Ok(vec![Point::new(0.0, y, 0.0)]);
            } else {
                return Ok(vec![Point::new(x, y, 0.0), Point::new(-x, y, 0.0)]);
            }
        } else {
            let a = other_center.x;
            let b = other_center.y;

            let ld = a * a + b * b + r1 * r1 - r2 * r2;
            let la = a * a / (b * b) + 1.0;
            let lb = -(a * ld / (b * b));
            let lc = ld * ld / (4.0 * b * b) - r1 * r1;

            let Ok(roots) = math::quadratic_equation(la, lb, lc, tol) else {
                return Err(BgcError::InvalidInput);
            };

            if (roots.0 - roots.1).abs() < tol.calculation() {
                let y = (-2.0 * a * roots.0 + ld) / (2.0 * b);
                return Ok(vec![Point::new(roots.0, y, 0.0)]);
            } else {
                let y1 = (-2.0 * a * roots.0 + ld) / (2.0 * b);
                let y2 = (-2.0 * a * roots.1 + ld) / (2.0 * b);
                return Ok(vec![Point::new(roots.0, y1, 0.0), Point::new(roots.1, y2, 0.0)]);
            }
        }
    }
}

impl Curve for Arc {
    fn intersect_with_line(
        &self,
        other: &Line,
        extends: bool,
        tol: &Tolerance
    ) -> Result<Vec<Point>, BgcError> {
        let local_plane = self.containing_plane(tol);

        if other.is_parallel_with_plane(&local_plane, tol) {
            if local_plane.contains(&other.start_point, tol) {
                let local_line = other.transform(
                    &Matrix3d::transform_to_local(
                        &self.center_point,
                        &self.x_axis,
                        &self.y_axis,
                        tol
                    ),
                tol)?;

                let local_points = self.intersect_with_line_in_local(&local_line, extends, tol)?;
                let world_points: Result<Vec<Point>, BgcError> = local_points.iter().map(|p| {
                    p.transform(
                        &Matrix3d::transform_to_world(
                            &self.center_point,
                            &self.x_axis,
                            &self.y_axis,
                            tol
                        ),
                        tol
                    )
                }).collect();
                return world_points;
            }
        } else {
            let intersections = other.intersect_with_plane(&local_plane, extends, tol)?;
            if intersections.is_empty() {
                return Err(BgcError::InvalidInput);
            }
            let intersection = intersections[0];
            if self.contains(&intersection, extends, tol) {
                return Ok(vec![intersection]);
            } else {
                return Err(BgcError::InvalidInput);
            }
        }

        Err(BgcError::InvalidInput)
    }

    fn intersect_with_arc(
        &self,
        other: &Arc,
        extends: bool,
        tol: &Tolerance
    ) -> Result<Vec<Point>, BgcError> {
        let local_plane = self.containing_plane(tol);
        let other_plane = self.containing_plane(tol);

        if local_plane.is_parallel_to(&other_plane, tol) {
            if local_plane.is_coplanar_with(&other_plane, tol) {
                let local_center = other.center_point.transform(
                    &&Matrix3d::transform_to_local(
                        &self.center_point,
                        &self.x_axis,
                        &self.y_axis,
                        tol
                    ),
                tol)?;
            }
        } else {
            return Err(BgcError::NotImplemented);
        }

        Err(BgcError::InvalidInput)
    }

    fn intersect_with_plane(
        &self,
        other: &Plane,
        extends: bool,
        tol: &Tolerance
    ) -> Result<Vec<Point>, BgcError> {
        let local_plane = self.containing_plane(tol);

        if local_plane.is_parallel_to(other, tol) {
            return Err(BgcError::InvalidInput);
        }

        let intersection_line = local_plane.intersect_with_plane(other, tol)?;

        // Transform the intersection line to the arc's local coordinate system
        let to_local_mat = Matrix3d::transform_to_local(
            &self.center_point,
            &self.x_axis,
            &self.y_axis,
            tol,
        );
        let local_line = intersection_line.transform(&to_local_mat, tol)?;

        // Solve for the intersection of the local line and the circle equation x^2 + y^2 = r^2
        let start = local_line.start_point;
        let dir = local_line.direction(tol).normal(tol);

        let a = dir.x * dir.x + dir.y * dir.y;
        let b = 2.0 * (start.x * dir.x + start.y * dir.y);
        let c = start.x * start.x + start.y * start.y - self.radius * self.radius;

        let Ok(roots) = math::quadratic_equation(a, b, c, tol) else {
            // No real roots means no intersection
            return Err(BgcError::InvalidInput);
        };

        // Calculate intersection points in the local coordinate system
        let p1_local = local_line.point_at_dist(roots.0, true, tol)?;
        let p2_local = local_line.point_at_dist(roots.1, true, tol)?;

        let mut local_points = vec![p1_local];
        if !p1_local.is_equal_to(&p2_local, tol) {
            local_points.push(p2_local);
        }

        // Transform intersection points back to the world coordinate system
        let to_world_mat = Matrix3d::transform_to_world(
            &self.center_point,
            &self.x_axis,
            &self.y_axis,
            tol,
        );
        let intersection_points: Result<Vec<Point>, BgcError> = local_points
            .iter()
            .map(|p| p.transform(&to_world_mat, tol))
            .collect();
        let intersection_points = intersection_points?;

        if extends {
            return Ok(intersection_points);
        }

        // Filter points to be within the arc's range
        let valid_points: Vec<Point> = intersection_points
            .into_iter()
            .filter(|p| self.contains(p, false, tol))
            .collect();

        if valid_points.is_empty() {
            Err(BgcError::InvalidInput)
        } else {
            Ok(valid_points)
        }
    }
}

#[cfg(test)]
mod tests  {
    use super::*;

    #[test]
    fn arc_from() {
        let arc = Arc::from_three_points(
            &Point::new(-25.1550, -11.1966, 0.0),
            &Point::new(41.0084, 0.8494, 0.0),
            &Point::new(4.7497, 7.9195, 0.0),
            &Tolerance::default()
        );

        let Ok(arc) = arc else {
            panic!("error in arc_from: {:?}", arc.unwrap_err());
        };

        assert!(arc.center_point.is_equal_to(
            &Point::new(14.2467, -39.8864, 0.0),
            &Tolerance::default()
        ));
        assert!(arc.x_axis.is_equal_to(
            &Vector::new(-0.808404, 0.588628, 0.0),
            &Tolerance::default()
        ), "x_axis is {:?}", arc.x_axis);
        assert!(arc.y_axis.is_equal_to(
            &Vector::new(0.588628, 0.808404, 0.0),
            &Tolerance::default()
        ), "y_axis is {:?}", arc.y_axis);
        assert!((arc.radius - 48.7401).abs() < 0.0001);
        assert!((arc.start_angle - 0.0).abs() < 0.0001);
        assert!((arc.end_angle - 1.5227).abs() < 0.0001);
    }

    #[test]
    fn arc_is_on() {
        let arc = Arc::from_three_points(
            &Point::new(45584.895199, 7078.244811, 0.0),
            &Point::new(60917.404770, 4381.865751, 0.0),
            &Point::new(64213.475424, 3403.635799, 0.0),
            &Tolerance::default()
        );
        let Ok(arc) = arc else {
            panic!("error in arc_is_on: {:?}", arc.unwrap_err());
        };

        assert!(arc.contains(
            &Point::new(50748.612270, 6499.672934, 0.0),
            false,
            &Tolerance::default()
        ));
    }

    #[test]
    fn arc_intersect_with_line() {
        let arc = Arc::from_three_points(
            &Point::new(-25.1550, -11.1966, 0.0),
            &Point::new(41.0084, 0.8494, 0.0),
            &Point::new(4.7497, 7.9195, 0.0),
            &Tolerance::default()
        );
        let Ok(arc) = arc else {
            panic!("error in arc_intersect_with_line: {:?}", arc.unwrap_err());
        };

        let line = Line::new(Point::new(30.0, 20.0, 0.0), Point::new(-5.0, -10.0, 0.0));

        let p = arc.intersect_with_line(&line, false, &Tolerance::default());

        let Ok(points) = p else {
            panic!("error in arc_intersect_with_line: {:?}", p.unwrap_err());
        };
        assert_eq!(points.len(), 1);
        assert!(points[0].is_equal_to(
            &Point::new(16.9110, 8.7808, 0.0),
            &Tolerance::default()
        ));
    }

    #[test]
    fn arc_intersect_with_line_two_points() {
        let arc = Arc {
            center_point: Point::origin(),
            x_axis: Vector::x_axis(),
            y_axis: Vector::y_axis(),
            radius: 5.0,
            start_angle: 0.0,
            end_angle: std::f64::consts::PI, // Semicircle
        };
        let line = Line::new(Point::new(-10.0, 3.0, 0.0), Point::new(10.0, 3.0, 0.0));
        let tol = Tolerance::default();

        let result = arc.intersect_with_line(&line, false, &tol);

        match result {
            Ok(points) => {
                assert_eq!(points.len(), 2);
                // Intersections should be at (4, 3) and (-4, 3)
                let p1 = Point::new(4.0, 3.0, 0.0);
                let p2 = Point::new(-4.0, 3.0, 0.0);
                assert!(points.iter().any(|p| p.is_equal_to(&p1, &tol)));
                assert!(points.iter().any(|p| p.is_equal_to(&p2, &tol)));
            },
            Err(e) => panic!("Expected two intersection points, but got error: {:?}", e),
        }
    }

    #[test]
    fn arc_intersect_with_line_no_intersection() {
        let arc = Arc {
            center_point: Point::origin(),
            x_axis: Vector::x_axis(),
            y_axis: Vector::y_axis(),
            radius: 5.0,
            start_angle: 0.0,
            end_angle: std::f64::consts::PI,
        };
        // Line is outside the circle
        let line = Line::new(Point::new(-10.0, 6.0, 0.0), Point::new(10.0, 6.0, 0.0));
        let tol = Tolerance::default();

        let result = arc.intersect_with_line(&line, false, &tol);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), BgcError::InvalidInput);
    }

    #[test]
    fn arc_intersect_with_line_tangent() {
        let arc = Arc {
            center_point: Point::origin(),
            x_axis: Vector::x_axis(),
            y_axis: Vector::y_axis(),
            radius: 5.0,
            start_angle: 0.0,
            end_angle: std::f64::consts::PI,
        };
        let line = Line::new(Point::new(-10.0, 5.0, 0.0), Point::new(10.0, 5.0, 0.0));
        let tol = Tolerance::default();

        let result = arc.intersect_with_line(&line, false, &tol);

        match result {
            Ok(points) => {
                assert_eq!(points.len(), 1);
                assert!(points[0].is_equal_to(&Point::new(0.0, 5.0, 0.0), &tol));
            },
            Err(e) => panic!("Expected a tangent point, but got error: {:?}", e),
        }
    }

    #[test]
    fn arc_intersect_with_line_endpoint() {
        let arc = Arc {
            center_point: Point::origin(),
            x_axis: Vector::x_axis(),
            y_axis: Vector::y_axis(),
            radius: 5.0,
            start_angle: 0.0,
            end_angle: std::f64::consts::PI,
        };
        // Line passes through the start point of the arc (5, 0, 0)
        let line = Line::new(Point::new(5.0, -5.0, 0.0), Point::new(5.0, 5.0, 0.0));
        let tol = Tolerance::default();

        let result = arc.intersect_with_line(&line, false, &tol);

        match result {
            Ok(points) => {
                assert_eq!(points.len(), 1);
                assert!(points[0].is_equal_to(&Point::new(5.0, 0.0, 0.0), &tol));
            },
            Err(e) => panic!("Expected an intersection at the endpoint, but got error: {:?}", e),
        }
    }

    #[test]
    fn arc_intersect_with_line_extends() {
        let arc = Arc {
            center_point: Point::origin(),
            x_axis: Vector::x_axis(),
            y_axis: Vector::y_axis(),
            radius: 5.0,
            start_angle: 0.0,
            end_angle: std::f64::consts::PI,
        };
        // The line segment does not cross the arc, but its extension does.
        let line = Line::new(Point::new(6.0, 3.0, 0.0), Point::new(10.0, 3.0, 0.0));
        let tol = Tolerance::default();

        // Test with extends: false - should be no intersection
        let result_no_extends = arc.intersect_with_line(&line, false, &tol);
        assert!(result_no_extends.is_err());
        assert_eq!(result_no_extends.unwrap_err(), BgcError::InvalidInput);

        // Test with extends: true - should be two intersections
        let result_extends = arc.intersect_with_line(&line, true, &tol);
        match result_extends {
            Ok(points) => {
                 assert_eq!(points.len(), 2);
                let p1 = Point::new(4.0, 3.0, 0.0);
                let p2 = Point::new(-4.0, 3.0, 0.0);
                assert!(points.iter().any(|p| p.is_equal_to(&p1, &tol)));
                assert!(points.iter().any(|p| p.is_equal_to(&p2, &tol)));
            },
            Err(e) => panic!("Expected two intersection points with extend=true, but got error: {:?}", e),
        }
    }

    #[test]
    fn arc_intersect_with_plane_one_point() {
        let arc = Arc {
            center_point: Point::origin(),
            x_axis: Vector::x_axis(),
            y_axis: Vector::y_axis(),
            radius: 5.0,
            start_angle: 0.0,
            end_angle: std::f64::consts::PI, // Semicircle
        };
        let plane = Plane::from(&Point::new(3.0, 0.0, 0.0), &Vector::new(1.0, 0.0, 0.0), &Tolerance::default());
        let tol = Tolerance::default();

        let result = arc.intersect_with_plane(&plane, false, &tol);

        match result {
            Ok(points) => {
                assert_eq!(points.len(), 1);
                assert!(points[0].is_equal_to(&Point::new(3.0, 4.0, 0.0), &tol));
            },
            Err(e) => panic!("Expected one intersection point, but got error: {:?}", e),
        }
    }

    #[test]
    fn arc_intersect_with_plane_two_points() {
        let arc = Arc {
            center_point: Point::origin(),
            x_axis: Vector::x_axis(),
            y_axis: Vector::y_axis(),
            radius: 5.0,
            start_angle: 0.0,
            end_angle: std::f64::consts::PI * 2.0, // Full circle
        };
        let plane = Plane::from(&Point::new(0.0, 3.0, 0.0), &Vector::new(0.0, 1.0, 0.0), &Tolerance::default());
        let tol = Tolerance::default();

        let result = arc.intersect_with_plane(&plane, false, &tol);

        match result {
            Ok(points) => {
                assert_eq!(points.len(), 2);
                let p1 = Point::new(4.0, 3.0, 0.0);
                let p2 = Point::new(-4.0, 3.0, 0.0);
                assert!(points.iter().any(|p| p.is_equal_to(&p1, &tol)));
                assert!(points.iter().any(|p| p.is_equal_to(&p2, &tol)));
            },
            Err(e) => panic!("Expected two intersection points, but got error: {:?}", e),
        }
    }

    #[test]
    fn arc_intersect_with_arc_two_points() {
        let arc1 = Arc {
            center_point: Point::origin(),
            x_axis: Vector::x_axis(),
            y_axis: Vector::y_axis(),
            radius: 5.0,
            start_angle: 0.0,
            end_angle: std::f64::consts::PI * 2.0, // Full circle
        };
        let arc2 = Arc {
            center_point: Point::new(6.0, 0.0, 0.0),
            x_axis: Vector::x_axis(),
            y_axis: Vector::y_axis(),
            radius: 5.0,
            start_angle: 0.0,
            end_angle: std::f64::consts::PI * 2.0, // Full circle
        };
        let tol = Tolerance::default();

        let result = arc1.intersect_with_arc(&arc2, false, &tol);

        match result {
            Ok(points) => {
                assert_eq!(points.len(), 2);
                let p1 = Point::new(3.0, 4.0, 0.0);
                let p2 = Point::new(3.0, -4.0, 0.0);
                assert!(points.iter().any(|p| p.is_equal_to(&p1, &tol)));
                assert!(points.iter().any(|p| p.is_equal_to(&p2, &tol)));
            },
            Err(e) => panic!("Expected two intersection points, but got error: {:?}", e),
        }
    }

    #[test]
    fn intersect_with_circle_in_local_two_points() {
        let arc = Arc {
            center_point: Point::origin(),
            x_axis: Vector::x_axis(),
            y_axis: Vector::y_axis(),
            radius: 5.0,
            start_angle: 0.0,
            end_angle: std::f64::consts::PI * 2.0,
        };
        let other_center = Point::new(6.0, 0.0, 0.0);
        let other_radius = 5.0;
        let tol = Tolerance::default();

        let result = arc.intersect_with_circle_in_local(&other_center, other_radius, &tol);

        match result {
            Ok(points) => {
                assert_eq!(points.len(), 2);
                let p1 = Point::new(3.0, 4.0, 0.0);
                let p2 = Point::new(3.0, -4.0, 0.0);
                assert!(points.iter().any(|p| p.is_equal_to(&p1, &tol)));
                assert!(points.iter().any(|p| p.is_equal_to(&p2, &tol)));
            }
            Err(e) => panic!("Expected two intersection points, but got error: {:?}", e),
        }
    }

    #[test]
    fn intersect_with_circle_in_local_one_point_tangent() {
        let arc = Arc {
            center_point: Point::origin(),
            x_axis: Vector::x_axis(),
            y_axis: Vector::y_axis(),
            radius: 5.0,
            start_angle: 0.0,
            end_angle: std::f64::consts::PI * 2.0,
        };
        let other_center = Point::new(10.0, 0.0, 0.0);
        let other_radius = 5.0;
        let tol = Tolerance::default();

        let result = arc.intersect_with_circle_in_local(&other_center, other_radius, &tol);

        match result {
            Ok(points) => {
                assert_eq!(points.len(), 1);
                assert!(points[0].is_equal_to(&Point::new(5.0, 0.0, 0.0), &tol));
            }
            Err(e) => panic!("Expected one tangent point, but got error: {:?}", e),
        }
    }

    #[test]
    fn intersect_with_circle_in_local_no_intersection_apart() {
        let arc = Arc {
            center_point: Point::origin(),
            x_axis: Vector::x_axis(),
            y_axis: Vector::y_axis(),
            radius: 5.0,
            start_angle: 0.0,
            end_angle: std::f64::consts::PI * 2.0,
        };
        let other_center = Point::new(11.0, 0.0, 0.0);
        let other_radius = 5.0;
        let tol = Tolerance::default();

        let result = arc.intersect_with_circle_in_local(&other_center, other_radius, &tol);
        assert!(result.is_err());
    }

    #[test]
    fn intersect_with_circle_in_local_no_intersection_inside() {
        let arc = Arc {
            center_point: Point::origin(),
            x_axis: Vector::x_axis(),
            y_axis: Vector::y_axis(),
            radius: 10.0,
            start_angle: 0.0,
            end_angle: std::f64::consts::PI * 2.0,
        };
        let other_center = Point::new(2.0, 0.0, 0.0);
        let other_radius = 5.0;
        let tol = Tolerance::default();

        let result = arc.intersect_with_circle_in_local(&other_center, other_radius, &tol);
        assert!(result.is_err());
    }

    #[test]
    fn intersect_with_circle_in_local_concentric_error() {
        let arc = Arc {
            center_point: Point::origin(),
            x_axis: Vector::x_axis(),
            y_axis: Vector::y_axis(),
            radius: 5.0,
            start_angle: 0.0,
            end_angle: std::f64::consts::PI * 2.0,
        };
        let other_center = Point::origin();
        let other_radius = 10.0;
        let tol = Tolerance::default();

        let result = arc.intersect_with_circle_in_local(&other_center, other_radius, &tol);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), BgcError::InvalidInput);
    }

    #[test]
    fn intersect_with_circle_in_local_coincident_error() {
        let arc = Arc {
            center_point: Point::origin(),
            x_axis: Vector::x_axis(),
            y_axis: Vector::y_axis(),
            radius: 5.0,
            start_angle: 0.0,
            end_angle: std::f64::consts::PI * 2.0,
        };
        let other_center = Point::origin();
        let other_radius = 5.0;
        let tol = Tolerance::default();

        let result = arc.intersect_with_circle_in_local(&other_center, other_radius, &tol);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), BgcError::InvalidInput);
    }
}
