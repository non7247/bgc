use super::*;
use crate::{ BgcError, Tolerance };

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
    ) -> Result<Self, BgcError>
    {
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

        let ip = line1.intersect_with_line(&line2, true, tol);
        let center = match ip {
            Ok(ip) => ip[0],
            Err(_) => {
                return Err(BgcError::InvalidInput);
            },
        };

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
    ) -> Result<Point, BgcError>
    {
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
}

impl Curve for Arc {
    fn intersect_with_line(
        &self,
        other: &Line,
        extends: bool,
        tol: &Tolerance
    ) -> Result<Vec<Point>, BgcError> {
        let z_axis = self.x_axis.outer_product(&self.y_axis);
        let local_plane = Plane::from(&self.center_point, &z_axis, tol);

        if other.is_parallel_with_plane(&local_plane, tol) {
            if local_plane.contains(&other.start_point, tol) {

            }
        } else {
            let intersection = other.intersect_with_plane(&local_plane, extends, tol)?;
            dbg!(intersection);
            if self.contains(&intersection, extends, tol) {
                return Ok(vec![intersection]);
            } else {
                return Err(BgcError::InvalidInput);
            }
        }

        Err(BgcError::InvalidInput)
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

        let arc = match arc {
            Ok(arc) => arc,
            Err(error) => {
                panic!("error in arc_from: {:?}", error);
            }
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
        let arc = match arc {
            Ok(arc) => arc,
            Err(error) => {
                panic!("error in arc_is_on: {:?}", error);
            }
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
        let arc = match arc {
            Ok(arc) => arc,
            Err(error) => {
                panic!("error in arc_intersect_with_line: {:?}", error);
            }
        };

        let line = Line::new(Point::new(30.0, 20.0, 0.0), Point::new(-5.0, -10.0, 0.0));

        let p = arc.intersect_with_line(&line, false, &Tolerance::default());

        match p {
            Ok(points) => {
                assert_eq!(points.len(), 1);
                assert!(points[0].is_equal_to(
                    &Point::new(16.9110, 8.7808, 0.0),
                    &Tolerance::default()
                ));
            },
            Err(error) => {
                panic!("error in arc_intersect_with_line: {:?}", error);
            },
        }
    }
}
