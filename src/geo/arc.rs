use super::*;
use crate::{ ErrorStatus, Tolerance };

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
    pub fn from(start_point: &Point, end_point: &Point, on_arc: &Point, tol: &Tolerance)
            -> Result<Self, ErrorStatus> {
        let to_start = (*start_point - *on_arc).normal(tol);
        let to_end = (*end_point - *on_arc).normal(tol);

        if to_start.is_equal_to(&(to_end * -1.0), tol) {
            return Err(ErrorStatus::InvalidInput);
        }

        let nrm_vec = to_start.outer_product(&to_end);

        let mid1 = start_point.calc_middle_point(&on_arc);
        let mid2 = end_point.calc_middle_point(&on_arc);

        let line1 = Line { start_point: mid1,
                           end_point: mid1 + to_start.outer_product(&nrm_vec) };
        let line2 = Line { start_point: mid2,
                           end_point: mid2 + to_end.outer_product(&nrm_vec) };

        let ip = line1.intersect_with_line(&line2, true, tol);
        let center = match ip {
            Ok(ip) => ip[0],
            Err(_) => {
                return Err(ErrorStatus::InvalidInput);
            },
        };

        let radius = center.distance_to(on_arc);
        let x_axis = (*start_point - center).normal(tol);

        let to_on_arc = (*on_arc - center).normal(tol);
        let ref_vec = x_axis.outer_product(&to_on_arc);
        let y_axis = ref_vec.outer_product(&x_axis).normal(tol);

        let local_end = end_point.transform(&Matrix3d::transform_to_local(&center,
                                                                          &x_axis,
                                                                          &y_axis,
                                                                          &Tolerance::default()));
        let end_angle = Arc::calc_angle_at_local_point(&local_end);

        Ok(Self { center_point: center,
                  x_axis: x_axis,
                  y_axis: y_axis,
                  radius: radius,
                  start_angle: 0.0,
                  end_angle: end_angle })
    }

    fn calc_angle_at_local_point(p: &Point) -> f64 {
        let angle = p.y.atan2(p.x);
        if angle < 0.0 {
            angle + std::f64::consts::PI * 2.0
        } else {
            angle
        }
    }
}

#[cfg(test)]
mod tests  {
    use super::*;

    #[test]
    fn arc_from() {
        let arc = Arc::from(&Point { x: -25.1550, y: -11.1966, z: 0.0 },
                            &Point { x: 41.0084, y: 0.8494, z: 0.0 },
                            &Point { x: 4.7497, y: 7.9195, z: 0.0 },
                            &Tolerance::default());

        let arc = match arc {
            Ok(arc) => arc,
            Err(error) => {
                panic!("error in arc_from: {:?}", error);
            }
        };

        assert!(arc.center_point.is_equal_to(&Point { x: 14.2467, y: -39.8864, z: 0.0 },
                                             &Tolerance::default()));
        assert!(arc.x_axis.is_equal_to(&Vector { x: -0.808404, y: 0.588628, z: 0.0 },
                                       &Tolerance::default()),
                                       "x_axis is {:?}", arc.x_axis);
        assert!(arc.y_axis.is_equal_to(&Vector { x: 0.588628, y: 0.808404, z: 0.0 },
                                       &Tolerance::default()),
                                       "y_axis is {:?}", arc.y_axis);
        assert!((arc.radius - 48.7401).abs() < 0.0001);
        assert!((arc.start_angle - 0.0).abs() < 0.0001);
        assert!((arc.end_angle - 1.5227).abs() < 0.0001);
    }
}
