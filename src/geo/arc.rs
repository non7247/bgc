use super::{ Point, Vector, Line, Curve };
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

        // ToDo: calculates end angle

        Ok(Self { center_point: center,
                  x_axis: x_axis,
                  y_axis: y_axis,
                  radius: radius,
                  start_angle: 0.0,
                  end_angle: 0.0 })
    }
}
