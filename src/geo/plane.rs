use super::*;

#[derive(Debug)]
pub struct Plane {
    /// 3D plane
    /// Ax + By + Cz + D = 0
    pub param_a: f64,
    pub param_b: f64,
    pub param_c: f64,
    pub param_d: f64,
}

impl Plane {
    pub fn from(point: &Point, vec: &Vector, tol: &Tolerance) -> Self {
        let normal_vec = vec.normal(tol);

        Self {
            param_a: normal_vec.x,
            param_b: normal_vec.y,
            param_c: normal_vec.z,
            param_d: -(normal_vec.x * point.x + normal_vec.y * point.y + normal_vec.z * point.z),
        }
    }

    /// Calculates the distance from a point to this plane.
    ///
    /// p0(x0, y0, z0) -> Ax + By + Cz + D = 0
    ///
    /// d = (Ax0 + By0 + Cz0 + D) / S
    /// S^2 = A^2 + B^2 + C^2
    pub fn distance_to(&self, point: &Point) -> f64 {
        let s = (self.param_a.powi(2) + self.param_b.powi(2) + self.param_c.powi(2)).sqrt();

        (point.x * self.param_a + point.y * self.param_b + point.z * self.param_c + self.param_d)
            .abs() / s
    }

    /// Calculates the closest point on this plane from a point.
    ///
    /// p0(x0, y0, z0) -> Ax + By + Cz + D = 0
    ///
    /// xa = x0 + At
    /// ya = y0 + Bt
    /// za = z0 + Ct
    /// t = -(Ax0 + By0 + Cz0 + D) / (A^2 + B^2 + C^2)
    pub fn closest_point(&self, point: & Point) -> Point {
        let t = -(self.param_a * point.x
            + self.param_b * point.y
            + self.param_c * point.z
            + self.param_d)
            / (self.param_a.powi(2) + self.param_b.powi(2) + self.param_c.powi(2));

        Point::new(
            point.x + self.param_a * t,
            point.y + self.param_b * t,
            point.z + self.param_c * t
        )
    }

    pub fn contains(&self, point: &Point, tol: &Tolerance) -> bool {
        self.distance_to(point) <= tol.equal_point()
    }

    pub fn get_normal_vector(&self, tol: &Tolerance) -> Vector {
        Vector::new(self.param_a, self.param_b, self.param_c).normal(tol)
    }

    pub fn is_parallel_to(&self, other: &Plane, tol: &Tolerance) -> bool {
        let self_normal = self.get_normal_vector(tol);
        let other_normal = other.get_normal_vector(tol);
        self_normal.is_parallel_to(&other_normal, tol)
    }

    pub fn is_coplanar_with(&self, other: &Plane, tol: &Tolerance) -> bool {
        self.is_parallel_to(other, tol) && (self.param_d - other.param_d).abs() < tol.equal_point()
    }

    pub fn intersect_with_plane(
        &self,
        other: &Plane,
        tol: &Tolerance
    ) -> Result<Line, crate::BgcError> {
        let self_normal = self.get_normal_vector(tol);
        let other_normal = other.get_normal_vector(tol);

        let line_dir = self_normal.outer_product(&other_normal);
        if line_dir.length() < tol.equal_vector() {
            return Err(crate::BgcError::InvalidInput); // Planes are parallel
        }

        // To find a point on the line, we can solve a system of 3 linear equations.
        // The equations are the two plane equations and a third plane equation that is
        // perpendicular to the line of intersection and passes through the origin.
        let third_plane_normal = line_dir.normal(tol);
        let third_plane = Plane::from(&Point::origin(), &third_plane_normal, tol);

        let a1 = self.param_a;
        let b1 = self.param_b;
        let c1 = self.param_c;
        let d1 = -self.param_d;

        let a2 = other.param_a;
        let b2 = other.param_b;
        let c2 = other.param_c;
        let d2 = -other.param_d;

        let a3 = third_plane.param_a;
        let b3 = third_plane.param_b;
        let c3 = third_plane.param_c;
        let d3 = -third_plane.param_d;

        let det = a1 * (b2 * c3 - b3 * c2) - b1 * (a2 * c3 - a3 * c2) + c1 * (a2 * b3 - a3 * b2);

        if det.abs() < tol.calculation() {
            return Err(crate::BgcError::InvalidInput);
        }

        let x = (d1 * (b2 * c3 - b3 * c2)
            - b1 * (d2 * c3 - d3 * c2)
            + c1 * (d2 * b3 - d3 * b2)) / det;
        let y = (a1 * (d2 * c3 - d3 * c2)
            - d1 * (a2 * c3 - a3 * c2)
            + c1 * (a2 * d3 - a3 * d2)) / det;
        let z = (a1 * (b2 * d3 - b3 * d2)
            - b1 * (a2 * d3 - a3 * d2)
            + d1 * (a2 * b3 - a3 * b2)) / det;

        let intersection_point = Point::new(x, y, z);

        Ok(Line::new(intersection_point, intersection_point + line_dir))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Tolerance;

    #[test]
    fn plane_from() {
        let tol = Tolerance::default();
        let p = Point::new(1.0, 2.0, 3.0);
        let v = Vector::new(0.0, 0.0, 1.0); // Z-axis normal

        let plane = Plane::from(&p, &v, &tol);

        assert!((plane.param_a - 0.0).abs() < tol.calculation());
        assert!((plane.param_b - 0.0).abs() < tol.calculation());
        assert!((plane.param_c - 1.0).abs() < tol.calculation());
        assert!((plane.param_d - (-3.0)).abs() < tol.calculation()); // -(0*1 + 0*2 + 1*3) = -3

        let p2 = Point::new(0.0, 0.0, 0.0);
        let v2 = Vector::new(1.0, 1.0, 1.0);
        let plane2 = Plane::from(&p2, &v2, &tol);

        let expected_normal_len = (1.0_f64.powi(2) + 1.0_f64.powi(2) + 1.0_f64.powi(2)).sqrt();
        assert!((plane2.param_a - (1.0 / expected_normal_len)).abs() < tol.calculation());
        assert!((plane2.param_b - (1.0 / expected_normal_len)).abs() < tol.calculation());
        assert!((plane2.param_c - (1.0 / expected_normal_len)).abs() < tol.calculation());
        assert!((plane2.param_d - 0.0).abs() < tol.calculation());
    }

    #[test]
    fn plane_distance_to() {
        let tol = Tolerance::default();
        let plane = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(0.0, 0.0, 1.0), &tol);

        // Point on the plane
        let p1 = Point::new(1.0, 1.0, 0.0);
        assert!((plane.distance_to(&p1) - 0.0).abs() < tol.equal_point());

        // Point above the plane
        let p2 = Point::new(1.0, 1.0, 5.0);
        assert!((plane.distance_to(&p2) - 5.0).abs() < tol.equal_point());

        // Point below the plane
        let p3 = Point::new(1.0, 1.0, -5.0);
        assert!((plane.distance_to(&p3) - 5.0).abs() < tol.equal_point());

        // Angled plane
        let plane2 = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(1.0, 1.0, 0.0), &tol);
        let p4 = Point::new(1.0, 0.0, 0.0);
        assert!((plane2.distance_to(&p4) - (1.0 / 2.0_f64.sqrt())).abs() < tol.equal_point());
    }

    #[test]
    fn plane_closest_point() {
        let tol = Tolerance::default();
        let plane = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(0.0, 0.0, 1.0), &tol);

        // Point above the plane
        let p1 = Point::new(1.0, 2.0, 5.0);
        let closest1 = plane.closest_point(&p1);
        assert!(closest1.is_equal_to(&Point::new(1.0, 2.0, 0.0), &tol));

        // Point below the plane
        let p2 = Point::new(3.0, 4.0, -5.0);
        let closest2 = plane.closest_point(&p2);
        assert!(closest2.is_equal_to(&Point::new(3.0, 4.0, 0.0), &tol));

        // Point on the plane
        let p3 = Point::new(5.0, 6.0, 0.0);
        let closest3 = plane.closest_point(&p3);
        assert!(closest3.is_equal_to(&Point::new(5.0, 6.0, 0.0), &tol));

        // Angled plane
        let plane2 = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(1.0, 1.0, 0.0), &tol);
        let p4 = Point::new(1.0, 0.0, 0.0);
        let closest4 = plane2.closest_point(&p4);
        assert!(closest4.is_equal_to(&Point::new(0.5, -0.5, 0.0), &tol));
    }

    #[test]
    fn plane_contains() {
        let tol = Tolerance::default();
        let plane = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(0.0, 0.0, 1.0), &tol);

        // Point on the plane
        let p1 = Point::new(1.0, 1.0, 0.0);
        assert!(plane.contains(&p1, &tol));

        // Point not on the plane
        let p2 = Point::new(1.0, 1.0, 5.0);
        assert!(!plane.contains(&p2, &tol));

        // Point very close to the plane (within tolerance)
        let p3 = Point::new(1.0, 1.0, tol.equal_point() * 0.5);
        assert!(plane.contains(&p3, &tol));
    }

    #[test]
    fn plane_get_normal_vector() {
        let tol = Tolerance::default();
        let plane = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(0.0, 0.0, 5.0), &tol);
        let normal = plane.get_normal_vector(&tol);
        assert!(normal.is_equal_to(&Vector::new(0.0, 0.0, 1.0), &tol));

        let plane2 = Plane::from(&Point::new(1.0, 2.0, 3.0), &Vector::new(1.0, 1.0, 1.0), &tol);
        let normal2 = plane2.get_normal_vector(&tol);
        let expected_normal = Vector::new(1.0, 1.0, 1.0).normal(&tol);
        assert!(normal2.is_equal_to(&expected_normal, &tol));
    }

    #[test]
    fn plane_is_parallel_to() {
        let tol = Tolerance::default();
        let plane1 = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(0.0, 0.0, 1.0), &tol);
        let plane2 = Plane::from(&Point::new(0.0, 0.0, 5.0), &Vector::new(0.0, 0.0, 1.0), &tol);
        let plane3 = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(1.0, 0.0, 0.0), &tol);

        assert!(plane1.is_parallel_to(&plane2, &tol));
        assert!(!plane1.is_parallel_to(&plane3, &tol));
    }

    #[test]
    fn plane_is_coplanar_with() {
        let tol = Tolerance::default();
        let plane1 = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(0.0, 0.0, 1.0), &tol);
        let plane2 = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(0.0, 0.0, 1.0), &tol);
        let plane3 = Plane::from(&Point::new(0.0, 0.0, 5.0), &Vector::new(0.0, 0.0, 1.0), &tol);

        assert!(plane1.is_coplanar_with(&plane2, &tol));
        assert!(!plane1.is_coplanar_with(&plane3, &tol)); // Parallel but not coplanar
    }

    #[test]
    fn plane_intersect_with_plane() {
        let tol = Tolerance::default();

        // Intersecting planes (X-Y plane and Y-Z plane)
        let plane1 = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(0.0, 0.0, 1.0), &tol); // Z=0
        let plane2 = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(1.0, 0.0, 0.0), &tol); // X=0
        let intersection_line = plane1.intersect_with_plane(&plane2, &tol).unwrap();
        assert!(intersection_line.start_point.is_equal_to(&Point::new(0.0, 0.0, 0.0), &tol));
        assert!(intersection_line.direction(&tol).is_parallel_to(&Vector::new(0.0, 1.0, 0.0), &tol));

        // Intersecting planes (angled)
        let plane3 = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(1.0, 1.0, 0.0), &tol);
        let plane4 = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(0.0, 1.0, 1.0), &tol);
        let intersection_line2 = plane3.intersect_with_plane(&plane4, &tol).unwrap();
        assert!(intersection_line2.start_point.is_equal_to(&Point::new(0.0, 0.0, 0.0), &tol));
        // Normal of plane3: (1,1,0), Normal of plane4: (0,1,1)
        // Cross product: (1*1 - 0*1, 0*0 - 1*1, 1*1 - 1*0) = (1, -1, 1)
        assert!(intersection_line2.direction(&tol).is_parallel_to(&Vector::new(1.0, -1.0, 1.0), &tol));

        // Parallel planes (should return error)
        let plane5 = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(0.0, 0.0, 1.0), &tol);
        let plane6 = Plane::from(&Point::new(0.0, 0.0, 5.0), &Vector::new(0.0, 0.0, 1.0), &tol);
        let result = plane5.intersect_with_plane(&plane6, &tol);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), crate::BgcError::InvalidInput);

        // Coplanar planes (should return error, as they don't intersect in a line)
        let plane7 = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(0.0, 0.0, 1.0), &tol);
        let plane8 = Plane::from(&Point::new(0.0, 0.0, 0.0), &Vector::new(0.0, 0.0, 1.0), &tol);
        let result2 = plane7.intersect_with_plane(&plane8, &tol);
        assert!(result2.is_err());
        assert_eq!(result2.unwrap_err(), crate::BgcError::InvalidInput);
    }
}