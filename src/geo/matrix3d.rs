use super::*;

#[derive(Debug)]
pub struct Matrix3d {
    /// \[\[row\]; column\]
    pub matrix: [[f64; 4]; 4],
}

impl Matrix3d {
    pub fn identity() -> Self {
        Self { matrix: [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ] }
    }

    pub fn new() -> Self {
        Self { matrix: [
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]
        ] }
    }

    fn to_origin(origin: &Point) -> Self {
        let mut matrix = Self::identity();

        matrix.set(0, 3, -origin.x);
        matrix.set(1, 3, -origin.y);
        matrix.set(2, 3, -origin.z);

        matrix
    }

    fn rotation_axis(uaxis: &Vector, vaxis: &Vector, waxis: &Vector) -> Self {
        Self { matrix: [
            [uaxis.x, uaxis.y, uaxis.z, 0.0],
            [vaxis.x, vaxis.y, vaxis.z, 0.0],
            [waxis.x, waxis.y, waxis.z, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ] }
    }

    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.matrix[row][col]
    }

    pub fn set(&mut self, row: usize, col: usize, val: f64) {
        self.matrix[row][col] = val;
    }

    pub fn multiply_by(&self, rhs: &Self) -> Self {
        let mut result = Self::new();

        for i in 0..4 {
            for j in 0..4 {
                let mut elm = 0.0;
                for k in 0..4 {
                    elm += self.get(i, k) * rhs.get(k, j);
                }
                result.set(i, j, elm);
            }
        }

        result
    }

    /// Returns the matrix of transformation into the local coordinate system.
    pub fn transform_to_local(
        origin: &Point,
        uaxis: &Vector,
        vaxis: &Vector,
        tol: &Tolerance
    ) -> Self {
        let waxis = uaxis.outer_product(vaxis);

        Self::rotation_axis(
            &uaxis.normal(tol),
            &vaxis.normal(tol),
            &waxis.normal(tol)
        ).multiply_by(&Self::to_origin(origin))
    }

    /// Returns the matrix of transformation into the world coordinate system.
    pub fn transform_to_world(
        origin: &Point,
        uaxis: &Vector,
        vaxis: &Vector,
        tol: &Tolerance
    ) -> Self {
        let waxis = uaxis.outer_product(vaxis);

        let u = uaxis.normal(tol);
        let v = vaxis.normal(tol);
        let w = waxis.normal(tol);

        Self { matrix: [
            [u.x, v.x, w.x, origin.x],
            [u.y, v.y, w.y, origin.y],
            [u.z, v.z, w.z, origin.z],
            [0.0, 0.0, 0.0, 1.0]
        ] }
    }
}

impl Default for Matrix3d {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matrix3d_identity() {
        let mat = Matrix3d::identity();
        assert_eq!(mat.get(0, 0), 1.0);
        assert_eq!(mat.get(1, 1), 1.0);
        assert_eq!(mat.get(2, 2), 1.0);
        assert_eq!(mat.get(3, 3), 1.0);
        assert_eq!(mat.get(0, 1), 0.0);
        assert_eq!(mat.get(1, 0), 0.0);
    }

    #[test]
    fn matrix3d_new() {
        let mat = Matrix3d::new();
        for i in 0..4 {
            for j in 0..4 {
                assert_eq!(mat.get(i, j), 0.0);
            }
        }
    }

    #[test]
    fn matrix3d_get_set() {
        let mut mat = Matrix3d::new();
        mat.set(1, 2, 5.0);
        assert_eq!(mat.get(1, 2), 5.0);
    }

    #[test]
    fn matrix3d_multiply_by() {
        let tol = Tolerance::default();
        let mat1 = Matrix3d::identity();
        let mut mat2 = Matrix3d::new();
        for i in 0..4 {
            for j in 0..4 {
                mat2.set(i, j, (i * 4 + j) as f64);
            }
        }

        let result = mat1.multiply_by(&mat2);
        for i in 0..4 {
            for j in 0..4 {
                assert!((result.get(i, j) - (i * 4 + j) as f64).abs() < tol.calculation());
            }
        }

        // Translation matrix * translation matrix
        let mut t1 = Matrix3d::identity();
        t1.set(0, 3, 10.0);
        t1.set(1, 3, 20.0);
        t1.set(2, 3, 30.0);

        let mut t2 = Matrix3d::identity();
        t2.set(0, 3, 5.0);
        t2.set(1, 3, -5.0);
        t2.set(2, 3, 0.0);

        let result = t1.multiply_by(&t2);
        assert!((result.get(0, 3) - 15.0).abs() < tol.calculation());
        assert!((result.get(1, 3) - 15.0).abs() < tol.calculation());
        assert!((result.get(2, 3) - 30.0).abs() < tol.calculation());
    }

    #[test]
    fn matrix3d_scale_robustness() {
        let tol = Tolerance::default();
        
        // 1. Large coordinates (1,000km range)
        let origin = Point::new(1_000_000.0, 1_000_000.0, 1_000_000.0);
        let uaxis = Vector::new(1.0, 0.0, 0.0);
        let vaxis = Vector::new(0.0, 1.0, 0.0);
        
        let to_local = Matrix3d::transform_to_local(&origin, &uaxis, &vaxis, &tol);
        
        // Point P at a large coordinate
        let p_world = Point::new(1_000_001.0, 1_000_000.5, 1_000_000.0);
        let p_local = p_world.transform(&to_local, &tol).expect("Transformation failed");
        
        // In local coordinates, it should be (1.0, 0.5, 0.0)
        assert!(p_local.is_equal_to(&Point::new(1.0, 0.5, 0.0), &tol));

        // 2. Tiny coordinates and vectors
        let origin_tiny = Point::new(1.0e-7, 1.0e-7, 1.0e-7);
        let u_tiny = Vector::new(1.0, 1.0e-9, 0.0).normal(&tol); 
        let v_tiny = Vector::new(-1.0e-9, 1.0, 0.0).normal(&tol);
        
        let to_world = Matrix3d::transform_to_world(&origin_tiny, &u_tiny, &v_tiny, &tol);
        
        // Local origin should transform back to world tiny origin
        let p_back = Point::origin().transform(&to_world, &tol).expect("Transformation failed");
        assert!(p_back.is_equal_to(&origin_tiny, &tol));
    }

    #[test]
    fn matrix3d_transform_to_local() {
        let tol = Tolerance::default();

        let origin = Point::new(1.0, 1.0, 1.0);
        let uaxis = Vector::new(1.0, 1.0, 1.0);
        let vaxis = Vector::new(-1.0, 1.0, -1.0);
        let to_local = Matrix3d::transform_to_local(&origin, &uaxis, &vaxis, &tol);

        assert!((to_local.get(0, 0) - 0.577350).abs() < tol.calculation());
        assert!((to_local.get(1, 1) - 0.577350).abs() < tol.calculation());
        assert!((to_local.get(2, 2) - 0.707107).abs() < tol.calculation());
        assert!((to_local.get(3, 3) - 1.0).abs() < tol.calculation());

        let transformed = origin.transform(&to_local, &tol);
        match transformed {
            Ok(p) => assert!(p.is_equal_to(&Point::new(0.0, 0.0, 0.0), &tol)),
            Err(error) => {
                panic!("error in matrix3d_transform_to_local: {:?}", error); 
            }
        }

        let origin = Point::new(10.0, 20.0, 30.0);
        let uaxis = Vector::new(0.866025, 0.5, 0.0);
        let vaxis = Vector::new(-0.5, 0.866025, 0.0);
        let to_local = Matrix3d::transform_to_local(&origin, &uaxis, &vaxis, &tol);

        let transformed = Point::new(8.6603, 42.3205, 60.0).transform(&to_local, &tol);
        match transformed {
            Ok(p) => assert!(p.is_equal_to(&Point::new(10.0, 20.0, 30.0), &tol)),
            Err(error) => {
                panic!("error in matrix3d_transform_to_local: {:?}", error); 
            }
        }

        let origin = Point::new(83055.711625, 4650.0, 14686.607338);
        let uaxis = Vector::new(1.0, 0.0, -0.000556);
        let vaxis = Vector::new(0.000510, 0.398880, 0.917003);
        let to_local = Matrix3d::transform_to_local(&origin, &uaxis, &vaxis, &tol);

        let transformed = origin.transform(&to_local, &tol);
        match transformed {
            Ok(p) => assert!(p.is_equal_to(&Point::new(0.0, 0.0, 0.0), &tol)),
            Err(error) => {
                panic!("error in matrix3d_transform_to_local: {:?}", error); 
            }
        }

        let transformed 
            = Point::new(92443.211625, 5959.902281, 17693.140222).transform(&to_local, &tol);
        match transformed {
            Ok(p) => {
                assert!(p.is_equal_to(&Point::new(9385.826917, 3284.281094, 0.143078), &tol));
            },
            Err(error) => {
                panic!("error in matrix3d_transform_to_local: {:?}", error); 
            }
        }
    }

    #[test]
    fn matrix3d_transform_to_world() {
        let tol = Tolerance::default();

        let origin = Point::new(10.0, 20.0, 30.0);
        let uaxis = Vector::new(0.866025, 0.5, 0.0);
        let vaxis = Vector::new(-0.5, 0.866025, 0.0);
        let to_world = Matrix3d::transform_to_world(&origin, &uaxis, &vaxis, &tol);

        let transformed = origin.transform(&to_world, &tol);
        match transformed {
            Ok(p) => assert!(p.is_equal_to(&Point::new(8.6603, 42.3205, 60.0), &tol)),
            Err(error) => {
                panic!("error in matrix3d_transform_to_world: {:?}", error);
            }
        }

        let origin = Point::new(83055.711625, 4650.0, 14686.607338);
        let uaxis = Vector::new(1.0, 0.0, -0.000556);
        let vaxis = Vector::new(0.000510, 0.398880, 0.917003);
        let to_world = Matrix3d::transform_to_world(&origin, &uaxis, &vaxis, &tol);

        let mut ex_tol = Tolerance::default();
        ex_tol.set_equal_point(0.005);
        let transformed 
            = Point::new(9385.826917, 3284.281094, 0.143078).transform(&to_world, &tol);
        match transformed {
            Ok(p) => assert!(p.is_equal_to(
                &Point::new(92443.211625, 5959.902281, 17693.140222),
                &ex_tol
            )),
            Err(error) => {
                panic!("error in matrix3d_transform_to_world: {:?}", error);
            }
        }
    }
}
