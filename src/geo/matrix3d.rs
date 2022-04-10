use super::*;

#[derive(Debug)]
pub struct Matrix3d {
    /// \[\[row\]; column\]
    pub matrix: [[f64; 4]; 4],
}

impl Matrix3d {
    pub fn identity() -> Self {
        Self { matrix: [[1.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0]] }
    }

    pub fn new() -> Self {
        Self { matrix: [[0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0]] }
    }

    fn to_origin(origin: &Point) -> Self {
        let mut matrix = Self::identity();

        matrix.set(3, 0, -origin.x);
        matrix.set(3, 1, -origin.y);
        matrix.set(3, 2, -origin.z);

        matrix
    }

    fn rotation_axis(uaxis: &Vector, vaxis: &Vector, waxis: &Vector) -> Self {
        Self { matrix: [[uaxis.x, vaxis.x, waxis.x, 0.0],
                        [uaxis.y, vaxis.y, waxis.y, 0.0],
                        [uaxis.z, vaxis.z, waxis.z, 0.0],
                        [0.0, 0.0, 0.0, 1.0]] }
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
    pub fn transform_to_local(origin: &Point, uaxis: &Vector, vaxis: &Vector, tol: &Tolerance)
            -> Self {
        let waxis = uaxis.outer_product(vaxis);

        Self::to_origin(origin).multiply_by(&Self::rotation_axis(&uaxis.normal(tol), 
                                                                 &vaxis.normal(tol),
                                                                 &waxis.normal(tol)))
    }

    /// Returns the matrix of transformation into the world coordinate system.
    pub fn transform_to_world(origin: &Point, uaxis: &Vector, vaxis: &Vector, tol: &Tolerance)
            -> Self {
        let waxis = uaxis.outer_product(vaxis);

        let u = uaxis.normal(tol);
        let v = vaxis.normal(tol);
        let w = waxis.normal(tol);

        Self { matrix: [[u.x, u.y, u.z, 0.0],
                        [v.x, v.y, v.z, 0.0],
                        [w.x, w.y, w.z, 0.0],
                        [origin.x, origin.y, origin.z, 1.0]] }
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
    fn matrix3d_transform_to_local() {
        let d = 1.0e-6;

        let origin = Point { x: 1.0, y: 1.0, z: 1.0 };
        let uaxis = Vector { x: 1.0, y: 1.0, z: 1.0 };
        let vaxis = Vector { x: -1.0, y: 1.0, z: -1.0 };
        let result = Matrix3d::transform_to_local(&origin, &uaxis, &vaxis, &Tolerance::default());

        assert!((result.get(0, 0) - 0.577350).abs() < d);
        assert!((result.get(1, 1) - 0.577350).abs() < d);
        assert!((result.get(2, 2) - 0.707107).abs() < d);
        assert!((result.get(3, 3) - 1.0).abs() < d);

        let transformed = origin.transform(&result);
        assert!(transformed.is_equal_to(&Point { x: 0.0, y: 0.0, z: 0.0 },
                                        &Tolerance::default()));

        let origin = Point { x: 10.0, y: 20.0, z: 30.0 };
        let uaxis = Vector { x: 0.866025, y: 0.5, z: 0.0 };
        let vaxis = Vector { x: -0.5, y: 0.866025, z: 0.0 };
        let result = Matrix3d::transform_to_local(&origin, &uaxis, &vaxis, &Tolerance::default());

        let transformed = Point { x: 8.6603, y:42.3205, z: 60.0 }.transform(&result);
        assert!(transformed.is_equal_to(&Point { x: 10.0, y: 20.0, z: 30.0 },
                                        &Tolerance::default()));

        let origin = Point { x: 83055.711625, y: 4650.0, z: 14686.607338 };
        let uaxis = Vector { x: 1.0, y: 0.0, z: -0.000556 };
        let vaxis = Vector { x: 0.000510, y: 0.398880, z: 0.917003 };
        let result = Matrix3d::transform_to_local(&origin, &uaxis, &vaxis, &Tolerance::default());

        let transformed = origin.transform(&result);
        assert!(transformed.is_equal_to(&Point { x: 0.0, y: 0.0, z: 0.0 },
                                        &Tolerance::default()));

        let transformed = Point { x: 92443.211625,
                                  y: 5959.902281,
                                  z: 17693.140222 }.transform(&result);
        assert!(transformed.is_equal_to(&Point { x: 9385.826917, y: 3284.281094, z: 0.143078 },
                                        &Tolerance::default()),
                                        "transformed is {:?}", transformed);
    }

    #[test]
    fn matrix3d_transform_to_world() {
        let origin = Point { x: 10.0, y: 20.0, z: 30.0 };
        let uaxis = Vector { x: 0.866025, y: 0.5, z: 0.0 };
        let vaxis = Vector { x: -0.5, y: 0.866025, z: 0.0 };
        let result = Matrix3d::transform_to_world(&origin, &uaxis, &vaxis, &Tolerance::default());

        let transformed = origin.transform(&result);
        assert!(transformed.is_equal_to(&Point { x: 8.6603, y: 42.3205, z: 60.0 },
                                        &Tolerance::default()));

        let origin = Point { x: 83055.711625, y: 4650.0, z: 14686.607338 };
        let uaxis = Vector { x: 1.0, y: 0.0, z: -0.000556 };
        let vaxis = Vector { x: 0.000510, y: 0.398880, z: 0.917003 };
        let result = Matrix3d::transform_to_world(&origin, &uaxis, &vaxis, &Tolerance::default());

        let mut tol = Tolerance::default();
        tol.set_equal_point(0.005);
        let transformed = Point { x: 9385.826917,
                                  y: 3284.281094,
                                  z: 0.143078 }.transform(&result);
        assert!(transformed.is_equal_to(&Point { x: 92443.211625, y: 5959.902281, z: 17693.140222 },
                                        &tol),
                                        "transformed is {:?}", transformed);
    }
}
