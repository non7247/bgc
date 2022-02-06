use super::*;

#[derive(Debug)]
pub struct Matrix3d {
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

    pub fn get(&self, i: usize, j: usize) -> f64 {
        self.matrix[i][j] 
    }

    pub fn set(&mut self, i: usize, j: usize, val: f64) {
        self.matrix[i][j] = val;
    }

    /// Returns the matrix of transformation into the local coordinate system.
    pub fn transform_to_local(origin: &Point, uxais: &Vector, vaxis: &Vector, tol: &Tolerance)
            -> Self {
        Self { matrix: [[0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0]] }
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
    }
}
