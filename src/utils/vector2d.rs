use vector2d::Vector2D;

pub trait MaxLength {
    fn limit(&self, max: f64) -> Vector2D<f64>;
    fn magnitude(&self, max: f64) -> Vector2D<f64>;
    fn angle_with(&self,other: &Vector2D<f64>) -> f64;
}

impl MaxLength for Vector2D<f64> {
    fn limit(&self, max: f64) ->  Vector2D<f64>{
        let length = self.length();
        if length > max {
            self.normalise() * max
        } else {
            *self
        }
    } 
    fn magnitude(&self, max: f64) ->  Vector2D<f64>{
        let length = self.length();
        if length != 0.0 {
            self.normalise() * max
        } else {
            *self
        }
    }

    fn angle_with(&self,other: &Vector2D<f64>) -> f64 {
        let scalar_product = (self.x * other.x) + (self.y * other.y);
        (scalar_product/(self.length()*other.length())).acos()
    }
}

