extern crate sdl2; 
extern crate vector2d; 
mod utils;
use utils::vector2d::MaxLength;

use std::{
    time::Duration,
    error::Error,
    ptr,
    f64::consts::PI,
};

use vector2d::Vector2D;
use rand::Rng;

use sdl2::{
    pixels::Color,
    event::Event,
    keyboard::Keycode,
    gfx::primitives::DrawRenderer,
    render::Canvas,
    Sdl,
};

type Ssl2Canvas = Canvas<sdl2::video::Window>;



pub struct Boid {
    pub max_force: f64,
    pub max_velocity: f64,
	pub position: Vector2D<f64>,
	pub velocity: Vector2D<f64>,
    pub acceleration: Vector2D<f64>,
    pub align_squared_perception_radius: f64,
    pub separation_squared_perception_radius: f64,
    pub cohesion_squared_perception_radius: f64,
    
}

impl Boid {
    pub fn new() -> Boid {
        Boid {
            position: Vector2D::new(0.0,0.0),
            velocity: Vector2D::new(0.0,0.0),
            acceleration: Vector2D::new(0.0,0.0),
            max_force: 1.0,
            max_velocity: 4.0,
            align_squared_perception_radius: 50.0 * 50.0,
            separation_squared_perception_radius: 50.0 * 50.0,
            cohesion_squared_perception_radius: 100.0 * 100.0,
        }
    }

    pub fn random(  min_v: f64, 
                    max_v: f64, 
                    width: f64, 
                    height: f64, 
                    max_force: f64, 
                    max_velocity: f64, 
                    align_perception_radius: f64,
                    separation_perception_radius: f64,
                    cohesion_perception_radius: f64
                ) -> Boid{
        let mut rng = rand::thread_rng();
        
        let velocity_norm = rng.gen_range(min_v, max_v);
        let velocity_angle = rng.gen_range(0.0, 2.0*PI);
        let velocity = Vector2D::new(velocity_angle.sin(),velocity_angle.cos()) * velocity_norm;
        
        let position = Vector2D::new(rng.gen_range(0.0, width) as f64,rng.gen_range(0.0, height) as f64);

        Boid {
            position,
            velocity,
            acceleration: Vector2D::new(0.0,0.0),
            max_force,
            max_velocity,
            align_squared_perception_radius: align_perception_radius,
            separation_squared_perception_radius:separation_perception_radius * separation_perception_radius,
            cohesion_squared_perception_radius: cohesion_perception_radius * cohesion_perception_radius,
        }
    }
    
    pub fn can_view<'a>(&'a self,  boids:  &'a Vec<Boid>, perception: f64) -> impl Iterator<Item = &'a Boid>{
        boids.iter()
            .filter(move |boid| ! ptr::eq(*boid,self))
            .filter(move |boid| (boid.position - self.position).length_squared() < perception)
            .filter(move |boid| self.velocity.angle_with(&boid.velocity) < PI/3.0)
    }


    pub fn align(&self,  boids:  &Vec<Boid>) -> Vector2D<f64> {

        let (mut steering,count) = self
            .can_view(boids,self.align_squared_perception_radius)
            .map(|boid| boid.velocity)
            .fold((Vector2D::new(0.0,0.0),0),|(sum_velocity,count),velocity| (sum_velocity + velocity,count + 1));
        
        if count > 0 {
            steering = steering / count as f64;
            steering = steering.magnitude(self.max_velocity);
            steering = steering - self.velocity;
            steering = steering.limit(self.max_force);
            steering
        } else {
            Vector2D::new(0.0,0.0)
        }
    }

    pub fn cohesion(&self,  boids:  &Vec<Boid>) -> Vector2D<f64> {

        let (mut steering,count) = self.can_view(boids,self.cohesion_squared_perception_radius)
            .map(|boid| boid.position)
            .fold((Vector2D::new(0.0,0.0),0),|(sum_position,count),position| {
                (sum_position + position,count + 1)
            });

        if count > 0 {   
            steering = steering / count as f64;
            steering = steering - self.position;
            steering = steering.magnitude(self.max_velocity);
            steering = steering - self.velocity;
            steering = steering.limit(self.max_force);
            steering
        } else {
            Vector2D::new(0.0,0.0)
        }
    }

    pub fn separation(&self,  boids:  &Vec<Boid>) -> Vector2D<f64> {
        
        let (mut steering,count) = self.can_view(boids, self.separation_squared_perception_radius)
            .map(|boid| self.position - boid.position)
            .map(|v| {  let d = v.length(); v/(d*d) })
            .fold((Vector2D::new(0.0,0.0),0),|(sum_position,count),position| {
                (sum_position + position,count + 1)
            });
        
        if count > 0 {
            steering = steering / count as f64;
            steering = steering.magnitude(self.max_velocity);
            steering = steering - self.velocity;
            steering = steering.limit(self.max_force);
            steering
        } else {
            Vector2D::new(0.0,0.0)
        }
    }

    pub fn compute_force(&self,  boids:  &Vec<Boid>) -> Vector2D<f64> {
        let align_force = self.align(boids);
        let cohesion_force = self.cohesion(boids);
        let separation_force = self.separation(boids);
        align_force * 0.8 + 
            cohesion_force * 0.8 + 
            separation_force * 0.4
    }

    pub fn update(&mut self,  force: &Vector2D<f64>) {
        self.position = self.position + self.velocity;
        self.velocity = self.velocity + *force;
        self.velocity = self.velocity.limit(self.max_velocity);
    }

    pub fn edges(&mut self, width: u32, height: u32){
        if self.position.x > width as f64 {
            self.position.x = 0.0
        }
        if self.position.x < 0.0 {
            self.position.x = width as f64
        }
        if self.position.y > height as f64 {
            self.position.y = 0.0
        }
        if self.position.y < 0.0 {
            self.position.y = height as f64
        }
    }

    pub fn display(&mut self, canvas: &mut Ssl2Canvas) -> Result<(),String>{
        let offset = self.velocity.normalise()*5.0;
        canvas.line(
            (self.position.x + offset.x) as i16,
            (self.position.y + offset.y) as i16,
            (self.position.x - offset.x) as i16,
            (self.position.y - offset.y) as i16,
            Color::RGB(255,255,255)
        )
        //canvas.filled_circle( self.position.x as i16, self.position.y as i16, 5,Color::RGB(255,255,255))
    }
}


pub struct World {
    width: u32,
    height: u32,
    flock: Vec<Boid>,
    forces: Vec<Vector2D<f64>>
}

impl World {
    pub fn new(width: u32, height: u32, flock_size: u32) -> World {
        
        let flock =  (0..flock_size).map(|_| Boid::random(1.0, 6.0, width as f64, height as f64, 1.0, 6.0, 50.0,50.0,100.0)).collect();
        let forces = (0..flock_size).map(|_| Vector2D::new(0.0,0.0)).collect();
        World { width, height, flock, forces }
    }

    pub fn display(&mut self, canvas: &mut Ssl2Canvas) -> Result<(),String>{   
        for boid in self.flock.iter_mut() {
            (*boid).display(canvas)?;
        }  
        Ok(())
    }

    pub fn update(&mut self){
        // Compute forces
        for i in 0..self.flock.len() {
            let boid = self.flock.get(i).unwrap();
            let force_to_update = self.forces.get_mut(i).unwrap();
            let force = boid.compute_force(&self.flock);
            force_to_update.x = force.x;
            force_to_update.y = force.y;
        }
        // Update boids
        for i in 0..self.flock.len() {
            let boid = self.flock.get_mut(i).unwrap();
            let force = self.forces.get_mut(i).unwrap();
            boid.update(&force);
            boid.edges(self.width,self.height);
        }
    }
}



pub fn create_canvas(title: &str, width: u32, height: u32) -> Result<(Ssl2Canvas,Sdl),Box<Error>>{
    let sdl_context = sdl2::init()?;
    let video_subsystem = sdl_context.video()?;

    let window = video_subsystem.window(title, width, height)
        .position_centered()
        .build()?;
    let mut canvas = window.into_canvas().build().unwrap();
    canvas.set_draw_color(Color::RGB(0, 255, 255));
    canvas.clear();
    canvas.present();
    Ok((canvas,sdl_context))
}


pub fn main() -> Result<(),Box<Error>> {
    let width: u32 = 1024;
    let height: u32 = 768;

    let (mut canvas, sdl_context) = create_canvas("Boids!", width, height).unwrap();
    let mut world = World::new(width, height, 1000);
    
    let mut event_pump = sdl_context.event_pump().unwrap();
    loop {
        canvas.set_draw_color(Color::RGB(0, 0, 0));
        canvas.clear();
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit {..} |
                Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                    return Ok(())
                },
                _ => {}
            }
        }
        
        world.update();
        world.display(&mut canvas)?;
        

        canvas.present();
        ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 60));
        //  ::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 20));
    }
}