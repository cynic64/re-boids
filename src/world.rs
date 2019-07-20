use re::*;
use rand::Rng;

extern crate nalgebra as na;
use na::{Point3, Vector3};

const WORLD_RADIUS: f32 = 32.0;
const BOID_SIZE: f32 = 0.3;
const MAX_VEL: f32 = 5.0;

pub struct BoidWorld {
    boids: Vec<Point3<f32>>,
    velocities: Vec<Vector3<f32>>,
    world_com: WorldCommunicator,
}

impl BoidWorld {
    pub fn new(world_com: WorldCommunicator) -> Self {
        let mut rng = rand::thread_rng();

        let boids: Vec<_> = (0..1_000).map(|_| Point3::new(
            rng.gen_range(-WORLD_RADIUS, WORLD_RADIUS),
            rng.gen_range(-WORLD_RADIUS, WORLD_RADIUS),
            rng.gen_range(-WORLD_RADIUS, WORLD_RADIUS),
        )).collect();

        let velocities = vec![Vector3::new(0.0, 0.0, 0.0); 100];

        BoidWorld {
            boids,
            velocities,
            world_com,
        }
    }

    pub fn update(&mut self, delta: f32) {
        self.world_com.delete_object("boids".to_string());

        let sum_x: f32 = self.boids.iter().map(|boid| boid.x).sum();
        let sum_y: f32 = self.boids.iter().map(|boid| boid.y).sum();
        let sum_z: f32 = self.boids.iter().map(|boid| boid.z).sum();
        let div = self.boids.len() as f32;
        let center_pos = Point3::new(sum_x / div, sum_y / div, sum_z / div);

        self.velocities = self.boids.iter().map(|pos| {
            let cohesion_vel = (center_pos - pos).normalize() * MAX_VEL * delta;

            cohesion_vel
        }).collect();

        self.boids = self.boids.iter().enumerate().map(|(idx, boid)| boid + self.velocities[idx]).collect();

        let verts: Vec<_> = self.boids.iter().flat_map(|pos| mesh_gen::create_vertices_for_sphere([pos.x, pos.y, pos.z], BOID_SIZE, [1.0, 1.0, 1.0])).collect();
        let object_spec = ObjectSpec::from_mesh(verts);

        self.world_com.add_object_from_spec("boids".to_string(), object_spec);
    }
}
