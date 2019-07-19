extern crate nalgebra as na;
use na::{Point3, Vector3};

extern crate rand;
use rand::Rng;

use re::*;

const BOID_COLOR: [f32; 3] = [1.0, 1.0, 1.0];
const BOID_SIZE: f32 = 0.3;

pub struct Boid {
    id: String,
    world_com: WorldCommunicator,
    position: Point3<f32>,
    velocity: Vector3<f32>,
}

impl Boid {
    pub fn new(id: String, world_com: WorldCommunicator, position: (f32, f32, f32)) -> Self {
        let mut rng = rand::thread_rng();
        let velocity = Vector3::new(rng.gen_range(-1.0, 1.0), rng.gen_range(-1.0, 1.0), rng.gen_range(-1.0, 1.0));

        Self {
            id,
            world_com,
            position: Point3::new(position.0, position.1, position.2),
            velocity,
        }
    }

    pub fn update_position(&mut self, delta: f32) {
        self.position += self.velocity * delta;
    }

    pub fn update_mesh(&mut self) {
        let mesh = mesh_gen::create_vertices_for_sphere([self.position.x, self.position.y, self.position.z], BOID_SIZE, BOID_COLOR);
        let object_spec = ObjectSpec::from_mesh(mesh);
        self.world_com.delete_object(self.id.clone());
        self.world_com.add_object_from_spec(self.id.clone(), object_spec);
    }
}
