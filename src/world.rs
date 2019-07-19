use re::*;

use rand::Rng;

mod boid;
use boid::Boid;

const WORLD_RADIUS: f32 = 32.0;

pub struct BoidWorld {
    boids: Vec<Boid>,
}

impl BoidWorld {
    pub fn new(world_com: WorldCommunicator) -> Self {
        let mut rng = rand::thread_rng();

        let boids: Vec<_> = (0..100).map(|x| Boid::new(x.to_string(), world_com.clone(), (
            rng.gen_range(-WORLD_RADIUS, WORLD_RADIUS),
            rng.gen_range(-WORLD_RADIUS, WORLD_RADIUS),
            rng.gen_range(-WORLD_RADIUS, WORLD_RADIUS),
        ))).collect();

        BoidWorld {
            boids,
        }
    }

    pub fn update(&mut self, delta: f32) {
        self.boids.iter_mut().for_each(|boid| {
            boid.update_position(delta);
            boid.update_mesh();
        });
    }
}
