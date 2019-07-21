use rand::Rng;
use re::*;

extern crate nalgebra as na;
use na::{Point3, Vector3};

use crate::WORLD_RADIUS;

const BOID_SIZE: f32 = 0.1;
const MAX_VEL: f32 = 5.0;

pub struct BoidWorld {
    boids: Vec<Point3<f32>>,
    velocities: Vec<Vector3<f32>>,
    attractor: Point3<f32>,
    repulsor: Point3<f32>,
    world_com: WorldCommunicator,
    start_time: std::time::Instant,
}

impl BoidWorld {
    pub fn new(world_com: WorldCommunicator) -> Self {
        let mut rng = rand::thread_rng();

        let boids: Vec<_> = (0..1_000)
            .map(|_| {
                Point3::new(
                    rng.gen_range(-WORLD_RADIUS, WORLD_RADIUS),
                    rng.gen_range(-WORLD_RADIUS, WORLD_RADIUS),
                    rng.gen_range(-WORLD_RADIUS, WORLD_RADIUS),
                )
            })
            .collect();

        let attractor = Point3::new(0.0, 0.0, 0.0);
        let repulsor = Point3::new(0.0, 0.0, 0.0);

        let velocities = vec![Vector3::new(0.0, 0.0, 0.0); 1_000];

        BoidWorld {
            boids,
            velocities,
            attractor,
            repulsor,
            world_com,
            start_time: std::time::Instant::now(),
        }
    }

    pub fn update(&mut self, delta: f32) {
        self.world_com.delete_object("boids".to_string());
        self.world_com.delete_object("attractor".to_string());
        self.world_com.delete_object("repulsor".to_string());

        let sum_x: f32 = self.boids.iter().map(|boid| boid.x).sum();
        let sum_y: f32 = self.boids.iter().map(|boid| boid.y).sum();
        let sum_z: f32 = self.boids.iter().map(|boid| boid.z).sum();
        let div = self.boids.len() as f32;
        let center_pos = Point3::new(sum_x / div, sum_y / div, sum_z / div);

        let v_sum_x: f32 = self.velocities.iter().map(|vel| vel.x).sum();
        let v_sum_y: f32 = self.velocities.iter().map(|vel| vel.y).sum();
        let v_sum_z: f32 = self.velocities.iter().map(|vel| vel.z).sum();
        let avg_vel = Vector3::new(v_sum_x / div, v_sum_y / div, v_sum_z / div);

        let attractor = self.attractor.clone();
        let repulsor = self.repulsor.clone();

        self.velocities = self
            .boids
            .iter()
            .enumerate()
            .map(|(idx, pos)| {
                let attraction = (attractor - pos).normalize() * MAX_VEL * 2.0;
                let repulsion_distance = ((repulsor.x - pos.x).powi(2)
                    + (repulsor.y - pos.y).powi(2)
                    + (repulsor.z - pos.z).powi(2))
                .sqrt();
                let repulsion = (repulsor - pos).normalize() * -1.0 * (1.0 / repulsion_distance) * 100.0;

                let cohesion = (center_pos - pos).normalize() * MAX_VEL;
                let separation = self
                    .boids
                    .iter()
                    .enumerate()
                    .filter_map(|(idx, &boid)| {
                        let distance = ((boid.x - pos.x).powi(2)
                            + (boid.y - pos.y).powi(2)
                            + (boid.z - pos.z).powi(2))
                        .sqrt();
                        if distance < 2.0 && boid != *pos {
                            Some((idx, boid, distance - BOID_SIZE * 3.0))
                        } else {
                            None
                        }
                    })
                    .fold(
                        (0.0, 0.0, 0.0),
                        |acc: (f32, f32, f32), (idx, _boid, distance)| {
                            (
                                acc.0 + (self.boids[idx].x - pos.x) * (1.0 / distance),
                                acc.1 + (self.boids[idx].y - pos.y) * (1.0 / distance),
                                acc.2 + (self.boids[idx].z - pos.z) * (1.0 / distance),
                            )
                        },
                    );

                let force = Vector3::new(
                    cohesion.x - separation.0 + avg_vel.x + attraction.x + repulsion.x,
                    cohesion.y - separation.1 + avg_vel.y + attraction.y + repulsion.y,
                    cohesion.z - separation.2 + avg_vel.z + attraction.z + repulsion.z,
                ) * delta
                    * 4.0;
                let mut new_vel = self.velocities[idx] + force;
                if (new_vel.x * new_vel.x + new_vel.y * new_vel.y + new_vel.z * new_vel.z).sqrt()
                    > MAX_VEL
                {
                    new_vel = new_vel.normalize() * MAX_VEL;
                }

                new_vel
            })
            .collect();

        self.boids = self
            .boids
            .iter()
            .enumerate()
            .map(|(idx, boid)| boid + self.velocities[idx] * delta)
            .collect();

        let verts: Vec<_> = self
            .boids
            .iter()
            .flat_map(|pos| {
                mesh_gen::create_vertices_for_sphere(
                    [pos.x, pos.y, pos.z],
                    BOID_SIZE,
                    [1.0, 1.0, 1.0],
                )
            })
            .collect();
        let object_spec = ObjectSpec::from_mesh(verts);
        self.world_com
            .add_object_from_spec("boids".to_string(), object_spec);

        let time = get_elapsed(self.start_time);
        self.attractor.x = (time / 4.0).sin() * 16.0;
        self.attractor.z = (time / 4.0).cos() * 16.0;
        let attractor_verts = mesh_gen::create_vertices_for_sphere([self.attractor.x, self.attractor.y, self.attractor.z], BOID_SIZE * 8.0, [0.0, 0.5, 1.0]);
        let attractor_spec = ObjectSpec::from_mesh(attractor_verts);
        self.world_com.add_object_from_spec("attractor".to_string(), attractor_spec);

        self.repulsor.x = (time).sin() * 16.0;
        self.repulsor.y = (time).cos() * 16.0;
        let repulsor_verts = mesh_gen::create_vertices_for_sphere([self.repulsor.x, self.repulsor.y, self.repulsor.z], BOID_SIZE * 8.0, [1.0, 0.0, 0.5]);
        let repulsor_spec = ObjectSpec::from_mesh(repulsor_verts);
        self.world_com.add_object_from_spec("repulsor".to_string(), repulsor_spec);
    }
}
