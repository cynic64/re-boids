extern crate render_engine as re;
use re::*;

mod boid;
use boid::Boid;

fn main() {
    let mut app = App::new();
    app.enable_multisampling();
    let mut camera = OrbitCamera::default();
    camera.orbit_distance = 10.0;
    app.camera = Box::new(camera);

    let mut boid = Boid::new("jeff".to_string(), app.get_world_com(), (0.0, 0.0, 0.0));

    while !app.done {
        boid.update_mesh();

        app.draw_frame();
    }
}
