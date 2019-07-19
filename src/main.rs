extern crate render_engine as re;
use re::*;

mod world;
use world::BoidWorld;

fn main() {
    let mut app = App::new();
    app.enable_multisampling();
    let mut camera = OrbitCamera::default();
    camera.orbit_distance = 10.0;
    app.camera = Box::new(camera);

    let mut world = BoidWorld::new(app.get_world_com());

    while !app.done {
        world.update(app.delta);

        app.draw_frame();
    }
}
