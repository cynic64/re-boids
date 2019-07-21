extern crate render_engine as re;
use re::*;

mod world;
use world::BoidWorld;

const WORLD_RADIUS: f32 = 32.0;

fn main() {
    let mut app = App::new();
    app.enable_multisampling();
    let mut camera = OrbitCamera::default();
    camera.orbit_distance = WORLD_RADIUS * 2.0;
    app.camera = Box::new(camera);

    let mut world_com = app.get_world_com();
    let mesh = mesh_gen::create_vertices_for_cube_edges([0.0, 0.0, 0.0], WORLD_RADIUS, [1.0, 0.0, 0.0]);
    let mut spec = ObjectSpec::from_mesh(mesh);
    spec.switch_fill_type(PrimitiveTopology::LineList);
    world_com.add_object_from_spec("bounding box".to_string(), spec);

    let mut world = BoidWorld::new(world_com);

    while !app.done {
        world.update(app.delta);

        app.draw_frame();
    }

    app.print_fps();
}
