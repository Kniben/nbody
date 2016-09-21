extern crate piston;
extern crate graphics;
extern crate glutin_window;
extern crate opengl_graphics;
extern crate rand;

use piston::window::WindowSettings;
use piston::event_loop::*;
use piston::input::*;
use glutin_window::GlutinWindow as Window;
use opengl_graphics::{ GlGraphics, OpenGL };
use rand::Rng;

pub struct App {
    gl: GlGraphics, // OpenGL drawing backend.
    rotation: f64,   // Rotation for the square.
    bodies: Vec<Box<Body>>
}

pub struct Body {
    position: (f64, f64),
    velocity: (f64, f64),
    mass: f64
}

/*
impl Body {
    fn new(pos: (f64, f64), vel: (f64, f64), m: f64) -> Body {
        Body {
            position: pos,
            velocity: vel,
            mass: m
        }
    }
}
*/

impl App {    
    fn render(&mut self, args: &RenderArgs) {
        use graphics::*;

        const GREEN: [f32; 4] = [0.0, 1.0, 0.0, 1.0];
        const BLACK: [f32; 4] = [0.0, 0.0, 0.0, 1.0];

        let square = rectangle::square(0.0, 0.0, 1.5);
        let rotation = self.rotation;

        clear(BLACK, &mut self.gl);

        for i in 0..self.bodies.len() {
            let (x, y) = ((args.width / 2) as f64 + self.bodies[i].position.0,
                          (args.height / 2) as f64 + self.bodies[i].position.1);
            
            self.gl.draw(args.viewport(), |c, gl| {
                let transform = c.transform.trans(x, y);
                ellipse(GREEN, square, transform, gl);
            });
        }
    }

    fn update(&mut self, args: &UpdateArgs) {
        fn simulate_gravity(dt: f64, bodies: &mut Vec<Box<Body>>) {
            if bodies.len() > 1 {
                // F1 = F2 = G m1 * m2 / r^2
                // F = ma    |    a = F / m
                // a1 = F1 / m1 = G * m2 / r^2
                
                // u.0 = apos.0    u.0' = u4
                // u.1 = apos.1    u.1' = u5
                // u.2 = bpos.0    u.2' = u6
                // u.3 = bpos.1    u.3' = u7
                // u.4 = avel.0    u.4' = dx / r * force / mb
                // u.5 = avel.1    u.5' = dy / r * force / mb
                // u.6 = bvel.0    u.6' = dx / r * force / ma
                // u.7 = bvel.1    u.7' = dy / r * force / ma
                
                fn delta(dt: f64, u: &[f64; 8], amass: f64, bmass: f64) 
                         -> [f64; 8] {
                    let dx = u[2] - u[0];
                    let dy = u[3] - u[1];
                    let eps = 0.00001 * dt;
                    let sqdist = dx * dx + dy * dy;
                    let r2 = sqdist.max(eps);
                    let r = r2.sqrt();
                    let force =  (6.674e-11 * dt) / r2;
                    let xforce = (dx * force) / r;
                    let yforce = (dy * force) / r;
                    
                    [u[4], u[5], u[6], u[7],
                     xforce / bmass,
                     yforce / bmass,
                     -xforce / amass,
                     -yforce / amass]
                }
                
                let npairs = bodies.len() * (bodies.len() - 1) / 2;

                let mut temp = [0.0; 8];

                let mut u: Vec<[f64; 8]> = Vec::with_capacity(npairs);
                let mut k1: Vec<[f64; 8]> = Vec::with_capacity(npairs);
                let mut k2: Vec<[f64; 8]> = Vec::with_capacity(npairs);
                let mut k3: Vec<[f64; 8]> = Vec::with_capacity(npairs);
                let mut k4: Vec<[f64; 8]> = Vec::with_capacity(npairs);

                // K1
                let mut pairi = 0;
                for a in 0..(bodies.len() - 1) {
                    for b in (a + 1)..bodies.len() {
                        // For each pair of bodies
                        // Init u
                        u.push([bodies[a].position.0,
                                       bodies[a].position.1,
                                       bodies[b].position.0,
                                       bodies[b].position.1,
                                       bodies[a].velocity.0,
                                       bodies[a].velocity.1,
                                       bodies[b].velocity.0,
                                       bodies[b].velocity.1]);
                            
                        k1.push(delta(dt, &u[pairi], bodies[a].mass, bodies[b].mass));
                        pairi += 1;
                    }
                }
                
                // K2
                pairi = 0;
                for a in 0..(bodies.len() - 1) {
                    for b in (a + 1)..bodies.len() {
                        // For each pair of bodies
                        for k in 0..temp.len() {
                            temp[k] = u[pairi][k] + 1.0 / 2.0 * k1[pairi][k];
                        }

                        k2.push(delta(dt, &temp, bodies[a].mass, bodies[b].mass));
                        pairi += 1;
                    }
                }
                
                // K3
                pairi = 0;
                for a in 0..(bodies.len() - 1) {
                    for b in (a + 1)..bodies.len() {
                        // For each pair of bodies
                        for k in 0..temp.len() {
                            temp[k] = u[pairi][k] + 1.0 / 2.0 * k2[pairi][k];
                        }

                        k3.push(delta(dt, &temp, bodies[a].mass, bodies[b].mass));
                        pairi += 1;
                    }
                }
                
                // K4
                pairi = 0;
                for a in 0..(bodies.len() - 1) {
                    for b in (a + 1)..bodies.len() {
                        // For each pair of bodies
                        for k in 0..temp.len() {
                            temp[k] = u[pairi][k] + 1.0 * k3[pairi][k];
                        }

                        k4.push(delta(dt, &temp, bodies[a].mass, bodies[b].mass));
                        pairi += 1;
                    }
                }

                // Compute next step
                pairi = 0;
                for a in 0..(bodies.len() - 1) {
                    for b in (a + 1)..bodies.len() {
                        // For each pair of bodies
                        for k in 0..temp.len() {
                            temp[k] = 1.0 / 6.0 * (k1[pairi][k] 
                                                   + 2.0 * k2[pairi][k] 
                                                   + 2.0 * k3[pairi][k] 
                                                   + k4[pairi][k]);
                        }
                        
                        // Change by difference
                        bodies[a].position.0 += temp[0];
                        bodies[a].position.1 += temp[1];
                        bodies[b].position.0 += temp[2];
                        bodies[b].position.1 += temp[3];
                        bodies[a].velocity.0 += temp[4];
                        bodies[a].velocity.1 += temp[5];
                        bodies[b].velocity.0 += temp[6];
                        bodies[b].velocity.1 += temp[7];
                        
                        pairi += 1;
                    }
                }
            }
        }

        simulate_gravity(100_000_000.0, &mut self.bodies);
    }
}

fn main() {
    // Change this to OpenGL::V2_1 if not working.
    let opengl = OpenGL::V3_2;
    
    // Create an Glutin window.
    let mut window: Window = WindowSettings::new(
        "nbody sim",
        [800, 600]
    ).opengl(opengl)
        .exit_on_esc(true)
        .build()
        .unwrap();
    
    fn gen_body(mass: f64) -> Body {
        let mut rng = rand::thread_rng();
        fn rpos(rng: &mut rand::ThreadRng) -> (f64, f64) {
            ((rng.gen::<f64>() - 0.5) * 100.0,
             (rng.gen::<f64>() - 0.5) * 100.0)
        }

        fn rvel(rng: &mut rand::ThreadRng) -> (f64, f64) {
            ((rng.gen::<f64>() - 0.5) * 0.001,
             (rng.gen::<f64>() - 0.5) * 0.001)
        }

        let mut b = Body {
            position: rpos(&mut rng),
            velocity: rvel(&mut rng),
            mass: mass
        };

        b
    }
    
    let mut v = Vec::new();

    for i in 0..100 {
        v.push(Box::new(genBody(1.0)));
    }
    
    v.push(Box::new(gen_body(10000.0)));

    // Create a new game and run it.
    let mut app = App {
        gl: GlGraphics::new(opengl),
        rotation: 0.0,
        bodies: v
    };

    let mut events = window.events();
    while let Some(e) = events.next(&mut window) {
        if let Some(r) = e.render_args() {
            app.render(&r);
        }

        if let Some(u) = e.update_args() {
            app.update(&u);
        }
    }
}
