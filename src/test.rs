fn main() {
    let mut state = [0.0, 0.0, 0.0, 1.0,
                     0.1, 0.0, 0.0, 0.0];

    loop {
        simulate_gravity(0.1, 1.0, 1.0, &mut state);
        println!("\n {:.*}", 2, state[1]);
    }
}

fn simulate_gravity(dt: f64,
                    ma: f64,
                    mb: f64,
                    u: &mut [f64; 8]) {
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
    
    fn f(u: &[f64; 8], ma: f64, mb: f64) -> [f64; 8] {
        let dx = u[2] - u[0];
        let dy = u[3] - u[1];
        let r2 = dx.powf(2.0) + dy.powf(2.0);
        let r = r2.sqrt();
        let force = 6.674e-11 / r2;
        let a_factor = force / mb / r;
        let b_factor = force / ma / r;
        println!("Force: {}", force);
        
        [u[4], u[5], u[6], u[7],
         dx * a_factor,
         dy * a_factor,
         -dx * b_factor,
         -dy * b_factor]
    }
    
    let mut temp = [0.0; 8];

    // K1
    let k_1 = f(u, ma, mb);
    
    // K2
    for i in 0..temp.len() {
        temp[i] = u[i] + dt / 2.0 * k_1[i];
    }
    let k_2 = f(&temp, ma, mb);

    // K3
    for i in 0..temp.len() {
        temp[i] = u[i] + dt / 2.0 * k_2[i];
    }  
    let k_3 = f(&temp, ma, mb);

    // K4
    for i in 0..temp.len() {
        temp[i] = u[i] + dt * k_3[i];
    }  
    let k_4 = f(&temp, ma, mb);

    // Compute next step
    for i in 0..temp.len() {
        u[i] = u[i] + dt / 6.0 * (k_1[i] + 2.0 * k_2[i] + 2.0 * k_3[i] + k_4[i]);
    }
}