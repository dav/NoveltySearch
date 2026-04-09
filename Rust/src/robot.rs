use crate::maze::Maze;

pub const ROBOT_RADIUS: f64 = 8.0;
pub const MAX_SENSOR_RANGE: f64 = 100.0;

/// Rangefinder angles relative to heading (in radians).
/// -90°, -45°, 0° (forward), 45°, 90°, 180° (rear)
const RANGEFINDER_ANGLES: [f64; 6] = [
    -std::f64::consts::FRAC_PI_2,      // -90°
    -std::f64::consts::FRAC_PI_4,      // -45°
    0.0,                                //   0°
    std::f64::consts::FRAC_PI_4,       //  45°
    std::f64::consts::FRAC_PI_2,       //  90°
    std::f64::consts::PI,              // 180°
];

pub struct Robot {
    pub x: f64,
    pub y: f64,
    pub heading: f64, // radians, 0 = right, pi/2 = down
    pub radius: f64,
}

impl Robot {
    pub fn new(x: f64, y: f64) -> Self {
        Robot {
            x,
            y,
            heading: 0.0,
            radius: ROBOT_RADIUS,
        }
    }

    /// Move the robot by one timestep.
    /// `ang_vel` rotates heading (radians per step).
    /// `speed` moves forward along heading (units per step).
    pub fn step(&mut self, ang_vel: f64, speed: f64, maze: &Maze) {
        self.heading += ang_vel;
        // Keep heading in [0, 2π)
        self.heading = self.heading.rem_euclid(std::f64::consts::TAU);

        let new_x = self.x + self.heading.cos() * speed;
        let new_y = self.y + self.heading.sin() * speed;

        let resolved = maze.resolve_collision((new_x, new_y), self.radius);
        self.x = resolved.0;
        self.y = resolved.1;
    }

    /// Cast 6 rangefinder rays and return normalized distances [0, 1].
    /// 0 = touching a wall, 1 = nothing within max range.
    pub fn rangefinders(&self, maze: &Maze) -> [f64; 6] {
        let mut readings = [0.0; 6];

        for (i, &offset) in RANGEFINDER_ANGLES.iter().enumerate() {
            let angle = self.heading + offset;
            let dir = (angle.cos(), angle.sin());
            let dist = maze.ray_cast((self.x, self.y), dir, MAX_SENSOR_RANGE);
            readings[i] = dist / MAX_SENSOR_RANGE; // normalize to [0, 1]
        }

        readings
    }

    /// Return the raw (non-normalized) rangefinder hit points for visualization.
    pub fn rangefinder_endpoints(&self, maze: &Maze) -> [(f64, f64); 6] {
        let mut endpoints = [(0.0, 0.0); 6];

        for (i, &offset) in RANGEFINDER_ANGLES.iter().enumerate() {
            let angle = self.heading + offset;
            let dir = (angle.cos(), angle.sin());
            let dist = maze.ray_cast((self.x, self.y), dir, MAX_SENSOR_RANGE);
            endpoints[i] = (self.x + dir.0 * dist, self.y + dir.1 * dist);
        }

        endpoints
    }

    /// 4 pie-slice radar sensors for goal direction.
    /// Each covers a 90° slice around the robot's heading.
    /// Returns [front, right, back, left] — 1.0 if goal is in that slice, else 0.0.
    pub fn radar(&self, goal: (f64, f64)) -> [f64; 4] {
        let dx = goal.0 - self.x;
        let dy = goal.1 - self.y;
        let angle_to_goal = dy.atan2(dx);

        // Relative angle from heading, normalized to [0, 2π)
        let relative = (angle_to_goal - self.heading).rem_euclid(std::f64::consts::TAU);

        // Determine which quadrant (each is 90° = π/2)
        // 0: front  [315°..360°) ∪ [0°..45°)
        // 1: right  [45°..135°)
        // 2: back   [135°..225°)
        // 3: left   [225°..315°)
        let mut sensors = [0.0; 4];
        let deg = relative.to_degrees();

        let quadrant = if deg < 45.0 || deg >= 315.0 {
            0 // front
        } else if deg < 135.0 {
            1 // right
        } else if deg < 225.0 {
            2 // back
        } else {
            3 // left
        };

        sensors[quadrant] = 1.0;
        sensors
    }

    /// Full sensor input vector: 6 rangefinders + 4 radar + 1 bias = 11 values.
    pub fn sensor_inputs(&self, maze: &Maze) -> [f64; 11] {
        let rf = self.rangefinders(maze);
        let rd = self.radar(maze.goal);

        let mut inputs = [0.0; 11];
        inputs[..6].copy_from_slice(&rf);
        inputs[6..10].copy_from_slice(&rd);
        inputs[10] = 1.0; // bias
        inputs
    }

    /// Distance from robot center to goal.
    pub fn distance_to_goal(&self, goal: (f64, f64)) -> f64 {
        let dx = goal.0 - self.x;
        let dy = goal.1 - self.y;
        (dx * dx + dy * dy).sqrt()
    }
}
