/// A wall is a line segment between two points.
#[derive(Clone, Debug)]
pub struct Wall {
    pub a: (f64, f64),
    pub b: (f64, f64),
}

/// A maze with walls, a start position, a goal position, and bounding dimensions.
#[derive(Clone, Debug)]
pub struct Maze {
    pub walls: Vec<Wall>,
    pub start: (f64, f64),
    pub goal: (f64, f64),
    pub bounds: (f64, f64),
}

impl Wall {
    pub fn new(ax: f64, ay: f64, bx: f64, by: f64) -> Self {
        Wall {
            a: (ax, ay),
            b: (bx, by),
        }
    }
}

impl Maze {
    /// Cast a ray from `origin` in `direction` and return the distance to the nearest wall hit.
    /// Returns `max_range` if no wall is hit within that range.
    pub fn ray_cast(&self, origin: (f64, f64), direction: (f64, f64), max_range: f64) -> f64 {
        let mut nearest = max_range;

        for wall in &self.walls {
            if let Some(t) = ray_segment_intersection(origin, direction, wall.a, wall.b) {
                if t > 0.0 && t < nearest {
                    nearest = t;
                }
            }
        }

        nearest
    }

    /// Push a circle (position + radius) out of all walls it overlaps.
    /// Returns the corrected position.
    pub fn resolve_collision(&self, pos: (f64, f64), radius: f64) -> (f64, f64) {
        let mut corrected = pos;

        // Multiple passes to handle corners where two walls meet
        for _ in 0..4 {
            let mut pushed = false;
            for wall in &self.walls {
                if let Some(push) = circle_segment_push(corrected, radius, wall.a, wall.b) {
                    corrected.0 += push.0;
                    corrected.1 += push.1;
                    pushed = true;
                }
            }
            if !pushed {
                break;
            }
        }

        // Also clamp to maze bounds
        corrected.0 = corrected.0.clamp(radius, self.bounds.0 - radius);
        corrected.1 = corrected.1.clamp(radius, self.bounds.1 - radius);

        corrected
    }

    /// The "medium" maze — a simple maze with a few internal walls.
    /// Roughly based on Figure 4.2a from Lehman's dissertation.
    pub fn medium() -> Self {
        let (w, h) = (200.0, 200.0);
        let mut walls = boundary_walls(w, h);

        // Internal walls creating a path with one main turn
        walls.extend([
            // Horizontal wall from left, creating an upper passage
            Wall::new(0.0, 66.0, 140.0, 66.0),
            // Horizontal wall creating a lower passage
            Wall::new(60.0, 133.0, 200.0, 133.0),
        ]);

        Maze {
            walls,
            start: (30.0, 180.0),   // bottom-left area
            goal: (170.0, 33.0),    // top-right area
            bounds: (w, h),
        }
    }

    /// The "hard" maze — an S-shaped path that is deceptive for fitness-based search.
    ///
    /// Layout (y=0 is top):
    /// ```
    ///   +-------------------------------+
    ///   |                     |gap| [G] |  y=0..50    (goal at top-right)
    ///   |---------------------+   +-----|  y=50       (gap at x=230..260)
    ///   |  |gap|                        |  y=50..110
    ///   |--+   +------------------------|  y=110      (gap at x=40..70)
    ///   |                     |gap|     |  y=110..170
    ///   |---------------------+   +-----|  y=170      (gap at x=230..260)
    ///   | [S]                           |  y=170..200 (start at bottom-left)
    ///   +-------------------------------+
    /// ```
    ///
    /// The path requires going right → up → left → up → right → up to goal.
    /// Fitness-based search gets trapped pressing against walls near (250, 170)
    /// because that's close to the goal in Euclidean distance, but far along the
    /// actual path.
    pub fn hard() -> Self {
        let (w, h) = (300.0, 200.0);
        let mut walls = boundary_walls(w, h);

        walls.extend([
            // Bottom horizontal wall — gap at x=230..260
            Wall::new(0.0, 170.0, 230.0, 170.0),
            Wall::new(260.0, 170.0, 300.0, 170.0),
            // Middle horizontal wall — gap at x=40..70
            Wall::new(0.0, 110.0, 40.0, 110.0),
            Wall::new(70.0, 110.0, 300.0, 110.0),
            // Top horizontal wall — gap at x=230..260
            Wall::new(0.0, 50.0, 230.0, 50.0),
            Wall::new(260.0, 50.0, 300.0, 50.0),
        ]);

        Maze {
            walls,
            start: (30.0, 185.0),    // bottom-left
            goal: (270.0, 25.0),     // top-right
            bounds: (w, h),
        }
    }

    /// Generate a random maze using Eller's algorithm.
    ///
    /// Eller's algorithm builds a perfect maze (exactly one path between any two cells)
    /// row by row. It's memory-efficient because it only needs to track the current row.
    ///
    /// The algorithm:
    /// 1. Assign each cell in the first row to its own set
    /// 2. For each row:
    ///    a. Randomly merge adjacent cells in different sets (remove right wall)
    ///    b. For each set, keep at least one downward connection (remove bottom wall)
    /// 3. Last row: merge all adjacent cells in different sets
    pub fn eller() -> Self {
        use rand::Rng;

        let mut rng = rand::rng();
        let cols = 8;
        let rows = 8;
        let (maze_w, maze_h) = (300.0, 300.0);
        let cell_w = maze_w / cols as f64;
        let cell_h = maze_h / rows as f64;

        // Track which walls exist: right walls and bottom walls for each cell
        // true = wall exists, false = wall removed (passage)
        let mut right_walls = vec![vec![true; cols]; rows];
        let mut bottom_walls = vec![vec![true; cols]; rows];

        // Set ID for each cell in the current row
        let mut sets = vec![0usize; cols];
        let mut next_set_id = 0;

        // Initialize first row: each cell in its own set
        for cell in sets.iter_mut() {
            *cell = next_set_id;
            next_set_id += 1;
        }

        for row in 0..rows {
            // Step 1: Randomly merge adjacent cells in different sets (remove right wall)
            for col in 0..cols - 1 {
                if sets[col] != sets[col + 1] && (row == rows - 1 || rng.random_bool(0.5)) {
                    // Merge: remove right wall and unify sets
                    right_walls[row][col] = false;
                    let old_set = sets[col + 1];
                    let new_set = sets[col];
                    for s in sets.iter_mut() {
                        if *s == old_set {
                            *s = new_set;
                        }
                    }
                }
            }

            if row == rows - 1 {
                break; // last row — no bottom walls to process
            }

            // Step 2: For each set, ensure at least one downward connection
            // First, collect which columns belong to each set
            let mut set_columns: std::collections::HashMap<usize, Vec<usize>> = std::collections::HashMap::new();
            for (col, &set_id) in sets.iter().enumerate() {
                set_columns.entry(set_id).or_default().push(col);
            }

            // For each set, randomly choose at least one cell to connect downward
            let mut connected_down = vec![false; cols];
            for (_set_id, columns) in &set_columns {
                // Randomly pick how many to connect (at least 1)
                let mut made_one = false;
                for &col in columns {
                    if rng.random_bool(0.4) || !made_one {
                        bottom_walls[row][col] = false;
                        connected_down[col] = true;
                        made_one = true;
                    }
                }
            }

            // Step 3: Prepare next row
            // Cells connected downward keep their set; others get new sets
            for col in 0..cols {
                if !connected_down[col] {
                    sets[col] = next_set_id;
                    next_set_id += 1;
                }
            }
        }

        // Convert grid walls to Wall segments
        let mut walls = boundary_walls(maze_w, maze_h);

        for row in 0..rows {
            for col in 0..cols {
                let x = col as f64 * cell_w;
                let y = row as f64 * cell_h;

                // Right wall
                if col < cols - 1 && right_walls[row][col] {
                    walls.push(Wall::new(x + cell_w, y, x + cell_w, y + cell_h));
                }

                // Bottom wall
                if row < rows - 1 && bottom_walls[row][col] {
                    walls.push(Wall::new(x, y + cell_h, x + cell_w, y + cell_h));
                }
            }
        }

        // Start at top-left cell center, goal at bottom-right cell center
        let start = (cell_w / 2.0, cell_h / 2.0);
        let goal = (maze_w - cell_w / 2.0, maze_h - cell_h / 2.0);

        Maze {
            walls,
            start,
            goal,
            bounds: (maze_w, maze_h),
        }
    }
}

/// Generate the four boundary walls for a rectangular maze.
fn boundary_walls(w: f64, h: f64) -> Vec<Wall> {
    vec![
        Wall::new(0.0, 0.0, w, 0.0),   // top
        Wall::new(w, 0.0, w, h),       // right
        Wall::new(w, h, 0.0, h),       // bottom
        Wall::new(0.0, h, 0.0, 0.0),   // left
    ]
}

/// Ray-segment intersection.
/// Returns the distance along the ray (parameter t) where it hits the segment,
/// or None if it doesn't hit.
fn ray_segment_intersection(
    origin: (f64, f64),
    dir: (f64, f64),
    seg_a: (f64, f64),
    seg_b: (f64, f64),
) -> Option<f64> {
    let dx = dir.0;
    let dy = dir.1;
    let ex = seg_b.0 - seg_a.0;
    let ey = seg_b.1 - seg_a.1;

    let denom = dx * ey - dy * ex;
    if denom.abs() < 1e-10 {
        return None; // parallel
    }

    let fx = seg_a.0 - origin.0;
    let fy = seg_a.1 - origin.1;

    let t = (fx * ey - fy * ex) / denom; // distance along ray
    let u = (fx * dy - fy * dx) / denom; // parameter along segment [0,1]

    if t > 0.0 && u >= 0.0 && u <= 1.0 {
        Some(t)
    } else {
        None
    }
}

/// Compute the push vector to move a circle out of a line segment.
/// Returns None if the circle doesn't overlap the segment.
fn circle_segment_push(
    center: (f64, f64),
    radius: f64,
    seg_a: (f64, f64),
    seg_b: (f64, f64),
) -> Option<(f64, f64)> {
    // Find closest point on segment to circle center
    let ex = seg_b.0 - seg_a.0;
    let ey = seg_b.1 - seg_a.1;
    let len_sq = ex * ex + ey * ey;

    if len_sq < 1e-10 {
        return None; // degenerate segment
    }

    let t = ((center.0 - seg_a.0) * ex + (center.1 - seg_a.1) * ey) / len_sq;
    let t = t.clamp(0.0, 1.0);

    let closest_x = seg_a.0 + t * ex;
    let closest_y = seg_a.1 + t * ey;

    let dx = center.0 - closest_x;
    let dy = center.1 - closest_y;
    let dist_sq = dx * dx + dy * dy;

    if dist_sq >= radius * radius || dist_sq < 1e-10 {
        return None; // no overlap, or center is exactly on the segment
    }

    let dist = dist_sq.sqrt();
    let overlap = radius - dist;

    // Push along the direction from closest point to center
    Some((dx / dist * overlap, dy / dist * overlap))
}
