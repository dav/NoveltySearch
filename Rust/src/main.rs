mod evolution;
mod explainer;
mod maze;
mod network;
mod novelty;
mod robot;

use eframe::egui;
use evolution::Evolution;
use maze::Maze;
use network::Network;
use novelty::NoveltySearch;
use robot::Robot;

/// Which maze is currently selected.
#[derive(PartialEq, Clone, Copy)]
enum MazeChoice {
    Medium,
    Hard,
    Eller,
}

/// How the robot is being controlled.
#[derive(PartialEq, Clone, Copy)]
enum Mode {
    Manual,
    Auto,
    Evolve,
    Novelty,
}

struct App {
    maze_choice: MazeChoice,
    maze: Maze,
    robot: Robot,
    mode: Mode,
    network: Network,
    steps_per_frame: u32,
    step_count: u32,

    // Manual-mode inputs (held keys)
    key_forward: bool,
    key_back: bool,
    key_left: bool,
    key_right: bool,

    // Evolution state
    evolution: Evolution,
    evo_running: bool,
    evo_gens_per_frame: u32,
    // Replay of best robot
    replay_trajectory: Vec<(f64, f64)>,
    replay_index: usize,

    // Novelty search state
    novelty_search: NoveltySearch,
    novelty_running: bool,
    novelty_gens_per_frame: u32,
    novelty_replay_trajectory: Vec<(f64, f64)>,
    novelty_replay_index: usize,
    novelty_step_one: bool,

    // Last export status (path on success, error message on failure).
    explainer_export_message: Option<Result<std::path::PathBuf, String>>,
}

impl App {
    fn new() -> Self {
        let maze = Maze::medium();
        let robot = Robot::new(maze.start.0, maze.start.1);
        App {
            maze_choice: MazeChoice::Medium,
            maze,
            robot,
            mode: Mode::Manual,
            network: Network::random(),
            steps_per_frame: 1,
            step_count: 0,
            key_forward: false,
            key_back: false,
            key_left: false,
            key_right: false,
            evolution: Evolution::new(),
            evo_running: false,
            evo_gens_per_frame: 1,
            replay_trajectory: Vec::new(),
            replay_index: 0,
            novelty_search: NoveltySearch::new(),
            novelty_running: false,
            novelty_gens_per_frame: 1,
            novelty_replay_trajectory: Vec::new(),
            novelty_replay_index: 0,
            novelty_step_one: false,
            explainer_export_message: None,
        }
    }

    fn reset(&mut self) {
        self.robot = Robot::new(self.maze.start.0, self.maze.start.1);
        self.step_count = 0;
        self.replay_trajectory.clear();
        self.replay_index = 0;
    }

    fn reset_evolution(&mut self) {
        self.evolution = Evolution::new();
        self.evo_running = false;
        self.replay_trajectory.clear();
        self.replay_index = 0;
    }

    fn reset_novelty(&mut self) {
        self.novelty_search = NoveltySearch::new();
        self.novelty_running = false;
        self.novelty_step_one = false;
        self.novelty_replay_trajectory.clear();
        self.novelty_replay_index = 0;
        self.explainer_export_message = None;
    }

    fn switch_maze(&mut self, choice: MazeChoice) {
        self.maze_choice = choice;
        self.maze = match choice {
            MazeChoice::Medium => Maze::medium(),
            MazeChoice::Hard => Maze::hard(),
            MazeChoice::Eller => Maze::eller(),
        };
        self.reset();
        self.reset_evolution();
        self.reset_novelty();
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Process key events for manual mode
        ctx.input(|i| {
            for event in &i.events {
                if let egui::Event::Key { key, pressed, .. } = event {
                    match key {
                        egui::Key::W | egui::Key::ArrowUp => self.key_forward = *pressed,
                        egui::Key::S | egui::Key::ArrowDown => self.key_back = *pressed,
                        egui::Key::A | egui::Key::ArrowLeft => self.key_left = *pressed,
                        egui::Key::D | egui::Key::ArrowRight => self.key_right = *pressed,
                        _ => {}
                    }
                }
            }
        });

        // Simulation step(s)
        match self.mode {
            Mode::Manual | Mode::Auto => {
                for _ in 0..self.steps_per_frame {
                    match self.mode {
                        Mode::Manual => {
                            let ang_vel = if self.key_left { -0.1 } else if self.key_right { 0.1 } else { 0.0 };
                            let speed = if self.key_forward { 3.0 } else if self.key_back { -1.5 } else { 0.0 };
                            self.robot.step(ang_vel, speed, &self.maze);
                        }
                        Mode::Auto => {
                            let inputs = self.robot.sensor_inputs(&self.maze);
                            let (ang_vel, speed) = self.network.forward(&inputs);
                            self.robot.step(ang_vel, speed, &self.maze);
                        }
                        _ => {}
                    }
                    self.step_count += 1;
                }
            }
            Mode::Evolve => {
                if self.evo_running && !self.evolution.solved {
                    for _ in 0..self.evo_gens_per_frame {
                        self.evolution.step_generation(&self.maze);
                        if self.evolution.solved {
                            break;
                        }
                    }
                    // Update replay with best trajectory
                    self.replay_trajectory = self.evolution.best_trajectory.clone();
                    self.replay_index = self.replay_trajectory.len();
                }
                // Animate replay: advance the position along the best trajectory
                if !self.replay_trajectory.is_empty() && self.replay_index < self.replay_trajectory.len() {
                    let pos = self.replay_trajectory[self.replay_index];
                    self.robot.x = pos.0;
                    self.robot.y = pos.1;
                    self.replay_index += self.steps_per_frame as usize;
                }
            }
            Mode::Novelty => {
                if self.novelty_step_one && !self.novelty_search.solved {
                    self.novelty_search.step_generation(&self.maze);
                    self.novelty_step_one = false;
                    self.novelty_replay_trajectory = self.novelty_search.best_trajectory.clone();
                    self.novelty_replay_index = self.novelty_replay_trajectory.len();
                }
                if self.novelty_running && !self.novelty_search.solved {
                    for _ in 0..self.novelty_gens_per_frame {
                        self.novelty_search.step_generation(&self.maze);
                        if self.novelty_search.solved {
                            break;
                        }
                    }
                    self.novelty_replay_trajectory = self.novelty_search.best_trajectory.clone();
                    self.novelty_replay_index = self.novelty_replay_trajectory.len();
                }
                if !self.novelty_replay_trajectory.is_empty() && self.novelty_replay_index < self.novelty_replay_trajectory.len() {
                    let pos = self.novelty_replay_trajectory[self.novelty_replay_index];
                    self.robot.x = pos.0;
                    self.robot.y = pos.1;
                    self.novelty_replay_index += self.steps_per_frame as usize;
                }
            }
        }

        // Side panel with controls and stats
        egui::SidePanel::left("controls").min_width(180.0).show(ctx, |ui| {
            ui.heading("Controls");
            ui.separator();

            // Maze selection
            ui.label("Maze:");
            let prev_choice = self.maze_choice;
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.maze_choice, MazeChoice::Medium, "Medium");
                ui.selectable_value(&mut self.maze_choice, MazeChoice::Hard, "Hard");
                ui.selectable_value(&mut self.maze_choice, MazeChoice::Eller, "Eller");
            });
            if self.maze_choice != prev_choice {
                self.switch_maze(self.maze_choice);
            }
            if self.maze_choice == MazeChoice::Eller && ui.button("New maze").clicked() {
                self.switch_maze(MazeChoice::Eller);
            }

            ui.separator();

            // Mode selection
            ui.label("Mode:");
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.mode, Mode::Manual, "Manual");
                ui.selectable_value(&mut self.mode, Mode::Auto, "Auto");
                ui.selectable_value(&mut self.mode, Mode::Evolve, "Evolve");
                ui.selectable_value(&mut self.mode, Mode::Novelty, "Novelty");
            });

            if self.mode == Mode::Manual {
                ui.small("WASD or arrow keys to drive");
            }

            ui.separator();

            if self.mode == Mode::Evolve {
                // Evolution controls
                if self.evolution.solved {
                    ui.label("SOLVED!");
                }

                ui.horizontal(|ui| {
                    if self.evo_running {
                        if ui.button("Pause").clicked() {
                            self.evo_running = false;
                        }
                    } else if ui.button("Start").clicked() {
                        self.evo_running = true;
                    }
                    if ui.button("Reset").clicked() {
                        self.reset_evolution();
                        self.reset();
                    }
                });

                ui.label("Gens/frame:");
                ui.add(egui::Slider::new(&mut self.evo_gens_per_frame, 1..=50));

                if ui.button("Replay best").clicked() {
                    self.replay_index = 0;
                    self.robot = Robot::new(self.maze.start.0, self.maze.start.1);
                }

                ui.label("Replay speed:");
                ui.add(egui::Slider::new(&mut self.steps_per_frame, 1..=20));

                ui.separator();

                // Evolution stats
                ui.heading("Evolution");
                ui.label(format!("Generation: {}", self.evolution.generation));
                ui.label(format!("Evaluations: {}", self.evolution.total_evaluations));
                ui.label(format!("Best fitness: {:.1}", self.evolution.best_fitness));

                // Fitness plot
                if !self.evolution.best_fitness_history.is_empty() {
                    ui.separator();
                    ui.label("Fitness over generations:");

                    let history = &self.evolution.best_fitness_history;
                    let min_f = history.iter().cloned().fold(f64::INFINITY, f64::min) as f32;
                    let max_f = history.iter().cloned().fold(f64::NEG_INFINITY, f64::max) as f32;
                    let range = (max_f - min_f).max(1.0);

                    let plot_height = 100.0;
                    let plot_width = 160.0;
                    let (response, painter) =
                        ui.allocate_painter(egui::vec2(plot_width, plot_height), egui::Sense::hover());
                    let rect = response.rect;

                    // Background
                    painter.rect_filled(rect, 2.0, egui::Color32::from_rgb(30, 30, 30));

                    // Plot line
                    if history.len() >= 2 {
                        let points: Vec<egui::Pos2> = history
                            .iter()
                            .enumerate()
                            .map(|(i, &f)| {
                                let x = rect.left() + (i as f32 / (history.len() - 1) as f32) * rect.width();
                                let y = rect.bottom() - ((f as f32 - min_f) / range) * rect.height();
                                egui::pos2(x, y)
                            })
                            .collect();

                        for w in points.windows(2) {
                            painter.line_segment(
                                [w[0], w[1]],
                                egui::Stroke::new(1.5, egui::Color32::from_rgb(100, 200, 100)),
                            );
                        }
                    }
                }
            } else if self.mode == Mode::Novelty {
                // Novelty search controls
                if self.novelty_search.solved {
                    ui.label("SOLVED!");
                }

                ui.horizontal(|ui| {
                    if self.novelty_running {
                        if ui.button("Pause").clicked() {
                            self.novelty_running = false;
                        }
                    } else {
                        if ui.button("Start").clicked() {
                            self.novelty_running = true;
                        }
                        if ui.button("Step 1 gen").clicked() {
                            self.novelty_step_one = true;
                        }
                    }
                    if ui.button("Reset").clicked() {
                        self.reset_novelty();
                        self.reset();
                    }
                });

                ui.label("Gens/frame:");
                ui.add(egui::Slider::new(&mut self.novelty_gens_per_frame, 1..=50));

                if ui.button("Replay closest").clicked() {
                    self.novelty_replay_index = 0;
                    self.robot = Robot::new(self.maze.start.0, self.maze.start.1);
                }

                ui.label("Replay speed:");
                ui.add(egui::Slider::new(&mut self.steps_per_frame, 1..=20));

                ui.separator();

                // Novelty stats
                ui.heading("Novelty Search");
                ui.label(format!("Generation: {}", self.novelty_search.generation));
                ui.label(format!("Evaluations: {}", self.novelty_search.total_evaluations));
                ui.label(format!("Archive size: {}", self.novelty_search.archive.len()));
                ui.label(format!("Closest to goal: {:.1}", self.novelty_search.closest_distance));
                ui.label(format!("Threshold (\u{03c1}_min): {:.2}", self.novelty_search.rho_min));

                if self.novelty_search.generation > 0 {
                    ui.separator();
                    ui.heading("Last Generation");
                    ui.label(format!(
                        "Archived: +{} (total: {})",
                        self.novelty_search.last_gen_archive_additions,
                        self.novelty_search.archive.len()
                    ));
                    ui.label(format!(
                        "Closest to goal: {:.1}",
                        self.novelty_search.last_gen_closest_dist
                    ));
                }

                // Explainer export
                ui.separator();
                ui.heading("Explainer export");
                let have_data = self.novelty_search.generation > 0;
                ui.add_enabled_ui(have_data, |ui| {
                    if ui.button("Export run to JSON").clicked() {
                        let path = explainer::default_export_path();
                        let result = explainer::write_export(
                            &self.novelty_search,
                            &self.maze,
                            &path,
                        );
                        self.explainer_export_message = Some(match result {
                            Ok(()) => {
                                let abs = std::fs::canonicalize(&path).unwrap_or(path);
                                Ok(abs)
                            }
                            Err(e) => Err(e.to_string()),
                        });
                    }
                });
                if !have_data {
                    ui.small("Run at least one generation first.");
                }
                match &self.explainer_export_message {
                    Some(Ok(path)) => {
                        ui.small(format!("Wrote: {}", path.display()));
                    }
                    Some(Err(e)) => {
                        ui.colored_label(egui::Color32::RED, format!("Export failed: {}", e));
                    }
                    None => {}
                }

                // Novelty score plot
                if !self.novelty_search.best_novelty_history.is_empty() {
                    ui.separator();
                    ui.label("Best novelty / gen:");

                    let history = &self.novelty_search.best_novelty_history;
                    let min_f = history.iter().cloned().fold(f64::INFINITY, f64::min) as f32;
                    let max_f = history.iter().cloned().fold(f64::NEG_INFINITY, f64::max) as f32;
                    let range = (max_f - min_f).max(1.0);

                    let plot_height = 100.0;
                    let plot_width = 160.0;
                    let (response, painter) =
                        ui.allocate_painter(egui::vec2(plot_width, plot_height), egui::Sense::hover());
                    let rect = response.rect;

                    painter.rect_filled(rect, 2.0, egui::Color32::from_rgb(30, 30, 30));

                    if history.len() >= 2 {
                        let points: Vec<egui::Pos2> = history
                            .iter()
                            .enumerate()
                            .map(|(i, &f)| {
                                let x = rect.left() + (i as f32 / (history.len() - 1) as f32) * rect.width();
                                let y = rect.bottom() - ((f as f32 - min_f) / range) * rect.height();
                                egui::pos2(x, y)
                            })
                            .collect();

                        for w in points.windows(2) {
                            painter.line_segment(
                                [w[0], w[1]],
                                egui::Stroke::new(1.5, egui::Color32::from_rgb(200, 100, 255)),
                            );
                        }
                    }
                }
            } else {
                // Non-evolution controls
                ui.label("Steps/frame:");
                ui.add(egui::Slider::new(&mut self.steps_per_frame, 1..=20));

                ui.separator();

                if ui.button("Reset robot").clicked() {
                    self.reset();
                }
                if self.mode == Mode::Auto && ui.button("New random network").clicked() {
                    self.network = Network::random();
                    self.reset();
                }

                ui.separator();

                // Stats
                ui.heading("Stats");
                ui.label(format!("Step: {}", self.step_count));
                ui.label(format!("Position: ({:.1}, {:.1})", self.robot.x, self.robot.y));
                ui.label(format!("Heading: {:.1}°", self.robot.heading.to_degrees()));
                ui.label(format!(
                    "Goal dist: {:.1}",
                    self.robot.distance_to_goal(self.maze.goal)
                ));

                ui.separator();
                ui.label("Rangefinders:");
                let rf = self.robot.rangefinders(&self.maze);
                let labels = ["-90°", "-45°", "0°", "45°", "90°", "180°"];
                for (label, val) in labels.iter().zip(rf.iter()) {
                    ui.label(format!("  {}: {:.2}", label, val));
                }

                ui.separator();
                ui.label("Radar:");
                let rd = self.robot.radar(self.maze.goal);
                let quad_labels = ["Front", "Right", "Back", "Left"];
                for (label, val) in quad_labels.iter().zip(rd.iter()) {
                    ui.label(format!("  {}: {:.0}", label, val));
                }
            }
        });

        // Main canvas
        egui::CentralPanel::default().show(ctx, |ui| {
            let available = ui.available_size();
            let (response, painter) =
                ui.allocate_painter(available, egui::Sense::hover());
            let rect = response.rect;

            // Compute transform from maze coords to screen coords
            let maze_w = self.maze.bounds.0 as f32;
            let maze_h = self.maze.bounds.1 as f32;
            let scale = (rect.width() / maze_w).min(rect.height() / maze_h) * 0.95;
            let offset_x = rect.left() + (rect.width() - maze_w * scale) / 2.0;
            let offset_y = rect.top() + (rect.height() - maze_h * scale) / 2.0;

            let to_screen = |x: f64, y: f64| -> egui::Pos2 {
                egui::pos2(
                    offset_x + x as f32 * scale,
                    offset_y + y as f32 * scale,
                )
            };

            // Draw maze background
            painter.rect_filled(
                egui::Rect::from_min_max(to_screen(0.0, 0.0), to_screen(self.maze.bounds.0, self.maze.bounds.1)),
                0.0,
                egui::Color32::from_rgb(240, 240, 240),
            );

            // Draw walls
            for wall in &self.maze.walls {
                painter.line_segment(
                    [to_screen(wall.a.0, wall.a.1), to_screen(wall.b.0, wall.b.1)],
                    egui::Stroke::new(2.0, egui::Color32::from_rgb(40, 40, 40)),
                );
            }

            // Draw goal
            let goal_screen = to_screen(self.maze.goal.0, self.maze.goal.1);
            painter.circle_filled(goal_screen, 8.0, egui::Color32::from_rgb(50, 200, 50));
            painter.circle_stroke(goal_screen, 8.0, egui::Stroke::new(1.5, egui::Color32::from_rgb(20, 120, 20)));

            // Draw start marker
            let start_screen = to_screen(self.maze.start.0, self.maze.start.1);
            painter.circle_stroke(start_screen, 6.0, egui::Stroke::new(1.5, egui::Color32::from_rgb(100, 100, 200)));

            // Draw trajectory trail (in Evolve mode)
            if self.mode == Mode::Evolve && !self.replay_trajectory.is_empty() {
                let end = self.replay_index.min(self.replay_trajectory.len());
                for i in 1..end {
                    let prev = self.replay_trajectory[i - 1];
                    let curr = self.replay_trajectory[i];
                    painter.line_segment(
                        [to_screen(prev.0, prev.1), to_screen(curr.0, curr.1)],
                        egui::Stroke::new(1.5, egui::Color32::from_rgba_premultiplied(60, 120, 220, 120)),
                    );
                }
            }

            // Draw final positions scatter (in Evolve mode)
            if self.mode == Mode::Evolve {
                for &pos in &self.evolution.all_final_positions {
                    let p = to_screen(pos.0, pos.1);
                    painter.circle_filled(p, 2.0, egui::Color32::from_rgba_premultiplied(200, 100, 50, 150));
                }
            }

            // Draw novelty search data
            if self.mode == Mode::Novelty {
                // Draw archive points colored by age (faded = old, vivid = new)
                let max_gen = self.novelty_search.generation.max(1) as f32;
                for &(pos, added_gen) in &self.novelty_search.archive {
                    let t = added_gen as f32 / max_gen;
                    let r = (180.0 - 20.0 * t) as u8;
                    let g = (140.0 - 90.0 * t) as u8;
                    let b = (200.0 + 55.0 * t) as u8;
                    let alpha = (80.0 + 140.0 * t) as u8;
                    let p = to_screen(pos.0, pos.1);
                    painter.circle_filled(p, 2.5, egui::Color32::from_rgba_premultiplied(r, g, b, alpha));
                }

                // Draw current generation final positions colored by novelty score
                if !self.novelty_search.all_final_positions.is_empty() {
                    let scores = &self.novelty_search.novelty_scores;
                    let positions = &self.novelty_search.all_final_positions;
                    let min_s = scores.iter().cloned().fold(f64::INFINITY, f64::min);
                    let max_s = scores.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                    let range = (max_s - min_s).max(1e-10);

                    for (pos, &score) in positions.iter().zip(scores.iter()) {
                        let t = ((score - min_s) / range) as f32;
                        let r = (120.0 + 135.0 * t) as u8;
                        let g = (70.0 + 150.0 * t) as u8;
                        let b = (30.0 + 20.0 * t) as u8;
                        let alpha = (100.0 + 130.0 * t) as u8;
                        let size = 1.5 + 3.5 * t;
                        let p = to_screen(pos.0, pos.1);
                        painter.circle_filled(p, size, egui::Color32::from_rgba_premultiplied(r, g, b, alpha));
                    }
                }

                // Draw trajectory trail for closest-to-goal robot
                if !self.novelty_replay_trajectory.is_empty() {
                    let end = self.novelty_replay_index.min(self.novelty_replay_trajectory.len());
                    for i in 1..end {
                        let prev = self.novelty_replay_trajectory[i - 1];
                        let curr = self.novelty_replay_trajectory[i];
                        painter.line_segment(
                            [to_screen(prev.0, prev.1), to_screen(curr.0, curr.1)],
                            egui::Stroke::new(1.5, egui::Color32::from_rgba_premultiplied(60, 200, 120, 150)),
                        );
                    }
                }
            }

            // Draw rangefinder rays
            let endpoints = self.robot.rangefinder_endpoints(&self.maze);
            let robot_screen = to_screen(self.robot.x, self.robot.y);
            for ep in &endpoints {
                painter.line_segment(
                    [robot_screen, to_screen(ep.0, ep.1)],
                    egui::Stroke::new(2.0, egui::Color32::from_rgb(255, 20, 147)),
                );
            }

            // Draw robot body
            painter.circle_filled(robot_screen, self.robot.radius as f32 * scale, egui::Color32::from_rgb(60, 120, 220));
            painter.circle_stroke(
                robot_screen,
                self.robot.radius as f32 * scale,
                egui::Stroke::new(1.5, egui::Color32::from_rgb(30, 60, 140)),
            );

            // Draw heading arrow
            let arrow_len = self.robot.radius * 1.8;
            let arrow_end = to_screen(
                self.robot.x + self.robot.heading.cos() * arrow_len,
                self.robot.y + self.robot.heading.sin() * arrow_len,
            );
            painter.line_segment(
                [robot_screen, arrow_end],
                egui::Stroke::new(2.0, egui::Color32::WHITE),
            );
        });

        // Request continuous repainting so animation runs smoothly
        ctx.request_repaint();
    }
}

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([900.0, 600.0])
            .with_title("Lehman — Maze Navigation"),
        ..Default::default()
    };

    eframe::run_native(
        "Novelty Search",
        options,
        Box::new(|_cc| Ok(Box::new(App::new()))),
    )
}
