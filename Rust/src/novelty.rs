use rand::Rng;

use crate::explainer::{GenerationSnapshot, IndividualSnapshot, Recording};
use crate::maze::Maze;
use crate::network::Network;
use crate::robot::Robot;

pub const POPULATION_SIZE: usize = 150;
pub const TIMESTEPS: u32 = 400;
pub const MUTATION_SIGMA: f64 = 0.1;
pub const TOURNAMENT_SIZE: usize = 3;
pub const SUCCESS_DISTANCE: f64 = 5.0;
pub const K_NEAREST: usize = 15;
pub const INITIAL_RHO_MIN: f64 = 6.0;

/// The novelty search state.
pub struct NoveltySearch {
    population: Vec<Network>,
    pub novelty_scores: Vec<f64>,
    pub generation: u32,
    pub total_evaluations: u32,

    // Behavior archive — stores final (x, y) positions + generation added
    pub archive: Vec<((f64, f64), u32)>,
    pub rho_min: f64,
    evals_since_last_addition: u32,

    // Per-generation data for visualization
    pub all_final_positions: Vec<(f64, f64)>,
    pub best_novelty_history: Vec<f64>,

    // Best individual that reached closest to goal (tracked but not used for selection)
    pub best_trajectory: Vec<(f64, f64)>,
    pub closest_distance: f64,
    pub closest_generation: u32,
    pub closest_individual_idx: usize,
    pub solved: bool,

    // Last generation summary (for UI)
    pub last_gen_archive_additions: u32,
    pub last_gen_closest_dist: f64,

    // Full per-generation recording for the explainer export.
    pub recording: Recording,
}

impl NoveltySearch {
    pub fn new() -> Self {
        let population: Vec<Network> = (0..POPULATION_SIZE).map(|_| Network::random()).collect();

        NoveltySearch {
            population,
            novelty_scores: vec![0.0; POPULATION_SIZE],
            generation: 0,
            total_evaluations: 0,
            archive: Vec::new(),
            rho_min: INITIAL_RHO_MIN,
            evals_since_last_addition: 0,
            all_final_positions: Vec::new(),
            best_novelty_history: Vec::new(),
            best_trajectory: Vec::new(),
            closest_distance: f64::INFINITY,
            closest_generation: 0,
            closest_individual_idx: 0,
            solved: false,
            last_gen_archive_additions: 0,
            last_gen_closest_dist: f64::INFINITY,
            recording: Recording::default(),
        }
    }

    /// Run one generation of novelty search. Returns true if any robot reached the goal.
    pub fn step_generation(&mut self, maze: &Maze) -> bool {
        let rho_min_before = self.rho_min;

        // Snapshot the current population's weights BEFORE selection/mutation replaces them.
        let population_weights: Vec<Network> = self.population.clone();

        // Evaluate all individuals — run each robot, record final position
        let mut final_positions = Vec::with_capacity(POPULATION_SIZE);
        let mut trajectories = Vec::with_capacity(POPULATION_SIZE);

        for network in &self.population {
            let (final_pos, trajectory) = evaluate(network, maze);
            final_positions.push(final_pos);
            trajectories.push(trajectory);
        }

        self.all_final_positions = final_positions.clone();
        self.total_evaluations += POPULATION_SIZE as u32;
        self.generation += 1;

        // Compute novelty score for each individual
        // Novelty = average distance to k-nearest neighbors in (population + archive)
        let mut best_novelty = f64::NEG_INFINITY;
        let mut additions_this_gen = 0u32;
        let mut closest_idx = 0;
        let mut closest_dist = f64::INFINITY;
        let mut archived_mask = vec![false; POPULATION_SIZE];

        for i in 0..POPULATION_SIZE {
            let novelty = self.compute_novelty(final_positions[i], &final_positions);
            self.novelty_scores[i] = novelty;

            if novelty > best_novelty {
                best_novelty = novelty;
            }

            // Check if this individual should be added to the archive
            if novelty > self.rho_min {
                self.archive.push((final_positions[i], self.generation));
                additions_this_gen += 1;
                archived_mask[i] = true;
            }

            // Track closest to goal (not used for selection, just for reporting)
            let dist = distance(final_positions[i], maze.goal);
            if dist < closest_dist {
                closest_dist = dist;
                closest_idx = i;
            }
        }

        // Store generation summary for UI
        self.last_gen_archive_additions = additions_this_gen;
        self.last_gen_closest_dist = closest_dist;

        // Update best-ever closest to goal
        if closest_dist < self.closest_distance {
            self.closest_distance = closest_dist;
            self.closest_generation = self.generation;
            self.closest_individual_idx = closest_idx;
            self.best_trajectory = trajectories[closest_idx].clone();
        }

        let solved_now = closest_dist <= SUCCESS_DISTANCE;
        if solved_now {
            self.solved = true;
            self.closest_generation = self.generation;
            self.closest_individual_idx = closest_idx;
            self.best_trajectory = trajectories[closest_idx].clone();
        }

        self.best_novelty_history.push(best_novelty);

        // Adaptive threshold for archive
        if additions_this_gen == 0 {
            self.evals_since_last_addition += POPULATION_SIZE as u32;
            if self.evals_since_last_addition >= 2500 {
                self.rho_min *= 0.95; // lower threshold by 5%
                self.evals_since_last_addition = 0;
            }
        } else {
            self.evals_since_last_addition = 0;
            if additions_this_gen > 4 {
                self.rho_min *= 1.20; // raise threshold by 20%
            }
        }

        // Elitism: keep the individual with the highest novelty
        let elite_idx = self.novelty_scores
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(i, _)| i)
            .unwrap_or(0);

        // Selection and reproduction (same GA, but using novelty scores).
        // We still run selection on the solved generation so the recording is uniform;
        // the new population is simply unused once `solved` is true.
        let mut new_population = Vec::with_capacity(POPULATION_SIZE);
        new_population.push(self.population[elite_idx].clone());

        let mut tournament_winners = Vec::with_capacity(POPULATION_SIZE - 1);
        let mut rng = rand::rng();
        while new_population.len() < POPULATION_SIZE {
            let winner = self.tournament_select(&mut rng);
            tournament_winners.push(winner);
            new_population.push(self.population[winner].mutated(MUTATION_SIGMA));
        }

        // Record this generation for the explainer export.
        let individuals: Vec<IndividualSnapshot> = (0..POPULATION_SIZE)
            .map(|i| IndividualSnapshot {
                weights: population_weights[i].clone(),
                trajectory: trajectories[i].clone(),
                final_position: final_positions[i],
                novelty_score: self.novelty_scores[i],
                archived: archived_mask[i],
            })
            .collect();

        self.recording.generations.push(GenerationSnapshot {
            generation: self.generation,
            rho_min_before,
            rho_min_after: self.rho_min,
            archive_additions: additions_this_gen,
            best_novelty,
            elite_idx,
            closest_idx,
            closest_distance: closest_dist,
            individuals,
            tournament_winners,
        });

        self.population = new_population;
        solved_now
    }

    /// Compute novelty of a behavior point: average distance to k-nearest neighbors
    /// in the combined set of current population final positions + archive.
    fn compute_novelty(&self, point: (f64, f64), population_behaviors: &[(f64, f64)]) -> f64 {
        // Collect distances to all other points (population + archive)
        let mut distances: Vec<f64> = Vec::new();

        for &other in population_behaviors {
            let d = distance(point, other);
            if d > 1e-10 {
                // skip self (distance ≈ 0)
                distances.push(d);
            }
        }

        for &(archived, _) in &self.archive {
            distances.push(distance(point, archived));
        }

        // Sort and take k-nearest
        distances.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let k = K_NEAREST.min(distances.len());

        if k == 0 {
            return 0.0;
        }

        distances[..k].iter().sum::<f64>() / k as f64
    }

    /// Tournament selection using novelty scores.
    fn tournament_select(&self, rng: &mut impl Rng) -> usize {
        let mut best_idx = rng.random_range(0..POPULATION_SIZE);
        let mut best_score = self.novelty_scores[best_idx];

        for _ in 1..TOURNAMENT_SIZE {
            let idx = rng.random_range(0..POPULATION_SIZE);
            if self.novelty_scores[idx] > best_score {
                best_score = self.novelty_scores[idx];
                best_idx = idx;
            }
        }

        best_idx
    }
}

/// Run a network on the maze for TIMESTEPS steps.
/// Returns (final_position, full_trajectory).
fn evaluate(network: &Network, maze: &Maze) -> ((f64, f64), Vec<(f64, f64)>) {
    let mut robot = Robot::new(maze.start.0, maze.start.1);
    let mut trajectory = Vec::with_capacity(TIMESTEPS as usize);

    for _ in 0..TIMESTEPS {
        let inputs = robot.sensor_inputs(maze);
        let (ang_vel, speed) = network.forward(&inputs);
        robot.step(ang_vel, speed, maze);
        trajectory.push((robot.x, robot.y));
    }

    ((robot.x, robot.y), trajectory)
}

fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    (dx * dx + dy * dy).sqrt()
}
