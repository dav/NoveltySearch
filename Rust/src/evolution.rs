use rand::Rng;

use crate::maze::Maze;
use crate::network::Network;
use crate::robot::Robot;

const POPULATION_SIZE: usize = 150;
const TIMESTEPS: u32 = 400;
const MUTATION_SIGMA: f64 = 0.1;
const TOURNAMENT_SIZE: usize = 3;
const SUCCESS_DISTANCE: f64 = 5.0;

/// Result of evaluating a single network on a maze.
struct EvalResult {
    fitness: f64,
    final_pos: (f64, f64),
    trajectory: Vec<(f64, f64)>,
}

/// The evolutionary search state.
pub struct Evolution {
    population: Vec<Network>,
    fitnesses: Vec<f64>,
    pub generation: u32,
    pub total_evaluations: u32,
    pub best_fitness_history: Vec<f64>,
    pub best_trajectory: Vec<(f64, f64)>,
    pub best_network: Network,
    pub best_fitness: f64,
    pub solved: bool,
    pub all_final_positions: Vec<(f64, f64)>,
}

impl Evolution {
    pub fn new() -> Self {
        let population: Vec<Network> = (0..POPULATION_SIZE).map(|_| Network::random()).collect();
        let best_network = population[0].clone();

        Evolution {
            population,
            fitnesses: vec![0.0; POPULATION_SIZE],
            generation: 0,
            total_evaluations: 0,
            best_fitness_history: Vec::new(),
            best_trajectory: Vec::new(),
            best_network,
            best_fitness: f64::NEG_INFINITY,
            solved: false,
            all_final_positions: Vec::new(),
        }
    }

    /// Run one generation of evolution. Returns true if the goal was reached.
    pub fn step_generation(&mut self, maze: &Maze) -> bool {
        // Evaluate all individuals
        let mut best_idx = 0;
        let mut gen_best_fitness = f64::NEG_INFINITY;
        let mut best_result: Option<EvalResult> = None;

        self.all_final_positions.clear();

        for i in 0..POPULATION_SIZE {
            let result = evaluate(&self.population[i], maze);
            self.fitnesses[i] = result.fitness;
            self.all_final_positions.push(result.final_pos);

            if result.fitness > gen_best_fitness {
                gen_best_fitness = result.fitness;
                best_idx = i;
                best_result = Some(result);
            }
        }

        self.total_evaluations += POPULATION_SIZE as u32;
        self.generation += 1;

        // Update best-ever
        if gen_best_fitness > self.best_fitness {
            self.best_fitness = gen_best_fitness;
            self.best_network = self.population[best_idx].clone();
            if let Some(result) = best_result {
                self.best_trajectory = result.trajectory;
            }
        }

        self.best_fitness_history.push(gen_best_fitness);

        // Check if solved
        if gen_best_fitness >= 0.0 {
            // fitness >= 0 means final_distance <= SUCCESS_DISTANCE
            // (see evaluate function below)
            self.solved = true;
            return true;
        }

        // Selection and reproduction
        let mut new_population = Vec::with_capacity(POPULATION_SIZE);

        // Elitism: keep the best individual unchanged
        new_population.push(self.population[best_idx].clone());

        // Fill the rest with tournament selection + mutation
        let mut rng = rand::rng();
        while new_population.len() < POPULATION_SIZE {
            let winner = self.tournament_select(&mut rng);
            new_population.push(self.population[winner].mutated(MUTATION_SIGMA));
        }

        self.population = new_population;
        false
    }

    /// Tournament selection: pick TOURNAMENT_SIZE random individuals, return the index of the best.
    fn tournament_select(&self, rng: &mut impl Rng) -> usize {
        let mut best_idx = rng.random_range(0..POPULATION_SIZE);
        let mut best_fit = self.fitnesses[best_idx];

        for _ in 1..TOURNAMENT_SIZE {
            let idx = rng.random_range(0..POPULATION_SIZE);
            if self.fitnesses[idx] > best_fit {
                best_fit = self.fitnesses[idx];
                best_idx = idx;
            }
        }

        best_idx
    }
}

/// Evaluate a network by running a robot for TIMESTEPS steps.
/// Fitness = -(final_distance_to_goal - SUCCESS_DISTANCE)
/// So fitness = 0 means exactly at the success threshold, positive means solved.
fn evaluate(network: &Network, maze: &Maze) -> EvalResult {
    let mut robot = Robot::new(maze.start.0, maze.start.1);
    let mut trajectory = Vec::with_capacity(TIMESTEPS as usize);

    for _ in 0..TIMESTEPS {
        let inputs = robot.sensor_inputs(maze);
        let (ang_vel, speed) = network.forward(&inputs);
        robot.step(ang_vel, speed, maze);
        trajectory.push((robot.x, robot.y));
    }

    let final_dist = robot.distance_to_goal(maze.goal);
    let fitness = -(final_dist - SUCCESS_DISTANCE);

    EvalResult {
        fitness,
        final_pos: (robot.x, robot.y),
        trajectory,
    }
}
