//! Data model + JSON export for the pedagogical explainer run.
//!
//! A `Recording` is accumulated during a novelty search run (one
//! `GenerationSnapshot` per generation). When the user clicks "Export run",
//! `write_export` serializes the recording plus the maze, config, archive, and
//! winner details (including a re-run of the winner with per-step activations
//! captured) into a single JSON file.

use std::io;
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

use serde::Serialize;

use crate::maze::Maze;
use crate::network::{ForwardTrace, Network};
use crate::novelty::{
    NoveltySearch, INITIAL_RHO_MIN, K_NEAREST, MUTATION_SIGMA, POPULATION_SIZE, SUCCESS_DISTANCE,
    TIMESTEPS, TOURNAMENT_SIZE,
};
use crate::robot::Robot;

/// One individual's state in one generation.
#[derive(Clone, Serialize)]
pub struct IndividualSnapshot {
    pub weights: Network,
    pub trajectory: Vec<(f64, f64)>,
    pub final_position: (f64, f64),
    pub novelty_score: f64,
    pub archived: bool,
}

/// Full record of one generation.
#[derive(Clone, Serialize)]
pub struct GenerationSnapshot {
    pub generation: u32,
    pub rho_min_before: f64,
    pub rho_min_after: f64,
    pub archive_additions: u32,
    pub best_novelty: f64,
    /// Index of the elite individual in THIS generation. That individual is
    /// copied unmutated into the next generation as index 0.
    pub elite_idx: usize,
    /// Closest-to-goal individual in this generation.
    pub closest_idx: usize,
    pub closest_distance: f64,
    pub individuals: Vec<IndividualSnapshot>,
    /// Tournament winners (parent indices in THIS generation) for each non-elite
    /// child of the NEXT generation. Length = POPULATION_SIZE - 1. So the
    /// next generation's individual at index `i` (for `i >= 1`) was produced
    /// by mutating `individuals[tournament_winners[i - 1]]`. Next gen's index 0
    /// is the unmutated elite.
    pub tournament_winners: Vec<usize>,
}

/// Full recording of a novelty search run.
#[derive(Default, Clone, Serialize)]
pub struct Recording {
    pub generations: Vec<GenerationSnapshot>,
}

/// Algorithm constants, shipped with the export so the explainer has them verbatim.
#[derive(Serialize)]
pub struct Config {
    pub population_size: usize,
    pub timesteps: u32,
    pub mutation_sigma: f64,
    pub tournament_size: usize,
    pub success_distance: f64,
    pub k_nearest: usize,
    pub initial_rho_min: f64,
}

/// One ancestor in the winner's lineage.
#[derive(Serialize)]
pub struct LineageEntry {
    pub generation: u32,
    pub idx: usize,
    pub novelty_score: f64,
    pub weights: Network,
    /// True iff this entry is the (unmutated) elite carried over from the
    /// previous generation. Useful for the explainer to know whether weights
    /// actually changed from the prior step of the lineage.
    pub from_elite: bool,
}

#[derive(Serialize)]
pub struct ArchiveEntry {
    pub position: (f64, f64),
    pub generation_added: u32,
}

/// Full detail of the individual considered the "winner" of the run.
/// If the run solved the maze, this is the solving individual. Otherwise it is
/// the closest-ever individual (useful for exporting an aborted-but-interesting run).
#[derive(Serialize)]
pub struct WinnerDetails {
    pub generation: u32,
    pub individual_idx: usize,
    pub solved: bool,
    pub closest_distance: f64,
    pub weights: Network,
    /// Per-step forward pass trace (inputs, hidden activations, outputs, derived
    /// angular velocity and speed). Length = TIMESTEPS. Recomputed deterministically
    /// from the weights and maze at export time.
    pub activations_per_step: Vec<ForwardTrace>,
    /// Ancestry from generation 1 (first entry) to the winning generation (last entry).
    pub lineage: Vec<LineageEntry>,
}

#[derive(Serialize)]
pub struct ExportMeta {
    pub version: u32,
    pub exported_at_unix: u64,
}

#[derive(Serialize)]
pub struct ExportBundle<'a> {
    pub meta: ExportMeta,
    pub config: Config,
    pub maze: &'a Maze,
    pub generations: &'a [GenerationSnapshot],
    pub archive: Vec<ArchiveEntry>,
    pub best_novelty_history: &'a [f64],
    pub winner: Option<WinnerDetails>,
}

/// Build the full export bundle from the live NoveltySearch state plus the maze.
/// Returns `None` for the winner if no generations have run yet.
pub fn build_export<'a>(ns: &'a NoveltySearch, maze: &'a Maze) -> ExportBundle<'a> {
    let exported_at_unix = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);

    let config = Config {
        population_size: POPULATION_SIZE,
        timesteps: TIMESTEPS,
        mutation_sigma: MUTATION_SIGMA,
        tournament_size: TOURNAMENT_SIZE,
        success_distance: SUCCESS_DISTANCE,
        k_nearest: K_NEAREST,
        initial_rho_min: INITIAL_RHO_MIN,
    };

    let archive: Vec<ArchiveEntry> = ns
        .archive
        .iter()
        .map(|&(position, generation_added)| ArchiveEntry {
            position,
            generation_added,
        })
        .collect();

    let winner = build_winner(ns, maze);

    ExportBundle {
        meta: ExportMeta {
            version: 1,
            exported_at_unix,
        },
        config,
        maze,
        generations: &ns.recording.generations,
        archive,
        best_novelty_history: &ns.best_novelty_history,
        winner,
    }
}

fn build_winner(ns: &NoveltySearch, maze: &Maze) -> Option<WinnerDetails> {
    if ns.recording.generations.is_empty() {
        return None;
    }

    let generation = ns.closest_generation;
    let individual_idx = ns.closest_individual_idx;
    if generation == 0 {
        return None;
    }

    let snap = ns.recording.generations.get(generation as usize - 1)?;
    let individual = snap.individuals.get(individual_idx)?;
    let weights = individual.weights.clone();
    let activations_per_step = run_with_activations(&weights, maze, TIMESTEPS);
    let lineage = build_lineage(&ns.recording.generations, generation, individual_idx);

    Some(WinnerDetails {
        generation,
        individual_idx,
        solved: ns.solved,
        closest_distance: ns.closest_distance,
        weights,
        activations_per_step,
        lineage,
    })
}

/// Walk backwards from (generation, idx) through the recording, using each
/// generation's `elite_idx` and `tournament_winners` to find the parent of the
/// next-generation individual we arrived from. Returns the chain ordered
/// gen 1 → winner.
fn build_lineage(
    generations: &[GenerationSnapshot],
    start_gen: u32,
    start_idx: usize,
) -> Vec<LineageEntry> {
    let mut lineage = Vec::new();
    let mut cur_gen = start_gen as usize;
    let mut cur_idx = start_idx;
    let mut came_from_elite = false;

    loop {
        let Some(snap) = generations.get(cur_gen - 1) else { break };
        let Some(individual) = snap.individuals.get(cur_idx) else { break };

        lineage.push(LineageEntry {
            generation: cur_gen as u32,
            idx: cur_idx,
            novelty_score: individual.novelty_score,
            weights: individual.weights.clone(),
            from_elite: came_from_elite,
        });

        if cur_gen == 1 {
            break;
        }

        // Look at the PREVIOUS generation's selection to determine our parent.
        let Some(prev_snap) = generations.get(cur_gen - 2) else { break };
        let (parent_idx, from_elite) = if cur_idx == 0 {
            (prev_snap.elite_idx, true)
        } else {
            match prev_snap.tournament_winners.get(cur_idx - 1).copied() {
                Some(p) => (p, false),
                None => break,
            }
        };

        came_from_elite = from_elite;
        cur_gen -= 1;
        cur_idx = parent_idx;
    }

    lineage.reverse();
    lineage
}

/// Replay a network on the maze for `timesteps` steps, capturing every
/// forward-pass trace. Deterministic given the weights and maze.
fn run_with_activations(network: &Network, maze: &Maze, timesteps: u32) -> Vec<ForwardTrace> {
    let mut robot = Robot::new(maze.start.0, maze.start.1);
    let mut traces = Vec::with_capacity(timesteps as usize);

    for _ in 0..timesteps {
        let inputs = robot.sensor_inputs(maze);
        let trace = network.forward_with_activations(&inputs);
        robot.step(trace.ang_vel, trace.speed, maze);
        traces.push(trace);
    }

    traces
}

/// Serialize the run to pretty JSON at the given path.
pub fn write_export(ns: &NoveltySearch, maze: &Maze, path: &Path) -> io::Result<()> {
    let bundle = build_export(ns, maze);
    let json = serde_json::to_string_pretty(&bundle)
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    std::fs::write(path, json)
}

/// Suggest a filename of the form `explainer-run-<unix>.json` in the current
/// working directory.
pub fn default_export_path() -> std::path::PathBuf {
    let ts = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    std::path::PathBuf::from(format!("explainer-run-{}.json", ts))
}

