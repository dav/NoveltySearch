# Novelty Search

A Rust implementation of novelty search applied to maze navigation, inspired by Joel Lehman's work on abandoning objectives in evolutionary algorithms.

A robot equipped with rangefinder sensors navigates mazes using a small feedforward neural network (11 inputs, 5 hidden, 2 outputs). The app provides an interactive [egui](https://github.com/emilk/egui) visualization with four control modes:

- **Manual** -- drive the robot with keyboard controls
- **Auto** -- run a single neural network controller forward
- **Evolve** -- evolve controllers using objective-based (fitness) evolution toward the maze goal
- **Novelty** -- evolve controllers using novelty search, which rewards behavioral novelty rather than proximity to the goal

Three maze environments are included: Medium, Hard, and a procedurally generated Eller maze.

## Paper

Lehman, J. and Stanley, K.O. (2011). [Abandoning Objectives: Evolution Through the Search for Novelty Alone](https://www.cs.swarthmore.edu/~meeden/DevelopmentalRobotics/lehman_ecj11.pdf). *Evolutionary Computation*, 19(2), 189-223.

## Getting Started

### Prerequisites

- [Rust](https://www.rust-lang.org/tools/install) (edition 2024)

### Clone and Run

```bash
git clone <repo-url>
cd NoveltySearch/novelty-search
cargo run
```
