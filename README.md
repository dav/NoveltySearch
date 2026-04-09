# Novelty Search

Rust and Swift implementations of novelty search applied to maze navigation, inspired by Joel Lehman's paper on abandoning objectives in evolutionary algorithms.

A robot equipped with rangefinder sensors navigates mazes using a small feedforward neural network (11 inputs, 5 hidden, 2 outputs). The app provides an interactive [egui](https://github.com/emilk/egui) visualization with four control modes:

- **Manual** -- drive the robot with keyboard controls
- **Auto** -- run a single neural network controller forward
- **Evolve** -- evolve controllers using objective-based (fitness) evolution toward the maze goal
- **Novelty** -- evolve controllers using novelty search, which rewards behavioral novelty rather than proximity to the goal

Three maze environments are included: Medium, Hard, and a procedurally generated Eller maze.

<img width="880" height="625" alt="image" src="https://github.com/user-attachments/assets/c46c1a26-8125-4523-a830-6129cdcd45e7" />

## Paper

Lehman, J. and Stanley, K.O. (2011). [Abandoning Objectives: Evolution Through the Search for Novelty Alone](https://www.cs.swarthmore.edu/~meeden/DevelopmentalRobotics/lehman_ecj11.pdf). *Evolutionary Computation*, 19(2), 189-223.

## Getting Started

### Prerequisites

- [Rust](https://www.rust-lang.org/tools/install) (edition 2024)

### Run

```bash
make run-rust
make run-swift
```

The interesting bit, imo, is comparing the performance of Eller mazes between Evolve and
Novelty modes. Sometimes the Evolve mode gets stuck with no progress apaprently ever to be made again, but I've never seen the Novelty algorithm get stuck.
