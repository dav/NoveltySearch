import Foundation
import Observation

/// Fitness-based evolutionary search state.
@Observable
class Evolution {
    static let populationSize = 150
    static let timesteps = 400
    static let mutationSigma = 0.1
    static let tournamentSize = 3
    static let successDistance = 5.0

    var population: [Network]
    var fitnesses: [Double]
    var generation = 0
    var totalEvaluations = 0
    var bestFitnessHistory: [Double] = []
    var bestTrajectory: [(Double, Double)] = []
    var bestNetwork: Network
    var bestFitness = -Double.infinity
    var solved = false
    var allFinalPositions: [(Double, Double)] = []

    init() {
        let pop = (0..<Self.populationSize).map { _ in Network.random() }
        population = pop
        fitnesses = Array(repeating: 0, count: Self.populationSize)
        bestNetwork = pop[0]
    }

    /// Run one generation of evolution. Returns true if the goal was reached.
    @discardableResult
    func stepGeneration(maze: Maze) -> Bool {
        var bestIdx = 0
        var genBestFitness = -Double.infinity
        var bestResult: (fitness: Double, finalPos: (Double, Double), trajectory: [(Double, Double)])?

        allFinalPositions.removeAll(keepingCapacity: true)

        for i in 0..<Self.populationSize {
            let result = Self.evaluate(network: population[i], maze: maze)
            fitnesses[i] = result.fitness
            allFinalPositions.append(result.finalPos)
            if result.fitness > genBestFitness {
                genBestFitness = result.fitness
                bestIdx = i
                bestResult = result
            }
        }

        totalEvaluations += Self.populationSize
        generation += 1

        if genBestFitness > bestFitness {
            bestFitness = genBestFitness
            bestNetwork = population[bestIdx]
            if let result = bestResult {
                bestTrajectory = result.trajectory
            }
        }

        bestFitnessHistory.append(genBestFitness)

        if genBestFitness >= 0 {
            solved = true
            return true
        }

        // Selection and reproduction
        var newPopulation = [population[bestIdx]]  // elitism
        while newPopulation.count < Self.populationSize {
            let winner = tournamentSelect()
            newPopulation.append(population[winner].mutated(sigma: Self.mutationSigma))
        }
        population = newPopulation
        return false
    }

    private func tournamentSelect() -> Int {
        var bestIdx = Int.random(in: 0..<Self.populationSize)
        var bestFit = fitnesses[bestIdx]
        for _ in 1..<Self.tournamentSize {
            let idx = Int.random(in: 0..<Self.populationSize)
            if fitnesses[idx] > bestFit {
                bestFit = fitnesses[idx]
                bestIdx = idx
            }
        }
        return bestIdx
    }

    private static func evaluate(
        network: Network, maze: Maze
    ) -> (fitness: Double, finalPos: (Double, Double), trajectory: [(Double, Double)]) {
        var robot = Robot(x: maze.start.0, y: maze.start.1)
        var trajectory: [(Double, Double)] = []
        trajectory.reserveCapacity(timesteps)

        for _ in 0..<timesteps {
            let inputs = robot.sensorInputs(maze: maze)
            let (angVel, speed) = network.forward(inputs)
            robot.step(angVel: angVel, speed: speed, maze: maze)
            trajectory.append((robot.x, robot.y))
        }

        let finalDist = robot.distanceToGoal(maze.goal)
        let fitness = -(finalDist - successDistance)
        return (fitness, (robot.x, robot.y), trajectory)
    }
}
