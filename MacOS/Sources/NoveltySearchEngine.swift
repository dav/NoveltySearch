import Foundation
import Observation

/// Novelty search state.
@Observable
class NoveltySearchEngine {
    static let populationSize = 150
    static let timesteps = 400
    static let mutationSigma = 0.1
    static let tournamentSize = 3
    static let successDistance = 5.0
    static let kNearest = 15
    static let initialRhoMin = 6.0

    var population: [Network]
    var noveltyScores: [Double]
    var generation: Int = 0
    var totalEvaluations: Int = 0

    // Behavior archive: final position + generation when added
    var archive: [((Double, Double), Int)] = []
    var rhoMin: Double = initialRhoMin
    private var evalsSinceLastAddition: Int = 0

    var allFinalPositions: [(Double, Double)] = []
    var bestNoveltyHistory: [Double] = []

    // Best individual that reached closest to goal (tracked but not used for selection)
    var bestTrajectory: [(Double, Double)] = []
    var closestDistance: Double = .infinity
    var solved = false

    // Last generation summary (for UI)
    var lastGenArchiveAdditions: Int = 0
    var lastGenClosestDist: Double = .infinity

    init() {
        population = (0..<Self.populationSize).map { _ in Network.random() }
        noveltyScores = Array(repeating: 0, count: Self.populationSize)
    }

    /// Run one generation of novelty search. Returns true if any robot reached the goal.
    @discardableResult
    func stepGeneration(maze: Maze) -> Bool {
        var finalPositions: [(Double, Double)] = []
        var trajectories: [[(Double, Double)]] = []
        finalPositions.reserveCapacity(Self.populationSize)
        trajectories.reserveCapacity(Self.populationSize)

        for network in population {
            let (finalPos, trajectory) = Self.evaluate(network: network, maze: maze)
            finalPositions.append(finalPos)
            trajectories.append(trajectory)
        }

        allFinalPositions = finalPositions
        totalEvaluations += Self.populationSize
        generation += 1

        var bestNovelty = -Double.infinity
        var additionsThisGen = 0
        var closestIdx = 0
        var closestDist = Double.infinity

        for i in 0..<Self.populationSize {
            let novelty = computeNovelty(point: finalPositions[i], populationBehaviors: finalPositions)
            noveltyScores[i] = novelty

            if novelty > bestNovelty { bestNovelty = novelty }

            if novelty > rhoMin {
                archive.append((finalPositions[i], generation))
                additionsThisGen += 1
            }

            let dist = distance(finalPositions[i], maze.goal)
            if dist < closestDist {
                closestDist = dist
                closestIdx = i
            }
        }

        lastGenArchiveAdditions = additionsThisGen
        lastGenClosestDist = closestDist

        if closestDist < closestDistance {
            closestDistance = closestDist
            bestTrajectory = trajectories[closestIdx]
        }

        if closestDist <= Self.successDistance {
            solved = true
            bestTrajectory = trajectories[closestIdx]
            return true
        }

        bestNoveltyHistory.append(bestNovelty)

        // Adaptive threshold for archive
        if additionsThisGen == 0 {
            evalsSinceLastAddition += Self.populationSize
            if evalsSinceLastAddition >= 2500 {
                rhoMin *= 0.95
                evalsSinceLastAddition = 0
            }
        } else {
            evalsSinceLastAddition = 0
            if additionsThisGen > 4 {
                rhoMin *= 1.20
            }
        }

        // Selection and reproduction using novelty scores
        let eliteIdx = noveltyScores.enumerated()
            .max(by: { $0.element < $1.element })?.offset ?? 0
        var newPopulation = [population[eliteIdx]]

        while newPopulation.count < Self.populationSize {
            let winner = tournamentSelect()
            newPopulation.append(population[winner].mutated(sigma: Self.mutationSigma))
        }
        population = newPopulation
        return false
    }

    /// Average distance to k-nearest neighbors in population + archive.
    private func computeNovelty(
        point: (Double, Double), populationBehaviors: [(Double, Double)]
    ) -> Double {
        var distances: [Double] = []
        for other in populationBehaviors {
            let d = distance(point, other)
            if d > 1e-10 { distances.append(d) }
        }
        for (archived, _) in archive {
            distances.append(distance(point, archived))
        }
        distances.sort()
        let k = min(Self.kNearest, distances.count)
        guard k > 0 else { return 0 }
        return distances[..<k].reduce(0, +) / Double(k)
    }

    private func tournamentSelect() -> Int {
        var bestIdx = Int.random(in: 0..<Self.populationSize)
        var bestScore = noveltyScores[bestIdx]
        for _ in 1..<Self.tournamentSize {
            let idx = Int.random(in: 0..<Self.populationSize)
            if noveltyScores[idx] > bestScore {
                bestScore = noveltyScores[idx]
                bestIdx = idx
            }
        }
        return bestIdx
    }

    private static func evaluate(
        network: Network, maze: Maze
    ) -> ((Double, Double), [(Double, Double)]) {
        var robot = Robot(x: maze.start.0, y: maze.start.1)
        var trajectory: [(Double, Double)] = []
        trajectory.reserveCapacity(timesteps)

        for _ in 0..<timesteps {
            let inputs = robot.sensorInputs(maze: maze)
            let (angVel, speed) = network.forward(inputs)
            robot.step(angVel: angVel, speed: speed, maze: maze)
            trajectory.append((robot.x, robot.y))
        }
        return ((robot.x, robot.y), trajectory)
    }
}

private func distance(_ a: (Double, Double), _ b: (Double, Double)) -> Double {
    let dx = a.0 - b.0, dy = a.1 - b.1
    return sqrt(dx * dx + dy * dy)
}
