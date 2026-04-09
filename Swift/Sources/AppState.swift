import Foundation
import SwiftUI

enum MazeChoice: String, CaseIterable {
    case medium = "Medium"
    case hard = "Hard"
    case eller = "Eller"
}

enum Mode: String, CaseIterable {
    case manual = "Manual"
    case auto = "Auto"
    case evolve = "Evolve"
    case novelty = "Novelty"
}

@Observable
class AppState {
    var mazeChoice: MazeChoice = .medium
    var maze: Maze
    var robot: Robot
    var mode: Mode = .manual
    var network: Network = .random()
    var stepsPerFrame: Int = 1
    var stepCount: Int = 0

    // Manual-mode inputs (held keys)
    var keyForward = false
    var keyBack = false
    var keyLeft = false
    var keyRight = false

    // Evolution state
    var evolution = Evolution()
    var evoRunning = false
    var evoGensPerFrame: Int = 1
    var replayTrajectory: [(Double, Double)] = []
    var replayIndex: Int = 0

    // Novelty search state
    var noveltySearch = NoveltySearchEngine()
    var noveltyRunning = false
    var noveltyGensPerFrame: Int = 1
    var noveltyReplayTrajectory: [(Double, Double)] = []
    var noveltyReplayIndex: Int = 0
    var noveltyStepOne = false

    // Keyboard monitor handles
    private var keyDownMonitor: Any?
    private var keyUpMonitor: Any?

    init() {
        let m = Maze.medium()
        maze = m
        robot = Robot(x: m.start.0, y: m.start.1)
    }

    // MARK: - Keyboard

    func setupKeyboard() {
        keyDownMonitor = NSEvent.addLocalMonitorForEvents(matching: .keyDown) { [weak self] event in
            self?.handleKey(event.keyCode, pressed: true)
            return event
        }
        keyUpMonitor = NSEvent.addLocalMonitorForEvents(matching: .keyUp) { [weak self] event in
            self?.handleKey(event.keyCode, pressed: false)
            return event
        }
    }

    func teardownKeyboard() {
        if let m = keyDownMonitor { NSEvent.removeMonitor(m) }
        if let m = keyUpMonitor { NSEvent.removeMonitor(m) }
    }

    private func handleKey(_ keyCode: UInt16, pressed: Bool) {
        switch keyCode {
        case 13, 126: keyForward = pressed  // W, Up
        case 1, 125: keyBack = pressed      // S, Down
        case 0, 123: keyLeft = pressed      // A, Left
        case 2, 124: keyRight = pressed     // D, Right
        default: break
        }
    }

    // MARK: - Tick (called each frame)

    func tick() {
        switch mode {
        case .manual, .auto:
            for _ in 0..<stepsPerFrame {
                switch mode {
                case .manual:
                    let angVel = keyLeft ? -0.1 : (keyRight ? 0.1 : 0)
                    let speed = keyForward ? 3.0 : (keyBack ? -1.5 : 0)
                    robot.step(angVel: angVel, speed: speed, maze: maze)
                case .auto:
                    let inputs = robot.sensorInputs(maze: maze)
                    let (angVel, speed) = network.forward(inputs)
                    robot.step(angVel: angVel, speed: speed, maze: maze)
                default: break
                }
                stepCount += 1
            }

        case .evolve:
            if evoRunning && !evolution.solved {
                for _ in 0..<evoGensPerFrame {
                    evolution.stepGeneration(maze: maze)
                    if evolution.solved { break }
                }
                replayTrajectory = evolution.bestTrajectory
                replayIndex = replayTrajectory.count
            }
            if !replayTrajectory.isEmpty && replayIndex < replayTrajectory.count {
                let pos = replayTrajectory[replayIndex]
                robot.x = pos.0
                robot.y = pos.1
                replayIndex += stepsPerFrame
            }

        case .novelty:
            if noveltyStepOne && !noveltySearch.solved {
                noveltySearch.stepGeneration(maze: maze)
                noveltyStepOne = false
                noveltyReplayTrajectory = noveltySearch.bestTrajectory
                noveltyReplayIndex = noveltyReplayTrajectory.count
            }
            if noveltyRunning && !noveltySearch.solved {
                for _ in 0..<noveltyGensPerFrame {
                    noveltySearch.stepGeneration(maze: maze)
                    if noveltySearch.solved { break }
                }
                noveltyReplayTrajectory = noveltySearch.bestTrajectory
                noveltyReplayIndex = noveltyReplayTrajectory.count
            }
            if !noveltyReplayTrajectory.isEmpty && noveltyReplayIndex < noveltyReplayTrajectory.count {
                let pos = noveltyReplayTrajectory[noveltyReplayIndex]
                robot.x = pos.0
                robot.y = pos.1
                noveltyReplayIndex += stepsPerFrame
            }
        }
    }

    // MARK: - Reset helpers

    func reset() {
        robot = Robot(x: maze.start.0, y: maze.start.1)
        stepCount = 0
        replayTrajectory.removeAll()
        replayIndex = 0
    }

    func resetEvolution() {
        evolution = Evolution()
        evoRunning = false
        replayTrajectory.removeAll()
        replayIndex = 0
    }

    func resetNovelty() {
        noveltySearch = NoveltySearchEngine()
        noveltyRunning = false
        noveltyStepOne = false
        noveltyReplayTrajectory.removeAll()
        noveltyReplayIndex = 0
    }

    func switchMaze(_ choice: MazeChoice) {
        mazeChoice = choice
        switch choice {
        case .medium: maze = .medium()
        case .hard: maze = .hard()
        case .eller: maze = .eller()
        }
        reset()
        resetEvolution()
        resetNovelty()
    }
}
