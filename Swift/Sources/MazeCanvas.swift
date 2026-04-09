import SwiftUI

struct MazeCanvas: View {
    var state: AppState

    var body: some View {
        Canvas { context, size in
            let mazeW = state.maze.bounds.0
            let mazeH = state.maze.bounds.1
            let scale = min(size.width / mazeW, size.height / mazeH) * 0.95
            let offsetX = (size.width - mazeW * scale) / 2
            let offsetY = (size.height - mazeH * scale) / 2

            func toScreen(_ x: Double, _ y: Double) -> CGPoint {
                CGPoint(x: offsetX + x * scale, y: offsetY + y * scale)
            }

            // Maze background
            let bgRect = CGRect(
                origin: toScreen(0, 0),
                size: CGSize(width: mazeW * scale, height: mazeH * scale)
            )
            context.fill(Path(bgRect), with: .color(.init(white: 0.94)))

            // Walls
            for wall in state.maze.walls {
                var path = Path()
                path.move(to: toScreen(wall.a.0, wall.a.1))
                path.addLine(to: toScreen(wall.b.0, wall.b.1))
                context.stroke(path, with: .color(.init(white: 0.16)), lineWidth: 2)
            }

            // Goal
            let goalPt = toScreen(state.maze.goal.0, state.maze.goal.1)
            context.fill(
                Path(ellipseIn: CGRect(x: goalPt.x - 8, y: goalPt.y - 8, width: 16, height: 16)),
                with: .color(.init(red: 50.0/255, green: 200.0/255, blue: 50.0/255))
            )
            context.stroke(
                Path(ellipseIn: CGRect(x: goalPt.x - 8, y: goalPt.y - 8, width: 16, height: 16)),
                with: .color(.init(red: 20.0/255, green: 120.0/255, blue: 20.0/255)), lineWidth: 1.5
            )

            // Start marker
            let startPt = toScreen(state.maze.start.0, state.maze.start.1)
            context.stroke(
                Path(ellipseIn: CGRect(x: startPt.x - 6, y: startPt.y - 6, width: 12, height: 12)),
                with: .color(.init(red: 100.0/255, green: 100.0/255, blue: 200.0/255)), lineWidth: 1.5
            )

            // Evolve mode: trajectory trail + final positions scatter
            if state.mode == .evolve {
                if !state.replayTrajectory.isEmpty {
                    let end = min(state.replayIndex, state.replayTrajectory.count)
                    for i in 1..<end {
                        let prev = state.replayTrajectory[i - 1]
                        let curr = state.replayTrajectory[i]
                        var path = Path()
                        path.move(to: toScreen(prev.0, prev.1))
                        path.addLine(to: toScreen(curr.0, curr.1))
                        context.stroke(
                            path,
                            with: .color(.init(red: 60.0/255, green: 120.0/255, blue: 220.0/255, opacity: 120.0/255)),
                            lineWidth: 1.5
                        )
                    }
                }
                for pos in state.evolution.allFinalPositions {
                    let p = toScreen(pos.0, pos.1)
                    context.fill(
                        Path(ellipseIn: CGRect(x: p.x - 2, y: p.y - 2, width: 4, height: 4)),
                        with: .color(.init(red: 200.0/255, green: 100.0/255, blue: 50.0/255, opacity: 150.0/255))
                    )
                }
            }

            // Novelty mode: archive points, population dots, trajectory
            if state.mode == .novelty {
                // Archive points colored by age
                let maxGen = max(state.noveltySearch.generation, 1)
                for (pos, addedGen) in state.noveltySearch.archive {
                    let t = Double(addedGen) / Double(maxGen)
                    let r = (180 - 20 * t) / 255
                    let g = (140 - 90 * t) / 255
                    let b = (200 + 55 * t) / 255
                    let a = (80 + 140 * t) / 255
                    let p = toScreen(pos.0, pos.1)
                    context.fill(
                        Path(ellipseIn: CGRect(x: p.x - 2.5, y: p.y - 2.5, width: 5, height: 5)),
                        with: .color(.init(red: r, green: g, blue: b, opacity: a))
                    )
                }

                // Current generation final positions colored by novelty score
                let positions = state.noveltySearch.allFinalPositions
                let scores = state.noveltySearch.noveltyScores
                if !positions.isEmpty && scores.count == positions.count {
                    let minS = scores.min() ?? 0
                    let maxS = scores.max() ?? 1
                    let range = max(maxS - minS, 1e-10)

                    for (pos, score) in zip(positions, scores) {
                        let t = (score - minS) / range
                        let r = (120 + 135 * t) / 255
                        let g = (70 + 150 * t) / 255
                        let b = (30 + 20 * t) / 255
                        let a = (100 + 130 * t) / 255
                        let sz = 1.5 + 3.5 * t
                        let p = toScreen(pos.0, pos.1)
                        context.fill(
                            Path(ellipseIn: CGRect(x: p.x - sz, y: p.y - sz, width: sz * 2, height: sz * 2)),
                            with: .color(.init(red: r, green: g, blue: b, opacity: a))
                        )
                    }
                }

                // Trajectory trail for closest-to-goal robot
                if !state.noveltyReplayTrajectory.isEmpty {
                    let end = min(state.noveltyReplayIndex, state.noveltyReplayTrajectory.count)
                    for i in 1..<end {
                        let prev = state.noveltyReplayTrajectory[i - 1]
                        let curr = state.noveltyReplayTrajectory[i]
                        var path = Path()
                        path.move(to: toScreen(prev.0, prev.1))
                        path.addLine(to: toScreen(curr.0, curr.1))
                        context.stroke(
                            path,
                            with: .color(.init(red: 60.0/255, green: 200.0/255, blue: 120.0/255, opacity: 150.0/255)),
                            lineWidth: 1.5
                        )
                    }
                }
            }

            // Rangefinder rays
            let endpoints = state.robot.rangefinderEndpoints(maze: state.maze)
            let robotPt = toScreen(state.robot.x, state.robot.y)
            for ep in endpoints {
                var path = Path()
                path.move(to: robotPt)
                path.addLine(to: toScreen(ep.0, ep.1))
                context.stroke(
                    path,
                    with: .color(.init(red: 1, green: 20.0/255, blue: 147.0/255)),
                    lineWidth: 2
                )
            }

            // Robot body
            let robotR = state.robot.radius * scale
            context.fill(
                Path(ellipseIn: CGRect(x: robotPt.x - robotR, y: robotPt.y - robotR, width: robotR * 2, height: robotR * 2)),
                with: .color(.init(red: 60.0/255, green: 120.0/255, blue: 220.0/255))
            )
            context.stroke(
                Path(ellipseIn: CGRect(x: robotPt.x - robotR, y: robotPt.y - robotR, width: robotR * 2, height: robotR * 2)),
                with: .color(.init(red: 30.0/255, green: 60.0/255, blue: 140.0/255)),
                lineWidth: 1.5
            )

            // Heading arrow
            let arrowLen = state.robot.radius * 1.8
            let arrowEnd = toScreen(
                state.robot.x + cos(state.robot.heading) * arrowLen,
                state.robot.y + sin(state.robot.heading) * arrowLen
            )
            var arrowPath = Path()
            arrowPath.move(to: robotPt)
            arrowPath.addLine(to: arrowEnd)
            context.stroke(arrowPath, with: .color(.white), lineWidth: 2)
        }
        .background(.black)
    }
}
