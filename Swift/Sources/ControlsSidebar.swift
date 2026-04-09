import SwiftUI

struct ControlsSidebar: View {
    @Bindable var state: AppState

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 8) {
                Text("Controls").font(.headline)
                Divider()

                // Maze selection
                Text("Maze:")
                Picker("", selection: $state.mazeChoice) {
                    ForEach(MazeChoice.allCases, id: \.self) { c in
                        Text(c.rawValue).tag(c)
                    }
                }
                .pickerStyle(.segmented)
                .onChange(of: state.mazeChoice) { _, newValue in
                    state.switchMaze(newValue)
                }

                if state.mazeChoice == .eller {
                    Button("New maze") { state.switchMaze(.eller) }
                }

                Divider()

                // Mode selection
                Text("Mode:")
                Picker("", selection: $state.mode) {
                    ForEach(Mode.allCases, id: \.self) { m in
                        Text(m.rawValue).tag(m)
                    }
                }
                .pickerStyle(.segmented)

                if state.mode == .manual {
                    Text("WASD or arrow keys to drive").font(.caption).foregroundStyle(.secondary)
                }

                Divider()

                // Mode-specific controls
                switch state.mode {
                case .manual, .auto:
                    manualAutoControls
                case .evolve:
                    evolveControls
                case .novelty:
                    noveltyControls
                }
            }
            .padding(12)
        }
    }

    // MARK: - Manual / Auto

    @ViewBuilder
    private var manualAutoControls: some View {
        HStack {
            Text("Steps/frame:")
            Slider(value: .init(
                get: { Double(state.stepsPerFrame) },
                set: { state.stepsPerFrame = Int($0) }
            ), in: 1...20, step: 1)
            Text("\(state.stepsPerFrame)")
        }

        Divider()

        Button("Reset robot") { state.reset() }

        if state.mode == .auto {
            Button("New random network") {
                state.network = .random()
                state.reset()
            }
        }

        Divider()

        Text("Stats").font(.headline)
        StatRow("Step", "\(state.stepCount)")
        StatRow("Position", String(format: "(%.1f, %.1f)", state.robot.x, state.robot.y))
        StatRow("Heading", String(format: "%.1f\u{00B0}", state.robot.heading * 180 / .pi))
        StatRow("Goal dist", String(format: "%.1f", state.robot.distanceToGoal(state.maze.goal)))

        Divider()
        Text("Rangefinders:").font(.subheadline)
        let rf = state.robot.rangefinders(maze: state.maze)
        let rfLabels = ["-90\u{00B0}", "-45\u{00B0}", "0\u{00B0}", "45\u{00B0}", "90\u{00B0}", "180\u{00B0}"]
        ForEach(Array(zip(rfLabels, rf)), id: \.0) { label, val in
            StatRow("  \(label)", String(format: "%.2f", val))
        }

        Divider()
        Text("Radar:").font(.subheadline)
        let rd = state.robot.radar(goal: state.maze.goal)
        let rdLabels = ["Front", "Right", "Back", "Left"]
        ForEach(Array(zip(rdLabels, rd)), id: \.0) { label, val in
            StatRow("  \(label)", String(format: "%.0f", val))
        }
    }

    // MARK: - Evolve

    @ViewBuilder
    private var evolveControls: some View {
        if state.evolution.solved {
            Text("SOLVED!").foregroundStyle(.green).bold()
        }

        HStack {
            if state.evoRunning {
                Button("Pause") { state.evoRunning = false }
            } else {
                Button("Start") { state.evoRunning = true }
            }
            Button("Reset") {
                state.resetEvolution()
                state.reset()
            }
        }

        HStack {
            Text("Gens/frame:")
            Slider(value: .init(
                get: { Double(state.evoGensPerFrame) },
                set: { state.evoGensPerFrame = Int($0) }
            ), in: 1...50, step: 1)
            Text("\(state.evoGensPerFrame)")
        }

        Button("Replay best") {
            state.replayIndex = 0
            state.robot = Robot(x: state.maze.start.0, y: state.maze.start.1)
        }

        HStack {
            Text("Replay speed:")
            Slider(value: .init(
                get: { Double(state.stepsPerFrame) },
                set: { state.stepsPerFrame = Int($0) }
            ), in: 1...20, step: 1)
            Text("\(state.stepsPerFrame)")
        }

        Divider()
        Text("Evolution").font(.headline)
        StatRow("Generation", "\(state.evolution.generation)")
        StatRow("Evaluations", "\(state.evolution.totalEvaluations)")
        StatRow("Best fitness", String(format: "%.1f", state.evolution.bestFitness))

        // Fitness plot
        if !state.evolution.bestFitnessHistory.isEmpty {
            Divider()
            Text("Fitness over generations:").font(.subheadline)
            MiniPlot(
                data: state.evolution.bestFitnessHistory,
                color: .init(red: 100.0/255, green: 200.0/255, blue: 100.0/255)
            )
            .frame(height: 80)
        }
    }

    // MARK: - Novelty

    @ViewBuilder
    private var noveltyControls: some View {
        if state.noveltySearch.solved {
            Text("SOLVED!").foregroundStyle(.green).bold()
        }

        HStack {
            if state.noveltyRunning {
                Button("Pause") { state.noveltyRunning = false }
            } else {
                Button("Start") { state.noveltyRunning = true }
                Button("Step 1 gen") { state.noveltyStepOne = true }
            }
            Button("Reset") {
                state.resetNovelty()
                state.reset()
            }
        }

        HStack {
            Text("Gens/frame:")
            Slider(value: .init(
                get: { Double(state.noveltyGensPerFrame) },
                set: { state.noveltyGensPerFrame = Int($0) }
            ), in: 1...50, step: 1)
            Text("\(state.noveltyGensPerFrame)")
        }

        Button("Replay closest") {
            state.noveltyReplayIndex = 0
            state.robot = Robot(x: state.maze.start.0, y: state.maze.start.1)
        }

        HStack {
            Text("Replay speed:")
            Slider(value: .init(
                get: { Double(state.stepsPerFrame) },
                set: { state.stepsPerFrame = Int($0) }
            ), in: 1...20, step: 1)
            Text("\(state.stepsPerFrame)")
        }

        Divider()
        Text("Novelty Search").font(.headline)
        StatRow("Generation", "\(state.noveltySearch.generation)")
        StatRow("Evaluations", "\(state.noveltySearch.totalEvaluations)")
        StatRow("Archive size", "\(state.noveltySearch.archive.count)")
        StatRow("Closest to goal", String(format: "%.1f", state.noveltySearch.closestDistance))
        StatRow("Threshold (\u{03c1}_min)", String(format: "%.2f", state.noveltySearch.rhoMin))

        if state.noveltySearch.generation > 0 {
            Divider()
            Text("Last Generation").font(.headline)
            StatRow("Archived", "+\(state.noveltySearch.lastGenArchiveAdditions) (total: \(state.noveltySearch.archive.count))")
            StatRow("Closest to goal", String(format: "%.1f", state.noveltySearch.lastGenClosestDist))
        }

        // Novelty score plot
        if !state.noveltySearch.bestNoveltyHistory.isEmpty {
            Divider()
            Text("Best novelty / gen:").font(.subheadline)
            MiniPlot(
                data: state.noveltySearch.bestNoveltyHistory,
                color: .init(red: 200.0/255, green: 100.0/255, blue: 1)
            )
            .frame(height: 80)
        }
    }
}

// MARK: - Helper views

private struct StatRow: View {
    let label: String
    let value: String
    init(_ label: String, _ value: String) {
        self.label = label
        self.value = value
    }
    var body: some View {
        HStack {
            Text(label).foregroundStyle(.secondary)
            Spacer()
            Text(value).monospacedDigit()
        }
        .font(.caption)
    }
}

struct MiniPlot: View {
    let data: [Double]
    let color: Color

    var body: some View {
        Canvas { context, size in
            guard data.count >= 2 else { return }
            let minV = data.min()!
            let maxV = data.max()!
            let range = max(maxV - minV, 1)

            context.fill(Path(CGRect(origin: .zero, size: size)), with: .color(.init(white: 0.12)))

            var path = Path()
            for (i, v) in data.enumerated() {
                let x = Double(i) / Double(data.count - 1) * size.width
                let y = size.height - (v - minV) / range * size.height
                if i == 0 { path.move(to: CGPoint(x: x, y: y)) }
                else { path.addLine(to: CGPoint(x: x, y: y)) }
            }
            context.stroke(path, with: .color(color), lineWidth: 1.5)
        }
        .clipShape(RoundedRectangle(cornerRadius: 3))
    }
}
