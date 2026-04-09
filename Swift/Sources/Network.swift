import Foundation

/// A simple feedforward neural network: 11 inputs -> 5 hidden (tanh) -> 2 outputs (tanh).
/// Used to map sensor inputs to (angular_velocity, speed).
struct Network {
    var wIH: [[Double]]  // 5x11
    var bH: [Double]     // 5
    var wHO: [[Double]]  // 2x5
    var bO: [Double]     // 2

    static func random() -> Network {
        Network(
            wIH: (0..<5).map { _ in (0..<11).map { _ in Double.random(in: -1...1) } },
            bH: (0..<5).map { _ in Double.random(in: -1...1) },
            wHO: (0..<2).map { _ in (0..<5).map { _ in Double.random(in: -1...1) } },
            bO: (0..<2).map { _ in Double.random(in: -1...1) }
        )
    }

    /// Forward pass: inputs (11) -> (angular_velocity, speed).
    func forward(_ inputs: [Double]) -> (Double, Double) {
        var hidden = [Double](repeating: 0, count: 5)
        for h in 0..<5 {
            var sum = bH[h]
            for i in 0..<11 { sum += wIH[h][i] * inputs[i] }
            hidden[h] = tanh(sum)
        }

        var output = [Double](repeating: 0, count: 2)
        for o in 0..<2 {
            var sum = bO[o]
            for h in 0..<5 { sum += wHO[o][h] * hidden[h] }
            output[o] = tanh(sum)
        }

        let angVel = output[0] * 0.3
        let speed = (output[1] + 1) / 2 * 3
        return (angVel, speed)
    }

    /// Return a mutated copy. Each weight is perturbed by Gaussian noise with std dev sigma.
    func mutated(sigma: Double) -> Network {
        var child = self

        // Box-Muller transform for standard normal samples
        func gauss() -> Double {
            let u1 = Double.random(in: 1e-10..<1)
            let u2 = Double.random(in: 0..<(.pi * 2))
            return sqrt(-2 * log(u1)) * cos(u2)
        }

        for h in 0..<5 {
            for i in 0..<11 { child.wIH[h][i] += sigma * gauss() }
        }
        for h in 0..<5 { child.bH[h] += sigma * gauss() }
        for o in 0..<2 {
            for h in 0..<5 { child.wHO[o][h] += sigma * gauss() }
        }
        for o in 0..<2 { child.bO[o] += sigma * gauss() }
        return child
    }
}
