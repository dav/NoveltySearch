import Foundation

let robotRadius: Double = 8.0
let maxSensorRange: Double = 100.0

/// Rangefinder angles relative to heading (in radians).
/// -90, -45, 0 (forward), 45, 90, 180 (rear)
private let rangefinderAngles: [Double] = [
    -.pi / 2, -.pi / 4, 0, .pi / 4, .pi / 2, .pi,
]

struct Robot {
    var x: Double
    var y: Double
    var heading: Double = 0  // radians, 0 = right, pi/2 = down
    var radius: Double = robotRadius

    init(x: Double, y: Double) {
        self.x = x
        self.y = y
    }

    /// Move the robot by one timestep.
    mutating func step(angVel: Double, speed: Double, maze: Maze) {
        heading += angVel
        heading = heading.truncatingRemainder(dividingBy: .pi * 2)
        if heading < 0 { heading += .pi * 2 }

        let newX = x + cos(heading) * speed
        let newY = y + sin(heading) * speed
        let resolved = maze.resolveCollision(pos: (newX, newY), radius: radius)
        x = resolved.0
        y = resolved.1
    }

    /// Cast 6 rangefinder rays and return normalized distances [0, 1].
    func rangefinders(maze: Maze) -> [Double] {
        rangefinderAngles.map { offset in
            let angle = heading + offset
            let dir = (cos(angle), sin(angle))
            return maze.rayCast(origin: (x, y), direction: dir, maxRange: maxSensorRange) / maxSensorRange
        }
    }

    /// Return the raw rangefinder hit points for visualization.
    func rangefinderEndpoints(maze: Maze) -> [(Double, Double)] {
        rangefinderAngles.map { offset in
            let angle = heading + offset
            let dir = (cos(angle), sin(angle))
            let dist = maze.rayCast(origin: (x, y), direction: dir, maxRange: maxSensorRange)
            return (x + dir.0 * dist, y + dir.1 * dist)
        }
    }

    /// 4 pie-slice radar sensors for goal direction.
    /// Returns [front, right, back, left] -- 1.0 if goal is in that slice, else 0.0.
    func radar(goal: (Double, Double)) -> [Double] {
        let dx = goal.0 - x, dy = goal.1 - y
        let angleToGoal = atan2(dy, dx)
        var relative = (angleToGoal - heading).truncatingRemainder(dividingBy: .pi * 2)
        if relative < 0 { relative += .pi * 2 }
        let deg = relative * 180 / .pi
        var sensors = [0.0, 0.0, 0.0, 0.0]
        let quadrant: Int
        if deg < 45 || deg >= 315 { quadrant = 0 }
        else if deg < 135 { quadrant = 1 }
        else if deg < 225 { quadrant = 2 }
        else { quadrant = 3 }
        sensors[quadrant] = 1.0
        return sensors
    }

    /// Full sensor input vector: 6 rangefinders + 4 radar + 1 bias = 11 values.
    func sensorInputs(maze: Maze) -> [Double] {
        rangefinders(maze: maze) + radar(goal: maze.goal) + [1.0]
    }

    /// Distance from robot center to goal.
    func distanceToGoal(_ goal: (Double, Double)) -> Double {
        let dx = goal.0 - x, dy = goal.1 - y
        return sqrt(dx * dx + dy * dy)
    }
}
