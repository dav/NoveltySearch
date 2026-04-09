import Foundation

struct Wall {
    var a: (Double, Double)
    var b: (Double, Double)

    init(_ ax: Double, _ ay: Double, _ bx: Double, _ by: Double) {
        a = (ax, ay)
        b = (bx, by)
    }
}

struct Maze {
    var walls: [Wall]
    var start: (Double, Double)
    var goal: (Double, Double)
    var bounds: (Double, Double)

    /// Cast a ray from origin in direction and return the distance to the nearest wall hit.
    /// Returns maxRange if no wall is hit within that range.
    func rayCast(origin: (Double, Double), direction: (Double, Double), maxRange: Double) -> Double {
        var nearest = maxRange
        for wall in walls {
            if let t = raySegmentIntersection(
                origin: origin, dir: direction, segA: wall.a, segB: wall.b
            ) {
                if t > 0 && t < nearest {
                    nearest = t
                }
            }
        }
        return nearest
    }

    /// Push a circle out of all walls it overlaps. Returns the corrected position.
    func resolveCollision(pos: (Double, Double), radius: Double) -> (Double, Double) {
        var corrected = pos
        for _ in 0..<4 {
            var pushed = false
            for wall in walls {
                if let push = circleSegmentPush(
                    center: corrected, radius: radius, segA: wall.a, segB: wall.b
                ) {
                    corrected.0 += push.0
                    corrected.1 += push.1
                    pushed = true
                }
            }
            if !pushed { break }
        }
        corrected.0 = min(max(corrected.0, radius), bounds.0 - radius)
        corrected.1 = min(max(corrected.1, radius), bounds.1 - radius)
        return corrected
    }

    // MARK: - Maze generators

    static func medium() -> Maze {
        let (w, h) = (200.0, 200.0)
        var walls = boundaryWalls(w: w, h: h)
        walls.append(contentsOf: [
            Wall(0, 66, 140, 66),
            Wall(60, 133, 200, 133),
        ])
        return Maze(walls: walls, start: (30, 180), goal: (170, 33), bounds: (w, h))
    }

    static func hard() -> Maze {
        let (w, h) = (300.0, 200.0)
        var walls = boundaryWalls(w: w, h: h)
        walls.append(contentsOf: [
            Wall(0, 170, 230, 170),
            Wall(260, 170, 300, 170),
            Wall(0, 110, 40, 110),
            Wall(70, 110, 300, 110),
            Wall(0, 50, 230, 50),
            Wall(260, 50, 300, 50),
        ])
        return Maze(walls: walls, start: (30, 185), goal: (270, 25), bounds: (w, h))
    }

    /// Generate a random maze using Eller's algorithm.
    static func eller() -> Maze {
        let cols = 8
        let rows = 8
        let (mazeW, mazeH) = (300.0, 300.0)
        let cellW = mazeW / Double(cols)
        let cellH = mazeH / Double(rows)

        var rightWalls = Array(repeating: Array(repeating: true, count: cols), count: rows)
        var bottomWalls = Array(repeating: Array(repeating: true, count: cols), count: rows)

        var sets = Array(0..<cols)
        var nextSetId = cols

        for row in 0..<rows {
            // Randomly merge adjacent cells in different sets
            for col in 0..<(cols - 1) {
                if sets[col] != sets[col + 1] && (row == rows - 1 || Bool.random()) {
                    rightWalls[row][col] = false
                    let oldSet = sets[col + 1]
                    let newSet = sets[col]
                    for i in 0..<cols {
                        if sets[i] == oldSet { sets[i] = newSet }
                    }
                }
            }

            if row == rows - 1 { break }

            // For each set, ensure at least one downward connection
            var setColumns: [Int: [Int]] = [:]
            for (col, setId) in sets.enumerated() {
                setColumns[setId, default: []].append(col)
            }

            var connectedDown = Array(repeating: false, count: cols)
            for (_, columns) in setColumns {
                var madeOne = false
                for col in columns {
                    if Double.random(in: 0..<1) < 0.4 || !madeOne {
                        bottomWalls[row][col] = false
                        connectedDown[col] = true
                        madeOne = true
                    }
                }
            }

            // Prepare next row
            for col in 0..<cols {
                if !connectedDown[col] {
                    sets[col] = nextSetId
                    nextSetId += 1
                }
            }
        }

        // Convert grid walls to Wall segments
        var walls = boundaryWalls(w: mazeW, h: mazeH)
        for row in 0..<rows {
            for col in 0..<cols {
                let x = Double(col) * cellW
                let y = Double(row) * cellH
                if col < cols - 1 && rightWalls[row][col] {
                    walls.append(Wall(x + cellW, y, x + cellW, y + cellH))
                }
                if row < rows - 1 && bottomWalls[row][col] {
                    walls.append(Wall(x, y + cellH, x + cellW, y + cellH))
                }
            }
        }

        let start = (cellW / 2, cellH / 2)
        let goal = (mazeW - cellW / 2, mazeH - cellH / 2)
        return Maze(walls: walls, start: start, goal: goal, bounds: (mazeW, mazeH))
    }
}

// MARK: - Geometry helpers

private func boundaryWalls(w: Double, h: Double) -> [Wall] {
    [Wall(0, 0, w, 0), Wall(w, 0, w, h), Wall(w, h, 0, h), Wall(0, h, 0, 0)]
}

private func raySegmentIntersection(
    origin: (Double, Double), dir: (Double, Double),
    segA: (Double, Double), segB: (Double, Double)
) -> Double? {
    let dx = dir.0, dy = dir.1
    let ex = segB.0 - segA.0, ey = segB.1 - segA.1
    let denom = dx * ey - dy * ex
    guard abs(denom) > 1e-10 else { return nil }
    let fx = segA.0 - origin.0, fy = segA.1 - origin.1
    let t = (fx * ey - fy * ex) / denom
    let u = (fx * dy - fy * dx) / denom
    if t > 0 && u >= 0 && u <= 1 { return t }
    return nil
}

private func circleSegmentPush(
    center: (Double, Double), radius: Double,
    segA: (Double, Double), segB: (Double, Double)
) -> (Double, Double)? {
    let ex = segB.0 - segA.0, ey = segB.1 - segA.1
    let lenSq = ex * ex + ey * ey
    guard lenSq > 1e-10 else { return nil }
    let t = min(max(((center.0 - segA.0) * ex + (center.1 - segA.1) * ey) / lenSq, 0), 1)
    let closestX = segA.0 + t * ex
    let closestY = segA.1 + t * ey
    let dx = center.0 - closestX, dy = center.1 - closestY
    let distSq = dx * dx + dy * dy
    guard distSq < radius * radius && distSq > 1e-10 else { return nil }
    let dist = sqrt(distSq)
    let overlap = radius - dist
    return (dx / dist * overlap, dy / dist * overlap)
}
