# @tscircuit/fastforce

Fast force-directed graph solving with support for segment-applied forces.

[View online](

## Installation

```bash
npm install @tscircuit/fastforce
```

## Usage

```ts
import { ForceRelaxationSolver } from "@tscircuit/fastforce"
import type { ForceRelaxationProblem } from "@tscircuit/fastforce"

const problem: ForceRelaxationProblem = {
  bounds: { minX: 0, minY: 0, maxX: 100, maxY: 100 },
  boundaryPadding: 10,
  layerIds: ["0", "1"],

  entities: {
    points: [
      {
        pointId: "p1",
        position: { x: 0, y: 20 },
        movable: false,
        radius: 0,
        layerIds: ["0"],
      },
      {
        pointId: "p2",
        position: { x: 50, y: 20 },
        movable: true,
        radius: 0,
        layerIds: ["0", "1"],
      },
    ],
    segments: [
      {
        segmentId: "s1",
        startPointId: "p1",
        endPointId: "p2",
        width: 2,
        layerId: "0",
        fixedLength: false,
        fixedOrientation: false,
      },
    ],
  },

  interactions: {
    segSegRepel: {
      strength: 1,
      exponentialDecay: 0.5,
      overlapMultiplier: 5,
      minSeparation: 10,
    },
    pointSegRepel: {
      strength: 1,
      exponentialDecay: 0.5,
      overlapMultiplier: 5,
      minSeparation: 10,
    },
    boundsKeepIn: {
      strength: 2,
      exponentialDecay: 1,
      overlapMultiplier: 10,
    },
    fixedLengthCorrection: {
      strength: 1,
      exponentialDecay: 0,
    },
    fixedOrientationCorrection: {
      strength: 5,
      exponentialDecay: 0,
    },
  },

  solve: {
    maxSteps: 300,
    stepSize: 0.1,
    epsilonMove: 0.01,
    maxMovePerStep: 2,
    friction: 0.1,
    relaxationSteps: 100,
  },
}

const solver = new ForceRelaxationSolver(problem)

// Run until solved or max iterations reached
solver.solve()

// Access updated point positions
for (const point of problem.entities.points) {
  console.log(`${point.pointId}: (${point.position.x}, ${point.position.y})`)
}
```

## API

### `ForceRelaxationSolver`

The main solver class. Extends `BaseSolver` from `@tscircuit/solver-utils`.

- `constructor(problem: ForceRelaxationProblem)` - Create a new solver
- `solve()` - Run the solver until convergence or max iterations
- `step()` - Run a single iteration
- `visualize()` - Returns a `GraphicsObject` for visualization

### `ForceRelaxationProblem`

Configuration object with the following properties:

| Property            | Description                                            |
| ------------------- | ------------------------------------------------------ |
| `bounds`            | Axis-aligned bounding box `{ minX, minY, maxX, maxY }` |
| `boundaryPadding`   | Inset from bounds for keep-in constraints              |
| `layerIds`          | Array of layer identifiers                             |
| `entities.points`   | Array of point entities                                |
| `entities.segments` | Array of segment entities                              |
| `interactions`      | Force interaction parameters                           |
| `solve`             | Solver parameters                                      |

### Solver Parameters

| Parameter         | Description                           | Default |
| ----------------- | ------------------------------------- | ------- |
| `maxSteps`        | Maximum iterations                    | -       |
| `stepSize`        | Integration step size                 | -       |
| `epsilonMove`     | Convergence threshold                 | -       |
| `maxMovePerStep`  | Maximum movement per step             | -       |
| `friction`        | Velocity damping (0-1, 1=no momentum) | 1.0     |
| `relaxationSteps` | Steps to linearly reduce forces       | 0       |
