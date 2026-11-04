import { expect, test } from "bun:test"
import { ForceRelaxationSolver } from "../lib/ForceRelaxationSolver"
import type { ForceRelaxationProblem } from "../lib/types"

const makeProblem = (): ForceRelaxationProblem => ({
  bounds: { minX: 0, minY: 0, maxX: 1.5, maxY: 1.5 },
  boundaryPadding: 0,
  layerIds: ["0"],
  entities: {
    points: [
      {
        pointId: "p",
        position: { x: -0.5, y: 0.75 },
        movable: true,
        radius: 0,
        layerIds: ["0"],
      },
    ],
    segments: [],
  },
  interactions: {
    segSegRepel: {
      strength: 0,
      exponentialDecay: 0,
      overlapMultiplier: 1,
      minSeparation: 0,
    },
    pointSegRepel: {
      strength: 0,
      exponentialDecay: 0,
      overlapMultiplier: 1,
      minSeparation: 0,
    },
    boundsKeepIn: { strength: 1, exponentialDecay: 0, overlapMultiplier: 10 },
    fixedLengthCorrection: { strength: 0, exponentialDecay: 0 },
    fixedOrientationCorrection: { strength: 0, exponentialDecay: 0 },
  },
  solve: {
    maxSteps: 300,
    stepSize: 0.1,
    epsilonMove: 0.01,
    friction: 0.1,
  },
})

for (const axis of ["x", "y"] as const) {
  test(`momentum reversal on ${axis} does not stop an active boundary correction`, () => {
    const problem = makeProblem()
    const point = problem.entities.points[0]
    if (axis === "y") point.position = { x: 0.75, y: -0.5 }
    const solver = new ForceRelaxationSolver(problem)

    for (let step = 0; step < 7; step++) solver.step()

    // The point is still outside, where the inward boundary force is active.
    // Its small seventh displacement is a momentum reversal, not equilibrium.
    expect(point.position[axis]).toBeLessThan(0)
    expect(solver.solved).toBe(false)
    expect(solver.progress).toBeLessThan(1)
    solver.step()
    expect(point.position[axis]).toBeGreaterThan(0)
    expect(point.position[axis]).toBeLessThan(1.5)
    expect(solver.solved).toBe(false)
    expect(solver.failed).toBe(false)
  })
}

test("damped momentum still settles after the boundary force stops", () => {
  const problem = makeProblem()
  problem.bounds.maxX = 10
  problem.bounds.maxY = 10
  problem.entities.points[0].position.y = 5
  const solver = new ForceRelaxationSolver(problem)
  solver.solve()
  expect(solver.solved).toBe(true)
  expect(solver.failed).toBe(false)
  expect(problem.entities.points[0].position.x).toBeGreaterThan(0)
  expect(problem.entities.points[0].position.x).toBeLessThan(10)
})

test("balanced boundary forces still converge immediately", () => {
  const problem = makeProblem()
  problem.entities.points[0].position.x = 0.75
  const solver = new ForceRelaxationSolver(problem)
  solver.step()
  expect(solver.solved).toBe(true)
  expect(problem.entities.points[0].position).toEqual({ x: 0.75, y: 0.75 })
})

for (const friction of [undefined, 1]) {
  test(`full friction converges after entering the bounds, friction=${friction}`, () => {
    const problem = makeProblem()
    problem.solve.friction = friction
    const solver = new ForceRelaxationSolver(problem)
    solver.solve()
    expect(solver.solved).toBe(true)
    expect(solver.iterations).toBe(2)
    expect(problem.entities.points[0].position.x).toBeCloseTo(0.4, 10)
  })
}

test("a fixed point does not prevent convergence", () => {
  const problem = makeProblem()
  problem.entities.points[0].movable = false
  const solver = new ForceRelaxationSolver(problem)
  solver.step()
  expect(solver.solved).toBe(true)
  expect(problem.entities.points[0].position).toEqual({ x: -0.5, y: 0.75 })
})

test("momentum corrections respect the movement cap", () => {
  const problem = makeProblem()
  problem.solve.maxMovePerStep = 0.005
  const solver = new ForceRelaxationSolver(problem)
  for (let step = 0; step < 20 && !solver.solved; step++) {
    const previous = { ...problem.entities.points[0].position }
    solver.step()
    const current = problem.entities.points[0].position
    expect(
      Math.hypot(current.x - previous.x, current.y - previous.y),
    ).toBeLessThanOrEqual(0.005 + Number.EPSILON)
    expect(solver.solved).toBe(false)
  }
  expect(solver.failed).toBe(false)
})

test("an unsettled momentum trajectory still respects the iteration limit", () => {
  const problem = makeProblem()
  problem.solve.maxSteps = 3
  const solver = new ForceRelaxationSolver(problem)
  solver.solve()
  expect(solver.solved).toBe(false)
  expect(solver.failed).toBe(true)
  expect(solver.iterations).toBe(3)
})

test("final force relaxation still settles with full damping", () => {
  const problem = makeProblem()
  problem.entities.points[0].position.x = -100
  problem.solve.maxSteps = 8
  problem.solve.relaxationSteps = 3
  const solver = new ForceRelaxationSolver(problem)
  solver.solve()
  expect(solver.solved).toBe(true)
  expect(solver.failed).toBe(false)
  expect(solver.iterations).toBe(8)
})
