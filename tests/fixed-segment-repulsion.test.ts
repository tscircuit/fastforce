import { expect, test } from "bun:test"
import { ForceRelaxationSolver } from "../lib/ForceRelaxationSolver"
import type { ForceRelaxationProblem } from "../lib/types"

const makeProblem = (): ForceRelaxationProblem => ({
  bounds: { minX: -10, minY: -10, maxX: 10, maxY: 10 },
  boundaryPadding: 0,
  layerIds: ["0", "1"],
  entities: {
    points: [
      {
        pointId: "a",
        position: { x: -1, y: 0 },
        movable: false,
        radius: 0,
        layerIds: ["0"],
      },
      {
        pointId: "b",
        position: { x: 1, y: 0 },
        movable: false,
        radius: 0,
        layerIds: ["0"],
      },
      {
        pointId: "c",
        position: { x: -1, y: 1 },
        movable: true,
        radius: 0,
        layerIds: ["0"],
      },
      {
        pointId: "d",
        position: { x: 1, y: 1 },
        movable: true,
        radius: 0,
        layerIds: ["0"],
      },
    ],
    segments: [
      {
        segmentId: "fixed",
        startPointId: "a",
        endPointId: "b",
        width: 0,
        layerId: "0",
        fixedLength: false,
        fixedOrientation: false,
      },
      {
        segmentId: "moving",
        startPointId: "c",
        endPointId: "d",
        width: 0,
        layerId: "0",
        fixedLength: false,
        fixedOrientation: false,
      },
    ],
  },
  interactions: {
    segSegRepel: {
      strength: 1,
      exponentialDecay: 0,
      overlapMultiplier: 1,
      minSeparation: 2,
    },
    pointSegRepel: {
      strength: 0,
      exponentialDecay: 0,
      overlapMultiplier: 1,
      minSeparation: 0,
    },
    boundsKeepIn: { strength: 0, exponentialDecay: 0, overlapMultiplier: 1 },
    fixedLengthCorrection: { strength: 0, exponentialDecay: 0 },
    fixedOrientationCorrection: { strength: 0, exponentialDecay: 0 },
  },
  solve: { maxSteps: 100, stepSize: 0.1, epsilonMove: 1e-9 },
})

for (const fixedFirst of [true, false]) {
  test(`segment repulsion retains point-segment forces, fixedFirst=${fixedFirst}`, () => {
    const problem = makeProblem()
    problem.interactions.pointSegRepel.strength = 1
    if (!fixedFirst) problem.entities.segments.reverse()
    new ForceRelaxationSolver(problem).step()

    // c receives the segment force and both point-segment reactions.
    // d receives the two point-segment reactions, but no segment force.
    expect(problem.entities.points[2].position.y).toBeCloseTo(1.3, 10)
    expect(problem.entities.points[3].position.y).toBeCloseTo(1.2, 10)
    expect(problem.entities.points[0].position).toEqual({ x: -1, y: 0 })
    expect(problem.entities.points[1].position).toEqual({ x: 1, y: 0 })
  })

  test(`fixed obstacle repels a moving segment, fixedFirst=${fixedFirst}`, () => {
    const problem = makeProblem()
    if (!fixedFirst) problem.entities.segments.reverse()
    const solver = new ForceRelaxationSolver(problem)
    solver.step()

    const [a, b, c, d] = problem.entities.points
    expect(a.position).toEqual({ x: -1, y: 0 })
    expect(b.position).toEqual({ x: 1, y: 0 })
    // Unit upward repulsion at the nearest endpoints, with a 0.1 step.
    expect(c.position.x).toBe(-1)
    expect(c.position.y).toBeCloseTo(1.1, 10)
    expect(d.position).toEqual({ x: 1, y: 1 })
    expect(solver.solved).toBe(false)
  })

  test(`different layers do not repel, fixedFirst=${fixedFirst}`, () => {
    const problem = makeProblem()
    problem.entities.segments[1].layerId = "1"
    if (!fixedFirst) problem.entities.segments.reverse()
    const before = structuredClone(problem.entities.points)
    new ForceRelaxationSolver(problem).step()
    expect(problem.entities.points).toEqual(before)
  })
}

test("fixed obstacle repels a segment with only one movable endpoint", () => {
  const problem = makeProblem()
  problem.entities.points[3].movable = false
  new ForceRelaxationSolver(problem).step()
  expect(problem.entities.points[2].position.y).toBeCloseTo(1.1, 10)
  expect(problem.entities.points[3].position).toEqual({ x: 1, y: 1 })
})

test("two fixed segments retain their positions", () => {
  const problem = makeProblem()
  for (const point of problem.entities.points) point.movable = false
  const before = structuredClone(problem.entities.points)
  new ForceRelaxationSolver(problem).step()
  expect(problem.entities.points).toEqual(before)
})

test("segments sharing an endpoint do not repel each other", () => {
  const problem = makeProblem()
  problem.entities.segments[1].startPointId = "b"
  const before = structuredClone(problem.entities.points)
  new ForceRelaxationSolver(problem).step()
  expect(problem.entities.points).toEqual(before)
})

test("earlier queries do not suppress segment repulsion on the next step", () => {
  const problem = makeProblem()
  problem.bounds.maxX = 1100
  // Three distant segments precede the interacting pair. Their points cannot
  // visit the nearby segments, so the previous step's query marks survive.
  for (let i = 0; i < 3; i++) {
    const startPointId = `far-${i}-start`
    const endPointId = `far-${i}-end`
    problem.entities.points.push(
      {
        pointId: startPointId,
        position: { x: 1000, y: i },
        movable: false,
        radius: 0,
        layerIds: ["0"],
      },
      {
        pointId: endPointId,
        position: { x: 1001, y: i },
        movable: false,
        radius: 0,
        layerIds: ["0"],
      },
    )
    problem.entities.segments.unshift({
      segmentId: `far-${i}`,
      startPointId,
      endPointId,
      width: 0,
      layerId: "0",
      fixedLength: false,
      fixedOrientation: false,
    })
  }
  const solver = new ForceRelaxationSolver(problem)
  solver.step()
  expect(problem.entities.points[2].position.y).toBeCloseTo(1.1, 10)
  solver.step()
  // The second endpoint is now closest and must still be repelled.
  expect(problem.entities.points[3].position.y).toBeCloseTo(1.1, 10)
  expect(solver.solved).toBe(false)
})
