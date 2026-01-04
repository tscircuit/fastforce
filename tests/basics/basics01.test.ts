import { test, expect } from "bun:test"
import inputProblem from "../../fixtures/basics/basics01-input.json"
import { ForceRelaxationSolver } from "../../lib/ForceRelaxationSolver"

test("basics01", () => {
  const solver = new ForceRelaxationSolver(inputProblem)
  solver.solve() // runs .step() repeatedly until solved=true, failed=true, or max iterations reached
  expect(solver.solved, solver.error ?? undefined).toBe(true)
})
