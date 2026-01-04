import { BaseSolver } from "@tscircuit/solver-utils"
import type { GraphicsObject } from "graphics-debug"
import type { ForceRelaxationProblem } from "./types"

export class ForceRelaxationSolver extends BaseSolver {
  constructor(public input: ForceRelaxationProblem) {
    super()
    this.MAX_ITERATIONS = input.solve.maxSteps
  }

  override _step(): void {
    // TODO perform forces
  }

  override visualize(): GraphicsObject {
    const graphics = {
      points: [],
      lines: [],
      rects: [],
      arrows: [],
      circles: [],
      texts: [],
      coordinateSystem: "cartesian",
      title: "force relaxation solver",
    } as Required<GraphicsObject>

    // TODO Draw points and segments

    return graphics
  }
}
