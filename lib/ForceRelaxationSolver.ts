import { BaseSolver } from "@tscircuit/solver-utils"
import type { GraphicsObject } from "graphics-debug"
export class ForceRelaxationSolver extends BaseSolver {
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

    return graphics
  }
}
