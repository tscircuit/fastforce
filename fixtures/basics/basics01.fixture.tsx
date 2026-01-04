import { GenericSolverDebugger } from "@tscircuit/solver-utils/react"
import { ForceRelaxationSolver } from "../../lib/ForceRelaxationSolver"
import inputProblem from "./basics01-input.json"
import type { ForceRelaxationProblem } from "../../lib/types"

export default () => (
  <GenericSolverDebugger
    createSolver={() =>
      new ForceRelaxationSolver(inputProblem as ForceRelaxationProblem)
    }
  />
)
