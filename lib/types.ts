/**
 * ForceRelaxationProblem
 *
 * A deterministic 2D constraint relaxation system.
 *
 * Core assumptions:
 * - Only points move.
 * - Segments derive geometry from point positions.
 * - Interactions only occur when layer compatibility rules allow them.
 */
export type ForceRelaxationProblem = {
  /** Axis-aligned bounding box */
  bounds: { minX: number; minY: number; maxX: number; maxY: number }

  /**
   * Extra inset applied to bounds when enforcing keep-in constraints.
   */
  boundaryPadding: number

  /**
   * Known layers (e.g. ["0", "1"])
   */
  layerIds: string[]

  entities: {
    points: PointEntity[]
    segments: SegmentEntity[]
  }

  interactions: {
    /**
     * Segment–Segment repulsion (same-layer only).
     */
    segSegRepel: RepulsionInteraction

    /**
     * Point–Segment repulsion (point.layerIds includes segment.layerId).
     */
    pointSegRepel: RepulsionInteraction

    /**
     * Boundary keep-in force for points.
     */
    boundsKeepIn: BoundaryInteraction

    /**
     * Constraint correction to keep selected segments at fixed length.
     */
    fixedLengthCorrection: LengthCorrectionInteraction

    /**
     * Constraint correction to keep selected segments at fixed orientation.
     */
    fixedOrientationCorrection: OrientationCorrectionInteraction
  }

  solve: {
    maxSteps: number
    stepSize: number
    epsilonMove: number
    maxMovePerStep?: number
    /**
     * Friction coefficient for velocity damping (0 to 1).
     * - 1.0 = full friction (no momentum, pure gradient descent)
     * - 0.0 = no friction (full momentum, no damping)
     * Default: 1.0 (no momentum)
     */
    friction?: number
    /**
     * Number of final steps during which forces are linearly relaxed.
     * If > 0, forces are scaled from 1.0 down to 0.0 over the final
     * relaxationSteps iterations, allowing the system to settle smoothly.
     * Default: 0 (no relaxation)
     */
    relaxationSteps?: number
  }
}

export type PointEntity = {
  pointId: string
  position: { x: number; y: number }
  movable: boolean

  /**
   * Radius used for clearance calculations.
   * (0 for dimensionless points)
   */
  radius: number

  /**
   * Layer membership for this point.
   * Example: ["0"] or ["0","1"]
   */
  layerIds: string[]
}

export type SegmentEntity = {
  segmentId: string
  startPointId: string
  endPointId: string

  /**
   * Physical width used for clearance (width/2 on each side).
   */
  width: number

  /**
   * Segment exists on exactly one layer.
   */
  layerId: string

  /**
   * If true, enforce constant length via fixedLengthCorrection.
   */
  fixedLength: boolean

  /**
   * If true, enforce constant orientation via fixedOrientationCorrection.
   */
  fixedOrientation: boolean

  /**
   * Optional color for visualization.
   */
  color?: string
}

/**
 * Generic repulsion interaction.
 *
 * Used when entities must maintain a minimum separation.
 */
export type RepulsionInteraction = {
  /**
   * Base force scale.
   */
  strength: number

  /**
   * Exponential decay factor.
   * Force ∝ exp(-exponentialDecay * distanceToSatisfy)
   */
  exponentialDecay: number

  /**
   * Applied when the constraint is violated
   * (distance < required separation).
   */
  overlapMultiplier: number

  /**
   * Extra clearance beyond geometry-derived separation.
   */
  minSeparation: number
}

/**
 * Boundary keep-in force for points.
 */
export type BoundaryInteraction = {
  strength: number
  exponentialDecay: number

  /**
   * Used when the point lies outside the allowed region.
   */
  overlapMultiplier: number
}

/**
 * Fixed-length constraint correction.
 *
 * Implementor guidance:
 * - Capture restLength at initialization.
 * - Each step:
 *     error = currentLength - restLength
 *     correctionForce ∝ error
 * - Apply forces along the segment direction.
 */
export type LengthCorrectionInteraction = {
  /**
   * Proportional gain for length error.
   */
  strength: number

  /**
   * Optional nonlinear scaling of large errors.
   * Example: force ∝ error * exp(exponentialDecay * |error|)
   * Set to 0 or 1 for linear behavior.
   */
  exponentialDecay: number
}

/**
 * Fixed-orientation constraint correction.
 *
 * Implementor guidance:
 * - Capture restAngle at initialization.
 * - Each step:
 *     angleError = wrapToPi(currentAngle - restAngle)
 * - Apply perpendicular forces to endpoints to reduce angleError.
 */
export type OrientationCorrectionInteraction = {
  /**
   * Proportional gain for angular error.
   */
  strength: number

  /**
   * Optional nonlinear scaling for large angular deviations.
   */
  exponentialDecay: number
}
