import { BaseSolver } from "@tscircuit/solver-utils"
import type { GraphicsObject } from "graphics-debug"
import type { ForceRelaxationProblem } from "./types"
import { visualizeForceRelaxationSolver } from "./visualization/visualizeForceRelaxationSolver"
import { visualizeInputProblem } from "./visualization/visualizeInputProblem"

const EPS = 1e-12
const TAU = Math.PI * 2

// A collision-free 53-bit key encoding for (cx, cy) when each is in [-2^25, 2^25-1]
const CELL_OFFSET = 33554432 // 2^25
const CELL_BASE = 67108864 // 2^26

const clamp01 = (t: number) => (t < 0 ? 0 : t > 1 ? 1 : t)
const safeExp = (x: number) => {
  // Prevent overflow/underflow from destabilizing the solver.
  if (x > 50) return Math.exp(50)
  if (x < -50) return Math.exp(-50)
  return Math.exp(x)
}
const wrapToPi = (a: number) => {
  // Wrap to (-pi, pi]
  a = (a + Math.PI) % TAU
  if (a < 0) a += TAU
  return a - Math.PI
}

type ClosestSegSegResult = {
  s: number
  t: number
  c1x: number
  c1y: number
  c2x: number
  c2y: number
  distSq: number
}

/**
 * Closest points between segments p1->q1 and p2->q2.
 * Adapted from the standard "closestPtSegmentSegment" formulation (Ericson).
 */
const closestPointsOnSegments = (
  p1x: number,
  p1y: number,
  q1x: number,
  q1y: number,
  p2x: number,
  p2y: number,
  q2x: number,
  q2y: number,
): ClosestSegSegResult => {
  const d1x = q1x - p1x
  const d1y = q1y - p1y
  const d2x = q2x - p2x
  const d2y = q2y - p2y
  const rx = p1x - p2x
  const ry = p1y - p2y

  const a = d1x * d1x + d1y * d1y
  const e = d2x * d2x + d2y * d2y
  const f = d2x * rx + d2y * ry

  let s = 0
  let t = 0

  // Both segments degenerate into points
  if (a <= EPS && e <= EPS) {
    const dx = p1x - p2x
    const dy = p1y - p2y
    return {
      s: 0,
      t: 0,
      c1x: p1x,
      c1y: p1y,
      c2x: p2x,
      c2y: p2y,
      distSq: dx * dx + dy * dy,
    }
  }

  if (a <= EPS) {
    // First segment is a point
    s = 0
    t = e <= EPS ? 0 : clamp01(f / e)
  } else {
    const c = d1x * rx + d1y * ry
    if (e <= EPS) {
      // Second segment is a point
      t = 0
      s = clamp01(-c / a)
    } else {
      const b = d1x * d2x + d1y * d2y
      const denom = a * e - b * b

      if (Math.abs(denom) > EPS) {
        s = clamp01((b * f - c * e) / denom)
      } else {
        // Parallel case
        s = 0
      }

      const tnom = b * s + f
      if (tnom < 0) {
        t = 0
        s = clamp01(-c / a)
      } else if (tnom > e) {
        t = 1
        s = clamp01((b - c) / a)
      } else {
        t = tnom / e
      }
    }
  }

  const c1x = p1x + d1x * s
  const c1y = p1y + d1y * s
  const c2x = p2x + d2x * t
  const c2y = p2y + d2y * t
  const dx = c1x - c2x
  const dy = c1y - c2y
  return { s, t, c1x, c1y, c2x, c2y, distSq: dx * dx + dy * dy }
}

type ClosestPtSegResult = {
  t: number
  cx: number
  cy: number
  dx: number
  dy: number
  distSq: number
}

const closestPointOnSegment = (
  px: number,
  py: number,
  ax: number,
  ay: number,
  bx: number,
  by: number,
): ClosestPtSegResult => {
  const vx = bx - ax
  const vy = by - ay
  const wx = px - ax
  const wy = py - ay

  const vv = vx * vx + vy * vy
  let t = 0
  if (vv > EPS) t = clamp01((wx * vx + wy * vy) / vv)

  const cx = ax + t * vx
  const cy = ay + t * vy
  const dx = px - cx
  const dy = py - cy
  return { t, cx, cy, dx, dy, distSq: dx * dx + dy * dy }
}

export class ForceRelaxationSolver extends BaseSolver {
  constructor(public input: ForceRelaxationProblem) {
    super()
    this.MAX_ITERATIONS = input.solve.maxSteps
  }

  // ---- cached problem data (initialized lazily) ----
  private _initialized = false

  private pointsCount = 0
  private segmentsCount = 0

  private pointsRef!: ForceRelaxationProblem["entities"]["points"]
  private segmentsRef!: ForceRelaxationProblem["entities"]["segments"]

  private pointIndexById = new Map<string, number>()

  // layer handling
  private useLayerBitmask = true
  private layerIndexById = new Map<string, number>()
  private pointLayerMask32!: Uint32Array
  private pointLayerSets?: Array<Set<string>>
  private segLayerIdx!: Int32Array

  // points arrays
  private px!: Float64Array
  private py!: Float64Array
  private pr!: Float64Array
  private movable!: Uint8Array

  // segments arrays
  private segA!: Int32Array
  private segB!: Int32Array
  private segHalfWidth!: Float64Array
  private segFixedLen!: Uint8Array
  private segFixedOri!: Uint8Array
  private restLen!: Float64Array
  private restAngle!: Float64Array

  // forces
  private fx!: Float64Array
  private fy!: Float64Array

  // velocities (for momentum)
  private vx!: Float64Array
  private vy!: Float64Array

  // broadphase grid (segments)
  private gridOriginX = 0
  private gridOriginY = 0
  private cellSize = 1
  private invCellSize = 1
  private globalInfluence = 1

  private segGrid = new Map<number, number[]>() // cellKey -> segment indices
  private segCellMinX!: Int32Array
  private segCellMaxX!: Int32Array
  private segCellMinY!: Int32Array
  private segCellMaxY!: Int32Array

  // visited marks to dedupe candidates from multiple cells
  private segVisitedMark!: Int32Array

  private _cellKey(cx: number, cy: number): number {
    return (cx + CELL_OFFSET) * CELL_BASE + (cy + CELL_OFFSET)
  }

  private _ensureInitialized(): void {
    if (this._initialized) return
    this._initialized = true

    const { points, segments } = this.input.entities
    this.pointsRef = points
    this.segmentsRef = segments
    this.pointsCount = points.length
    this.segmentsCount = segments.length

    // Layer indexing
    this.layerIndexById.clear()
    for (let i = 0; i < this.input.layerIds.length; i++) {
      this.layerIndexById.set(this.input.layerIds[i], i)
    }
    // 32-bit bitmask only safely supports up to 31 distinct bits via JS bitwise ops;
    // we'll allow up to 30 bits to keep everything unsigned and predictable.
    this.useLayerBitmask = this.input.layerIds.length <= 30

    // Point ID -> index
    this.pointIndexById.clear()
    for (let i = 0; i < this.pointsCount; i++) {
      this.pointIndexById.set(points[i].pointId, i)
    }

    // Allocate typed arrays
    this.px = new Float64Array(this.pointsCount)
    this.py = new Float64Array(this.pointsCount)
    this.pr = new Float64Array(this.pointsCount)
    this.movable = new Uint8Array(this.pointsCount)
    this.fx = new Float64Array(this.pointsCount)
    this.fy = new Float64Array(this.pointsCount)
    this.vx = new Float64Array(this.pointsCount)
    this.vy = new Float64Array(this.pointsCount)

    if (this.useLayerBitmask) {
      this.pointLayerMask32 = new Uint32Array(this.pointsCount)
    } else {
      this.pointLayerMask32 = new Uint32Array(0)
      this.pointLayerSets = new Array(this.pointsCount)
    }

    for (let i = 0; i < this.pointsCount; i++) {
      const p = points[i]
      this.px[i] = p.position.x
      this.py[i] = p.position.y
      this.pr[i] = p.radius
      this.movable[i] = p.movable ? 1 : 0

      if (this.useLayerBitmask) {
        let mask = 0
        for (let k = 0; k < p.layerIds.length; k++) {
          const li = this.layerIndexById.get(p.layerIds[k])
          if (li === undefined) continue
          mask |= 1 << li
        }
        this.pointLayerMask32[i] = mask >>> 0
      } else {
        this.pointLayerSets![i] = new Set(p.layerIds)
      }
    }

    this.segA = new Int32Array(this.segmentsCount)
    this.segB = new Int32Array(this.segmentsCount)
    this.segHalfWidth = new Float64Array(this.segmentsCount)
    this.segFixedLen = new Uint8Array(this.segmentsCount)
    this.segFixedOri = new Uint8Array(this.segmentsCount)
    this.restLen = new Float64Array(this.segmentsCount)
    this.restAngle = new Float64Array(this.segmentsCount)
    this.segLayerIdx = new Int32Array(this.segmentsCount)

    this.segCellMinX = new Int32Array(this.segmentsCount)
    this.segCellMaxX = new Int32Array(this.segmentsCount)
    this.segCellMinY = new Int32Array(this.segmentsCount)
    this.segCellMaxY = new Int32Array(this.segmentsCount)

    this.segVisitedMark = new Int32Array(this.segmentsCount)

    for (let i = 0; i < this.segmentsCount; i++) {
      const s = segments[i]
      const a = this.pointIndexById.get(s.startPointId)
      const b = this.pointIndexById.get(s.endPointId)
      this.segA[i] = a === undefined ? -1 : a
      this.segB[i] = b === undefined ? -1 : b
      this.segHalfWidth[i] = s.width * 0.5
      this.segFixedLen[i] = s.fixedLength ? 1 : 0
      this.segFixedOri[i] = s.fixedOrientation ? 1 : 0

      const li = this.layerIndexById.get(s.layerId)
      this.segLayerIdx[i] = li === undefined ? -1 : li

      // Capture rest length/orientation at init
      if (a !== undefined && b !== undefined) {
        const dx = this.px[b] - this.px[a]
        const dy = this.py[b] - this.py[a]
        const len = Math.sqrt(dx * dx + dy * dy)
        this.restLen[i] = len
        this.restAngle[i] = Math.atan2(dy, dx)
      } else {
        this.restLen[i] = 0
        this.restAngle[i] = 0
      }
    }

    // Compute global influence distance (used for grid cell size)
    let maxPointR = 0
    for (let i = 0; i < this.pointsCount; i++)
      maxPointR = Math.max(maxPointR, this.pr[i])

    let maxSegHalf = 0
    for (let i = 0; i < this.segmentsCount; i++)
      maxSegHalf = Math.max(maxSegHalf, this.segHalfWidth[i])

    const maxMinSep = Math.max(
      this.input.interactions.segSegRepel.minSeparation,
      this.input.interactions.pointSegRepel.minSeparation,
    )

    const baseSep = Math.max(maxPointR + maxSegHalf, 2 * maxSegHalf) + maxMinSep

    const decays = [
      this.input.interactions.segSegRepel.exponentialDecay,
      this.input.interactions.pointSegRepel.exponentialDecay,
    ].filter((d) => d > 0)

    const minPositiveDecay = decays.length > 0 ? Math.min(...decays) : 0
    const cutoffGap =
      minPositiveDecay > 0
        ? 6.907755278982137 / minPositiveDecay
        : baseSep * 10 + 1

    this.globalInfluence = baseSep + cutoffGap
    this.cellSize = Math.max(this.globalInfluence, 1e-3)
    this.invCellSize = 1 / this.cellSize

    // Grid origin anchored at bounds min for stable-ish cell coordinates
    this.gridOriginX = this.input.bounds.minX
    this.gridOriginY = this.input.bounds.minY
  }

  private _pointOnLayer(
    pointIndex: number,
    segLayerIdx: number,
    segLayerId: string,
  ): boolean {
    if (segLayerIdx < 0) return false
    if (this.useLayerBitmask) {
      const bit = 1 << segLayerIdx
      return (this.pointLayerMask32[pointIndex] & (bit >>> 0)) !== 0
    }
    return this.pointLayerSets![pointIndex].has(segLayerId)
  }

  private _repulsionMagnitude(
    gap: number,
    strength: number,
    decay: number,
    overlapMultiplier: number,
  ): number {
    // gap = actualDistance - requiredSeparation
    // If gap < 0 => violation (overlap), magnify.
    const mult = gap < 0 ? overlapMultiplier : 1
    if (decay === 0) return strength * mult
    // Force ∝ exp(-decay * gap). If gap negative, exp increases.
    return strength * mult * safeExp(-decay * gap)
  }

  private _rebuildSegmentGrid(): void {
    this.segGrid.clear()

    const infl = this.globalInfluence
    const ox = this.gridOriginX
    const oy = this.gridOriginY
    const inv = this.invCellSize

    for (let i = 0; i < this.segmentsCount; i++) {
      const a = this.segA[i]
      const b = this.segB[i]
      if (a < 0 || b < 0) {
        this.segCellMinX[i] = 0
        this.segCellMaxX[i] = -1
        this.segCellMinY[i] = 0
        this.segCellMaxY[i] = -1
        continue
      }

      const ax = this.px[a]
      const ay = this.py[a]
      const bx = this.px[b]
      const by = this.py[b]

      // Expand AABB by influence for broadphase
      const minx = (ax < bx ? ax : bx) - infl
      const maxx = (ax > bx ? ax : bx) + infl
      const miny = (ay < by ? ay : by) - infl
      const maxy = (ay > by ? ay : by) + infl

      const cminx = Math.floor((minx - ox) * inv)
      const cmaxx = Math.floor((maxx - ox) * inv)
      const cminy = Math.floor((miny - oy) * inv)
      const cmaxy = Math.floor((maxy - oy) * inv)

      this.segCellMinX[i] = cminx
      this.segCellMaxX[i] = cmaxx
      this.segCellMinY[i] = cminy
      this.segCellMaxY[i] = cmaxy

      for (let cx = cminx; cx <= cmaxx; cx++) {
        for (let cy = cminy; cy <= cmaxy; cy++) {
          const key = this._cellKey(cx, cy)
          const arr = this.segGrid.get(key)
          if (arr) arr.push(i)
          else this.segGrid.set(key, [i])
        }
      }
    }
  }

  override _step(): void {
    this._ensureInitialized()
    if (this.solved) return

    // Clear forces
    this.fx.fill(0)
    this.fy.fill(0)

    // Rebuild broadphase grid for segments
    this._rebuildSegmentGrid()

    const {
      segSegRepel,
      pointSegRepel,
      boundsKeepIn,
      fixedLengthCorrection,
      fixedOrientationCorrection,
    } = this.input.interactions

    // ---- Segment–Segment repulsion (same-layer only) ----
    for (let i = 0; i < this.segmentsCount; i++) {
      const a0 = this.segA[i]
      const a1 = this.segB[i]
      if (a0 < 0 || a1 < 0) continue

      const layerI = this.segLayerIdx[i]
      if (layerI < 0) continue

      // If both endpoints are fixed, seg can't move -> skip pairwise work
      if ((this.movable[a0] | this.movable[a1]) === 0) continue

      const cminx = this.segCellMinX[i]
      const cmaxx = this.segCellMaxX[i]
      const cminy = this.segCellMinY[i]
      const cmaxy = this.segCellMaxY[i]
      if (cmaxx < cminx || cmaxy < cminy) continue

      const mark = i + 1

      const p1x = this.px[a0]
      const p1y = this.py[a0]
      const q1x = this.px[a1]
      const q1y = this.py[a1]

      for (let cx = cminx; cx <= cmaxx; cx++) {
        for (let cy = cminy; cy <= cmaxy; cy++) {
          const key = this._cellKey(cx, cy)
          const list = this.segGrid.get(key)
          if (!list) continue

          for (let idx = 0; idx < list.length; idx++) {
            const j = list[idx]
            if (j <= i) continue
            if (this.segVisitedMark[j] === mark) continue
            this.segVisitedMark[j] = mark

            const b0 = this.segA[j]
            const b1 = this.segB[j]
            if (b0 < 0 || b1 < 0) continue

            // same layer only
            if (this.segLayerIdx[j] !== layerI) continue

            // Skip segments that share an endpoint (common in polylines)
            if (a0 === b0 || a0 === b1 || a1 === b0 || a1 === b1) continue

            // If neither segment's endpoints can move, no need to compute
            if (
              ((this.movable[a0] |
                this.movable[a1] |
                this.movable[b0] |
                this.movable[b1]) as number) === 0
            )
              continue

            const p2x = this.px[b0]
            const p2y = this.py[b0]
            const q2x = this.px[b1]
            const q2y = this.py[b1]

            const res = closestPointsOnSegments(
              p1x,
              p1y,
              q1x,
              q1y,
              p2x,
              p2y,
              q2x,
              q2y,
            )
            const dist = Math.sqrt(res.distSq)

            const required =
              this.segHalfWidth[i] +
              this.segHalfWidth[j] +
              segSegRepel.minSeparation
            const gap = dist - required

            const mag = this._repulsionMagnitude(
              gap,
              segSegRepel.strength,
              segSegRepel.exponentialDecay,
              segSegRepel.overlapMultiplier,
            )
            if (!Number.isFinite(mag) || mag === 0) continue

            // Direction from segment j to segment i
            let dx = res.c1x - res.c2x
            let dy = res.c1y - res.c2y

            if (res.distSq <= EPS) {
              // Overlapping/intersecting: pick a deterministic direction via midpoints
              const m1x = (p1x + q1x) * 0.5
              const m1y = (p1y + q1y) * 0.5
              const m2x = (p2x + q2x) * 0.5
              const m2y = (p2y + q2y) * 0.5
              dx = m1x - m2x
              dy = m1y - m2y

              const dmid2 = dx * dx + dy * dy
              if (dmid2 <= EPS) {
                // Still ambiguous: use perpendicular to segment i
                const sx = q1x - p1x
                const sy = q1y - p1y
                dx = -sy
                dy = sx
                if (dx * dx + dy * dy <= EPS) {
                  dx = 1
                  dy = 0
                }
              }
            }

            const invd = 1 / Math.sqrt(dx * dx + dy * dy + EPS)
            const ux = dx * invd
            const uy = dy * invd

            const fx = ux * mag
            const fy = uy * mag

            // Distribute forces to endpoints based on closest-point parameters
            const s = res.s
            const t = res.t

            const wA0 = 1 - s
            const wA1 = s
            const wB0 = 1 - t
            const wB1 = t

            // Segment i endpoints get +F
            this.fx[a0] += fx * wA0
            this.fy[a0] += fy * wA0
            this.fx[a1] += fx * wA1
            this.fy[a1] += fy * wA1

            // Segment j endpoints get -F
            this.fx[b0] -= fx * wB0
            this.fy[b0] -= fy * wB0
            this.fx[b1] -= fx * wB1
            this.fy[b1] -= fy * wB1
          }
        }
      }
    }

    // ---- Point–Segment repulsion (layer compatible) ----
    {
      const ox = this.gridOriginX
      const oy = this.gridOriginY
      const inv = this.invCellSize

      for (let pi = 0; pi < this.pointsCount; pi++) {
        const px = this.px[pi]
        const py = this.py[pi]

        const cx = Math.floor((px - ox) * inv)
        const cy = Math.floor((py - oy) * inv)

        // Neighbor query (3x3) for safety around cell boundaries
        const mark = pi + 1

        for (let dxCell = -1; dxCell <= 1; dxCell++) {
          for (let dyCell = -1; dyCell <= 1; dyCell++) {
            const key = this._cellKey(cx + dxCell, cy + dyCell)
            const list = this.segGrid.get(key)
            if (!list) continue

            for (let li = 0; li < list.length; li++) {
              const si = list[li]
              if (this.segVisitedMark[si] === mark) continue
              this.segVisitedMark[si] = mark

              const a = this.segA[si]
              const b = this.segB[si]
              if (a < 0 || b < 0) continue

              // skip self-attachment (point is endpoint)
              if (pi === a || pi === b) continue

              const seg = this.segmentsRef[si]
              if (!this._pointOnLayer(pi, this.segLayerIdx[si], seg.layerId))
                continue

              // If neither point nor segment endpoints can move, skip
              if (
                ((this.movable[pi] |
                  this.movable[a] |
                  this.movable[b]) as number) === 0
              )
                continue

              const ax = this.px[a]
              const ay = this.py[a]
              const bx = this.px[b]
              const by = this.py[b]

              const res = closestPointOnSegment(px, py, ax, ay, bx, by)
              const dist = Math.sqrt(res.distSq)

              const required =
                this.pr[pi] +
                this.segHalfWidth[si] +
                pointSegRepel.minSeparation
              const gap = dist - required

              const mag = this._repulsionMagnitude(
                gap,
                pointSegRepel.strength,
                pointSegRepel.exponentialDecay,
                pointSegRepel.overlapMultiplier,
              )
              if (!Number.isFinite(mag) || mag === 0) continue

              let ux = 0
              let uy = 0
              if (res.distSq > EPS) {
                const invd = 1 / dist
                ux = res.dx * invd
                uy = res.dy * invd
              } else {
                // point lies on the segment line: choose deterministic perpendicular
                const sx = bx - ax
                const sy = by - ay
                const nn = Math.sqrt(sx * sx + sy * sy)
                if (nn > EPS) {
                  ux = -sy / nn
                  uy = sx / nn
                } else {
                  ux = 1
                  uy = 0
                }
              }

              const fx = ux * mag
              const fy = uy * mag

              // Point gets +F (away from segment)
              this.fx[pi] += fx
              this.fy[pi] += fy

              // Segment gets -F distributed by t
              const t = res.t
              const w0 = 1 - t
              const w1 = t
              this.fx[a] -= fx * w0
              this.fy[a] -= fy * w0
              this.fx[b] -= fx * w1
              this.fy[b] -= fy * w1
            }
          }
        }
      }
    }

    // ---- Bounds keep-in (points) ----
    {
      const { bounds, boundaryPadding } = this.input
      const minX = bounds.minX + boundaryPadding
      const minY = bounds.minY + boundaryPadding
      const maxX = bounds.maxX - boundaryPadding
      const maxY = bounds.maxY - boundaryPadding

      for (let i = 0; i < this.pointsCount; i++) {
        // Even for fixed points, apply force (it will be ignored on movement but affects connected segments)
        const x = this.px[i]
        const y = this.py[i]
        const r = this.pr[i]

        // Distance from the allowed region edges (positive inside, negative outside)
        const gapL = x - (minX + r)
        const gapR = maxX - r - x
        const gapB = y - (minY + r)
        const gapT = maxY - r - y

        const k = boundsKeepIn.exponentialDecay
        const s = boundsKeepIn.strength
        const om = boundsKeepIn.overlapMultiplier

        // Left edge pushes +X
        {
          const mag =
            (gapL < 0 ? om : 1) * s * (k === 0 ? 1 : safeExp(-k * gapL))
          this.fx[i] += mag
        }
        // Right edge pushes -X
        {
          const mag =
            (gapR < 0 ? om : 1) * s * (k === 0 ? 1 : safeExp(-k * gapR))
          this.fx[i] -= mag
        }
        // Bottom edge pushes +Y
        {
          const mag =
            (gapB < 0 ? om : 1) * s * (k === 0 ? 1 : safeExp(-k * gapB))
          this.fy[i] += mag
        }
        // Top edge pushes -Y
        {
          const mag =
            (gapT < 0 ? om : 1) * s * (k === 0 ? 1 : safeExp(-k * gapT))
          this.fy[i] -= mag
        }
      }
    }

    // ---- Fixed-length correction (segments) ----
    {
      const k = fixedLengthCorrection.exponentialDecay
      const s = fixedLengthCorrection.strength

      for (let i = 0; i < this.segmentsCount; i++) {
        if (this.segFixedLen[i] === 0) continue

        const a = this.segA[i]
        const b = this.segB[i]
        if (a < 0 || b < 0) continue
        if ((this.movable[a] | this.movable[b]) === 0) continue

        const dx = this.px[b] - this.px[a]
        const dy = this.py[b] - this.py[a]
        const len = Math.sqrt(dx * dx + dy * dy)
        if (len <= EPS) continue

        const err = len - this.restLen[i]
        if (Math.abs(err) <= 0) continue

        const gain = k === 0 ? 1 : safeExp(k * Math.abs(err))
        const mag = s * err * gain

        const ux = dx / len
        const uy = dy / len

        // Pull together if too long; push apart if too short (sign via err)
        this.fx[a] += ux * mag
        this.fy[a] += uy * mag
        this.fx[b] -= ux * mag
        this.fy[b] -= uy * mag
      }
    }

    // ---- Fixed-orientation correction (segments) ----
    {
      const k = fixedOrientationCorrection.exponentialDecay
      const s = fixedOrientationCorrection.strength

      for (let i = 0; i < this.segmentsCount; i++) {
        if (this.segFixedOri[i] === 0) continue

        const a = this.segA[i]
        const b = this.segB[i]
        if (a < 0 || b < 0) continue
        if ((this.movable[a] | this.movable[b]) === 0) continue

        const dx = this.px[b] - this.px[a]
        const dy = this.py[b] - this.py[a]
        const len = Math.sqrt(dx * dx + dy * dy)
        if (len <= EPS) continue

        const ang = Math.atan2(dy, dx)
        const err = wrapToPi(ang - this.restAngle[i])

        const gain = k === 0 ? 1 : safeExp(k * Math.abs(err))
        // Scale by length so longer segments get proportionate correction
        const mag = s * err * len * gain

        const ux = dx / len
        const uy = dy / len
        // Left normal (CCW perpendicular)
        const nx = -uy
        const ny = ux

        // Apply a couple: A += n*mag, B -= n*mag.
        // This reduces angle error for the chosen sign convention above.
        this.fx[a] += nx * mag
        this.fy[a] += ny * mag
        this.fx[b] -= nx * mag
        this.fy[b] -= ny * mag
      }
    }

    // ---- Linear force relaxation in final steps ----
    const relaxationSteps = this.input.solve.relaxationSteps ?? 0
    let relaxationScale = 1.0
    if (relaxationSteps > 0) {
      const remainingSteps = this.MAX_ITERATIONS - this.iterations
      if (remainingSteps <= relaxationSteps) {
        relaxationScale = remainingSteps / relaxationSteps
        for (let i = 0; i < this.pointsCount; i++) {
          this.fx[i] *= relaxationScale
          this.fy[i] *= relaxationScale
        }
      }
    }

    // ---- Integrate movement with momentum ----
    const stepSize = this.input.solve.stepSize
    const epsMove = this.input.solve.epsilonMove
    const maxMovePerStep = this.input.solve.maxMovePerStep
    const baseFriction = this.input.solve.friction ?? 1.0
    // During relaxation, friction increases towards 1.0 (full damping)
    const friction =
      relaxationScale < 1.0
        ? 1.0 - (1.0 - baseFriction) * relaxationScale
        : baseFriction
    const momentum = 1.0 - friction

    let maxMove = 0

    for (let i = 0; i < this.pointsCount; i++) {
      if (this.movable[i] === 0) continue

      // Update velocity: v = v * momentum + force * stepSize
      let vx = this.vx[i] * momentum + this.fx[i] * stepSize
      let vy = this.vy[i] * momentum + this.fy[i] * stepSize

      const m2 = vx * vx + vy * vy
      if (m2 <= EPS) {
        this.vx[i] = 0
        this.vy[i] = 0
        continue
      }

      let m = Math.sqrt(m2)

      if (
        maxMovePerStep !== undefined &&
        maxMovePerStep > 0 &&
        m > maxMovePerStep
      ) {
        const s = maxMovePerStep / m
        vx *= s
        vy *= s
        m = maxMovePerStep
      }

      // Store updated velocity
      this.vx[i] = vx
      this.vy[i] = vy

      // Update position
      this.px[i] += vx
      this.py[i] += vy

      // write back to the original objects (so visualization / consumers see updates)
      const p = this.pointsRef[i]
      p.position.x = this.px[i]
      p.position.y = this.py[i]

      if (m > maxMove) maxMove = m
    }
    // Progress heuristic (shrinks as movement shrinks)
    // BaseSolver typically exposes `progress`, so set it if present.
    ;(this as any).progress = Math.min(1, epsMove / Math.max(epsMove, maxMove))

    if (maxMove <= epsMove) {
      this.solved = true
    }
  }

  override visualize(): GraphicsObject {
    // iteration 0: show initial (input) state
    if (this.iterations === 0) {
      return visualizeInputProblem(this.input)
    }

    // Ensure arrays exist even if visualize is called before stepping
    this._ensureInitialized()

    return visualizeForceRelaxationSolver(this)
  }
}
