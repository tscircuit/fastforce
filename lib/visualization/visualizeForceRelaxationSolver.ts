import type { GraphicsObject } from "graphics-debug"
import type { ForceRelaxationSolver } from "../ForceRelaxationSolver"

export const visualizeForceRelaxationSolver = (
  solver: ForceRelaxationSolver,
): GraphicsObject => {
  const graphics: Required<GraphicsObject> = {
    points: [],
    lines: [],
    rects: [],
    arrows: [],
    circles: [],
    texts: [],
    coordinateSystem: "cartesian",
    title: "force relaxation solver",
  }

  const { bounds, entities } = solver.input
  const { points, segments } = entities

  const pointMap = new Map(points.map((p) => [p.pointId, p]))

  // Draw bounds
  graphics.rects.push({
    center: {
      x: (bounds.minX + bounds.maxX) / 2,
      y: (bounds.minY + bounds.maxY) / 2,
    },
    width: bounds.maxX - bounds.minX,
    height: bounds.maxY - bounds.minY,
    stroke: "gray",
    label: "bounds",
  })

  // Draw segments
  for (const seg of segments) {
    const startPt = pointMap.get(seg.startPointId)
    const endPt = pointMap.get(seg.endPointId)
    if (!startPt || !endPt) continue

    graphics.lines.push({
      points: [
        { x: startPt.position.x, y: startPt.position.y },
        { x: endPt.position.x, y: endPt.position.y },
      ],
      strokeWidth: seg.width,
      strokeColor: seg.color ?? "blue",
      strokeDash: seg.layerId !== "0" ? [4, 2] : undefined,
      label: seg.segmentId,
      layer: seg.layerId,
    })
  }

  // Draw points + radius circles
  for (const pt of points) {
    graphics.points.push({
      x: pt.position.x,
      y: pt.position.y,
      color: pt.movable ? "green" : "red",
      label: pt.pointId,
      layer: pt.layerIds[0],
    })

    if (pt.radius > 0) {
      graphics.circles.push({
        center: { x: pt.position.x, y: pt.position.y },
        radius: pt.radius,
        stroke: pt.movable ? "green" : "red",
        layer: pt.layerIds[0],
      })
    }
  }

  return graphics
}
