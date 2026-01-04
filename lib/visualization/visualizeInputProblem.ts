import type { GraphicsObject } from "graphics-debug"
import type { ForceRelaxationProblem } from "../types"

export const visualizeInputProblem = (
  problem: ForceRelaxationProblem,
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

  const { bounds, boundaryPadding, entities, interactions } = problem
  const { points, segments } = entities
  const { segSegRepel, pointSegRepel } = interactions

  // Build point lookup for segment endpoints
  const pointMap = new Map(points.map((p) => [p.pointId, p]))

  // Draw bounds (outer)
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

  // Draw effective bounds after boundary padding
  if (boundaryPadding > 0) {
    graphics.rects.push({
      center: {
        x: (bounds.minX + bounds.maxX) / 2,
        y: (bounds.minY + bounds.maxY) / 2,
      },
      width: bounds.maxX - bounds.minX - boundaryPadding * 2,
      height: bounds.maxY - bounds.minY - boundaryPadding * 2,
      stroke: "rgba(128, 128, 128, 0.5)",
      strokeDash: [4, 4],
      label: "effective bounds",
    })
  }

  // Draw segments as lines
  for (const seg of segments) {
    const startPt = pointMap.get(seg.startPointId)
    const endPt = pointMap.get(seg.endPointId)
    if (!startPt || !endPt) continue

    // Draw segment with minSeparation buffer (wider, semi-transparent)
    const bufferWidth = seg.width + segSegRepel.minSeparation * 2
    graphics.lines.push({
      points: [
        { x: startPt.position.x, y: startPt.position.y },
        { x: endPt.position.x, y: endPt.position.y },
      ],
      strokeWidth: bufferWidth,
      strokeColor: "rgba(200, 200, 200, 0.3)",
      layer: seg.layerId,
    })

    // Draw actual segment
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

  // Draw points
  for (const pt of points) {
    graphics.points.push({
      x: pt.position.x,
      y: pt.position.y,
      color: pt.movable ? "green" : "red",
      label: pt.pointId,
      layer: pt.layerIds[0],
    })

    // Draw radius circle if point has one
    if (pt.radius > 0) {
      graphics.circles.push({
        center: { x: pt.position.x, y: pt.position.y },
        radius: pt.radius,
        stroke: pt.movable ? "green" : "red",
        layer: pt.layerIds[0],
      })
    }

    // Draw point-segment minSeparation circle (radius + minSeparation)
    const separationRadius = pt.radius + pointSegRepel.minSeparation
    if (separationRadius > 0) {
      graphics.circles.push({
        center: { x: pt.position.x, y: pt.position.y },
        radius: separationRadius,
        stroke: "rgba(128, 128, 128, 0.3)",
        strokeDash: [2, 2],
        layer: pt.layerIds[0],
      })
    }
  }

  return graphics
}
