package xyz.thewind.kbox2d.collision

import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2

class WorldManifold {
    val normal: Vec2 = Vec2()
    val points: Array<Vec2> = Array(Settings.maxManifoldPoints) { Vec2() }
    val separations: FloatArray = FloatArray(Settings.maxManifoldPoints)

    private val pool3 = Vec2()
    private val pool4 = Vec2()

    fun initialize(manifold: Manifold, xfA: Transform, radiusA: Float, xfB: Transform, radiusB: Float) {
        if (manifold.pointCount == 0) return

        when (manifold.type) {
            Manifold.ManifoldType.CIRCLES -> {
                val pointA = pool3
                val pointB = pool4
                normal.x = 1f
                normal.y = 0f
                val v = manifold.localPoint
                pointA.x = (xfA.q.c * v.x - xfA.q.s * v.y) + xfA.p.x
                pointA.y = (xfA.q.s * v.x + xfA.q.c * v.y) + xfA.p.y
                val mp0p = manifold.points[0].localPoint
                pointB.x = (xfB.q.c * mp0p.x - xfB.q.s * mp0p.y) + xfB.p.x
                pointB.y = (xfB.q.s * mp0p.x + xfB.q.c * mp0p.y) + xfB.p.y

                if (MathUtils.distanceSquared(pointA, pointB) > Settings.EPSILON * Settings.EPSILON) {
                    normal.x = pointB.x - pointA.x
                    normal.y = pointB.y - pointA.y
                    normal.normalize()
                }

                val cAx = normal.x * radiusA + pointA.x
                val cAy = normal.y * radiusA + pointA.y
                val cBx = -normal.x * radiusB + pointB.x
                val cBy = -normal.y * radiusB + pointB.y
                points[0].x = (cAx + cBx) * .5f
                points[0].y = (cAy + cBy) * .5f
                separations[0] = (cBx - cAx) * normal.x + (cBy - cAy) * normal.y
            }
            Manifold.ManifoldType.FACE_A -> {
                val planePoint = pool3
                Rot.mulToOutUnsafe(xfA.q, manifold.localNormal, normal)
                Transform.mulToOut(xfA, manifold.localPoint, planePoint)
                val clipPoint = pool4
                for (i in 0 until manifold.pointCount) {
                    Transform.mulToOut(xfB, manifold.points[i].localPoint, clipPoint)
                    val scalar = radiusA - ((clipPoint.x - planePoint.x) * normal.x + (clipPoint.y - planePoint.y) * normal.y)
                    val cAx = normal.x * scalar + clipPoint.x
                    val cAy = normal.y * scalar + clipPoint.y
                    val cBx = -normal.x * radiusB + clipPoint.x
                    val cBy = -normal.y * radiusB + clipPoint.y
                    points[i].x = (cAx + cBx) * .5f
                    points[i].y = (cAy + cBy) * .5f
                    separations[i] = (cBx - cAx) * normal.x + (cBy - cAy) * normal.y
                }
            }
            Manifold.ManifoldType.FACE_B -> {
                val planePoint = pool3
                Rot.mulToOutUnsafe(xfB.q, manifold.localNormal, normal)
                Transform.mulToOut(xfB, manifold.localPoint, planePoint)
                val clipPoint = pool4
                for (i in 0 until manifold.pointCount) {
                    Transform.mulToOut(xfA, manifold.points[i].localPoint, clipPoint)
                    val scalar = radiusB - ((clipPoint.x - planePoint.x) * normal.x + (clipPoint.y - planePoint.y) * normal.y)
                    val cBx = normal.x * scalar + clipPoint.x
                    val cBy = normal.y * scalar + clipPoint.y
                    val cAx = -normal.x * radiusA + clipPoint.x
                    val cAy = -normal.y * radiusA + clipPoint.y
                    points[i].x = (cAx + cBx) * .5f
                    points[i].y = (cAy + cBy) * .5f
                    separations[i] = (cAx - cBx) * normal.x + (cAy - cBy) * normal.y
                }
                normal.x = -normal.x
                normal.y = -normal.y
            }
            null -> Unit
        }
    }
}
