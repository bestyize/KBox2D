package xyz.thewind.kbox2d.collision

import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Vec2

class Manifold {
    enum class ManifoldType {
        CIRCLES,
        FACE_A,
        FACE_B,
    }

    val points: Array<ManifoldPoint>
    val localNormal: Vec2
    val localPoint: Vec2
    var type: ManifoldType? = null
    var pointCount: Int = 0

    constructor() {
        points = Array(Settings.maxManifoldPoints) { ManifoldPoint() }
        localNormal = Vec2()
        localPoint = Vec2()
    }

    constructor(other: Manifold) {
        points = Array(Settings.maxManifoldPoints) { index -> ManifoldPoint(other.points[index]) }
        localNormal = other.localNormal.clone()
        localPoint = other.localPoint.clone()
        pointCount = other.pointCount
        type = other.type
    }

    fun set(cp: Manifold) {
        for (i in 0 until cp.pointCount) {
            points[i].set(cp.points[i])
        }
        type = cp.type
        localNormal.set(cp.localNormal)
        localPoint.set(cp.localPoint)
        pointCount = cp.pointCount
    }
}
