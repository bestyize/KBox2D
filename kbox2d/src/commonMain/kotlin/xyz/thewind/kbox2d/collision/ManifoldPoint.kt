package xyz.thewind.kbox2d.collision

import xyz.thewind.kbox2d.common.Vec2

class ManifoldPoint() {
    val localPoint: Vec2 = Vec2()
    var normalImpulse: Float = 0f
    var tangentImpulse: Float = 0f
    val id: ContactID = ContactID()

    constructor(cp: ManifoldPoint) : this() {
        localPoint.set(cp.localPoint)
        normalImpulse = cp.normalImpulse
        tangentImpulse = cp.tangentImpulse
        id.set(cp.id)
    }

    fun set(cp: ManifoldPoint) {
        localPoint.set(cp.localPoint)
        normalImpulse = cp.normalImpulse
        tangentImpulse = cp.tangentImpulse
        id.set(cp.id)
    }
}
