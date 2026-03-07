package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.Body

class DistanceJointDef : JointDef(JointType.DISTANCE) {
    val localAnchorA: Vec2 = Vec2(0f, 0f)
    val localAnchorB: Vec2 = Vec2(0f, 0f)
    var length: Float = 1f
    var frequencyHz: Float = 0f
    var dampingRatio: Float = 0f

    fun initialize(b1: Body, b2: Body, anchor1: Vec2, anchor2: Vec2) {
        bodyA = b1
        bodyB = b2
        localAnchorA.set(b1.getLocalPoint(anchor1))
        localAnchorB.set(b2.getLocalPoint(anchor2))
        length = anchor2.sub(anchor1).length()
    }
}
