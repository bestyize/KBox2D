package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.Body

class PulleyJointDef : JointDef(JointType.PULLEY) {
    val groundAnchorA: Vec2 = Vec2(-1f, 1f)
    val groundAnchorB: Vec2 = Vec2(1f, 1f)
    val localAnchorA: Vec2 = Vec2(-1f, 0f)
    val localAnchorB: Vec2 = Vec2(1f, 0f)
    var lengthA: Float = 0f
    var lengthB: Float = 0f
    var ratio: Float = 1f

    init {
        collideConnected = true
    }

    fun initialize(b1: Body, b2: Body, ga1: Vec2, ga2: Vec2, anchor1: Vec2, anchor2: Vec2, r: Float) {
        bodyA = b1
        bodyB = b2
        groundAnchorA.set(ga1)
        groundAnchorB.set(ga2)
        localAnchorA.set(b1.getLocalPoint(anchor1))
        localAnchorB.set(b2.getLocalPoint(anchor2))
        lengthA = anchor1.sub(ga1).length()
        lengthB = anchor2.sub(ga2).length()
        ratio = r
        require(ratio > Settings.EPSILON)
    }
}
