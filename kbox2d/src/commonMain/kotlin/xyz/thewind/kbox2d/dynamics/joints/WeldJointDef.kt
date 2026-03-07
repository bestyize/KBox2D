package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.Body

class WeldJointDef : JointDef(JointType.WELD) {
    val localAnchorA: Vec2 = Vec2()
    val localAnchorB: Vec2 = Vec2()
    var referenceAngle: Float = 0f
    var frequencyHz: Float = 0f
    var dampingRatio: Float = 0f

    fun initialize(bodyA: Body, bodyB: Body, anchor: Vec2) {
        this.bodyA = bodyA
        this.bodyB = bodyB
        bodyA.getLocalPointToOut(anchor, localAnchorA)
        bodyB.getLocalPointToOut(anchor, localAnchorB)
        referenceAngle = bodyB.getAngle() - bodyA.getAngle()
    }
}
