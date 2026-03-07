package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.Body

class FrictionJointDef : JointDef(JointType.FRICTION) {
    val localAnchorA: Vec2 = Vec2()
    val localAnchorB: Vec2 = Vec2()
    var maxForce: Float = 0f
    var maxTorque: Float = 0f

    fun initialize(bodyA: Body, bodyB: Body, anchor: Vec2) {
        this.bodyA = bodyA
        this.bodyB = bodyB
        bodyA.getLocalPointToOut(anchor, localAnchorA)
        bodyB.getLocalPointToOut(anchor, localAnchorB)
    }
}
