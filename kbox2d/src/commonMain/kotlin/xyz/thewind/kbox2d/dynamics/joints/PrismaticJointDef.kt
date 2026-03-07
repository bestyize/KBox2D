package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.Body

class PrismaticJointDef : JointDef(JointType.PRISMATIC) {
    val localAnchorA: Vec2 = Vec2()
    val localAnchorB: Vec2 = Vec2()
    val localAxisA: Vec2 = Vec2(1f, 0f)
    var referenceAngle: Float = 0f
    var enableLimit: Boolean = false
    var lowerTranslation: Float = 0f
    var upperTranslation: Float = 0f
    var enableMotor: Boolean = false
    var maxMotorForce: Float = 0f
    var motorSpeed: Float = 0f

    fun initialize(bodyA: Body, bodyB: Body, anchor: Vec2, axis: Vec2) {
        this.bodyA = bodyA
        this.bodyB = bodyB
        bodyA.getLocalPointToOut(anchor, localAnchorA)
        bodyB.getLocalPointToOut(anchor, localAnchorB)
        bodyA.getLocalVectorToOut(axis, localAxisA)
        referenceAngle = bodyB.getAngle() - bodyA.getAngle()
    }
}
