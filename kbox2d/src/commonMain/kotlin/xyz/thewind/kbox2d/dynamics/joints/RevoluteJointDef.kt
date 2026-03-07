package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.Body

class RevoluteJointDef : JointDef(JointType.REVOLUTE) {
    val localAnchorA: Vec2 = Vec2(0f, 0f)
    val localAnchorB: Vec2 = Vec2(0f, 0f)
    var referenceAngle: Float = 0f
    var enableLimit: Boolean = false
    var lowerAngle: Float = 0f
    var upperAngle: Float = 0f
    var enableMotor: Boolean = false
    var motorSpeed: Float = 0f
    var maxMotorTorque: Float = 0f

    fun initialize(b1: Body, b2: Body, anchor: Vec2) {
        bodyA = b1
        bodyB = b2
        b1.getLocalPointToOut(anchor, localAnchorA)
        b2.getLocalPointToOut(anchor, localAnchorB)
        referenceAngle = b2.getAngle() - b1.getAngle()
    }
}
