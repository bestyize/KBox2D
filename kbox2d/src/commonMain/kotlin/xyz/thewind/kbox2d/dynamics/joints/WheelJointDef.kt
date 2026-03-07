package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.Body

class WheelJointDef : JointDef(JointType.WHEEL) {
    val localAnchorA: Vec2 = Vec2()
    val localAnchorB: Vec2 = Vec2()
    val localAxisA: Vec2 = Vec2(1f, 0f)
    var enableMotor: Boolean = false
    var maxMotorTorque: Float = 0f
    var motorSpeed: Float = 0f
    var frequencyHz: Float = 0f
    var dampingRatio: Float = 0f

    fun initialize(b1: Body, b2: Body, anchor: Vec2, axis: Vec2) {
        bodyA = b1
        bodyB = b2
        b1.getLocalPointToOut(anchor, localAnchorA)
        b2.getLocalPointToOut(anchor, localAnchorB)
        b1.getLocalVectorToOut(axis, localAxisA)
    }
}
