package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.Body

class MotorJointDef : JointDef(JointType.MOTOR) {
    val linearOffset: Vec2 = Vec2()
    var angularOffset: Float = 0f
    var maxForce: Float = 1f
    var maxTorque: Float = 1f
    var correctionFactor: Float = 0.3f

    fun initialize(bodyA: Body, bodyB: Body) {
        this.bodyA = bodyA
        this.bodyB = bodyB
        bodyA.getLocalPointToOut(bodyB.getPosition(), linearOffset)
        angularOffset = bodyB.getAngle() - bodyA.getAngle()
    }
}
