package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Vec2

class MouseJointDef : JointDef(JointType.MOUSE) {
    val target: Vec2 = Vec2()
    var maxForce: Float = 0f
    var frequencyHz: Float = 5f
    var dampingRatio: Float = 0.7f
}
