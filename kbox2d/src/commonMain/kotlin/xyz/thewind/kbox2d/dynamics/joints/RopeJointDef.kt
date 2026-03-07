package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.common.Vec2

class RopeJointDef : JointDef(JointType.ROPE) {
    val localAnchorA: Vec2 = Vec2()
    val localAnchorB: Vec2 = Vec2()
    var maxLength: Float = 0f

    init {
        localAnchorA.set(-1f, 0f)
        localAnchorB.set(1f, 0f)
    }
}
