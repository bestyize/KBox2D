package xyz.thewind.kbox2d.dynamics.joints

class GearJointDef : JointDef(JointType.GEAR) {
    var joint1: Joint? = null
        set(value) {
            field = value
            if (bodyA == null && value != null) {
                bodyA = value.getBodyB()
            }
        }
    var joint2: Joint? = null
        set(value) {
            field = value
            if (bodyB == null && value != null) {
                bodyB = value.getBodyB()
            }
        }
    var ratio: Float = 0f
}
