package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.dynamics.Body

class ConstantVolumeJointDef : JointDef(JointType.CONSTANT_VOLUME) {
    var frequencyHz: Float = 0f
    var dampingRatio: Float = 0f
    val bodies: MutableList<Body> = mutableListOf()
    val joints: MutableList<Joint> = mutableListOf()

    init {
        collideConnected = false
    }

    fun addBody(body: Body) {
        bodies += body
        if (bodies.size == 1) {
            bodyA = body
        }
        if (bodies.size == 2) {
            bodyB = body
        }
    }

    fun addBodyAndJoint(body: Body, joint: Joint) {
        addBody(body)
        joints += joint
    }
}
