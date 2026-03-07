package xyz.thewind.kbox2d.dynamics.contacts

import xyz.thewind.kbox2d.common.Mat22
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Vec2

class ContactVelocityConstraint {
    val points: Array<VelocityConstraintPoint> = Array(Settings.maxManifoldPoints) { VelocityConstraintPoint() }
    val normal: Vec2 = Vec2()
    val normalMass: Mat22 = Mat22()
    val K: Mat22 = Mat22()
    var indexA: Int = 0
    var indexB: Int = 0
    var invMassA: Float = 0f
    var invMassB: Float = 0f
    var invIA: Float = 0f
    var invIB: Float = 0f
    var friction: Float = 0f
    var restitution: Float = 0f
    var tangentSpeed: Float = 0f
    var pointCount: Int = 0
    var contactIndex: Int = 0

    class VelocityConstraintPoint {
        val rA: Vec2 = Vec2()
        val rB: Vec2 = Vec2()
        var normalImpulse: Float = 0f
        var tangentImpulse: Float = 0f
        var normalMass: Float = 0f
        var tangentMass: Float = 0f
        var velocityBias: Float = 0f
    }
}
