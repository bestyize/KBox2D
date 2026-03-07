package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.dynamics.Body

open class JointDef(
    val type: JointType,
) {
    var userData: Any? = null
    var bodyA: Body? = null
    var bodyB: Body? = null
    var collideConnected: Boolean = false
}
