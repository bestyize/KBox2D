package xyz.thewind.kbox2d.dynamics

import xyz.thewind.kbox2d.common.Vec2

class BodyDef {
    var type: BodyType = BodyType.STATIC
    var userData: Any? = null
    var position: Vec2 = Vec2()
    var angle: Float = 0f
    var linearVelocity: Vec2 = Vec2()
    var angularVelocity: Float = 0f
    var linearDamping: Float = 0f
    var angularDamping: Float = 0f
    var allowSleep: Boolean = true
    var awake: Boolean = true
    var fixedRotation: Boolean = false
    var bullet: Boolean = false
    var active: Boolean = true
    var gravityScale: Float = 1f
}
