package xyz.thewind.kbox2d.common

import kotlin.math.ceil

object Settings {
    const val EPSILON: Float = 1.1920929e-7f
    val PI: Float = kotlin.math.PI.toFloat()

    var FAST_ABS: Boolean = true
    var FAST_FLOOR: Boolean = true
    var FAST_CEIL: Boolean = true
    var FAST_ROUND: Boolean = true
    var FAST_ATAN2: Boolean = true
    var FAST_POW: Boolean = true
    var CONTACT_STACK_INIT_SIZE: Int = 10
    var SINCOS_LUT_ENABLED: Boolean = true
    const val SINCOS_LUT_PRECISION: Float = 0.00011f
    val SINCOS_LUT_LENGTH: Int = ceil(PI * 2.0 / SINCOS_LUT_PRECISION).toInt()
    var SINCOS_LUT_LERP: Boolean = false

    var maxManifoldPoints: Int = 2
    var maxPolygonVertices: Int = 8
    var aabbExtension: Float = 0.1f
    var aabbMultiplier: Float = 2.0f
    var linearSlop: Float = 0.005f
    var angularSlop: Float = (2.0f / 180.0f * PI)
    var polygonRadius: Float = 2.0f * linearSlop
    var maxSubSteps: Int = 8

    var maxTOIContacts: Int = 32
    var velocityThreshold: Float = 1.0f
    var maxLinearCorrection: Float = 0.2f
    var maxAngularCorrection: Float = (8.0f / 180.0f * PI)
    var maxTranslation: Float = 2.0f
    var maxTranslationSquared: Float = maxTranslation * maxTranslation
    var maxRotation: Float = 0.5f * PI
    var maxRotationSquared: Float = maxRotation * maxRotation
    var baumgarte: Float = 0.2f
    var toiBaugarte: Float = 0.75f

    var timeToSleep: Float = 0.5f
    var linearSleepTolerance: Float = 0.01f
    var angularSleepTolerance: Float = (2.0f / 180.0f * PI)

    const val invalidParticleIndex: Int = -1
    const val particleStride: Float = 0.75f
    const val minParticleWeight: Float = 1.0f
    const val maxParticleWeight: Float = 5.0f
    const val maxTriadDistance: Int = 2
    const val maxTriadDistanceSquared: Int = maxTriadDistance * maxTriadDistance
    const val minParticleBufferCapacity: Int = 256

    fun mixFriction(friction1: Float, friction2: Float): Float = MathUtils.sqrt(friction1 * friction2)

    fun mixRestitution(restitution1: Float, restitution2: Float): Float =
        if (restitution1 > restitution2) restitution1 else restitution2
}
