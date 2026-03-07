package xyz.thewind.kbox2d.common

import kotlin.math.roundToInt
import kotlin.random.Random

object MathUtils {
    const val PI: Float = kotlin.math.PI.toFloat()
    const val TWOPI: Float = (kotlin.math.PI * 2.0).toFloat()
    const val INV_PI: Float = 1f / PI
    const val HALF_PI: Float = PI / 2f
    const val QUARTER_PI: Float = PI / 4f
    const val THREE_HALVES_PI: Float = TWOPI - HALF_PI

    const val DEG2RAD: Float = PI / 180f
    const val RAD2DEG: Float = 180f / PI

    val sinLUT: FloatArray = FloatArray(Settings.SINCOS_LUT_LENGTH) { index ->
        kotlin.math.sin((index * Settings.SINCOS_LUT_PRECISION).toDouble()).toFloat()
    }

    fun sin(x: Float): Float {
        if (!Settings.SINCOS_LUT_ENABLED) {
            return kotlin.math.sin(x.toDouble()).toFloat()
        }
        return sinLUT(x)
    }

    fun sinLUT(x: Float): Float {
        var angle = x % TWOPI
        if (angle < 0f) {
            angle += TWOPI
        }
        val index = angle / Settings.SINCOS_LUT_PRECISION
        val baseIndex = index.toInt() % sinLUT.size
        if (!Settings.SINCOS_LUT_LERP) {
            return sinLUT[baseIndex]
        }
        val nextIndex = (baseIndex + 1) % sinLUT.size
        val fraction = index - baseIndex
        return sinLUT[baseIndex] + (sinLUT[nextIndex] - sinLUT[baseIndex]) * fraction
    }

    fun cos(x: Float): Float = kotlin.math.cos(x.toDouble()).toFloat()

    fun abs(x: Float): Float = kotlin.math.abs(x)

    fun fastAbs(x: Float): Float = kotlin.math.abs(x)

    fun abs(x: Int): Int = kotlin.math.abs(x)

    fun floor(x: Float): Int = kotlin.math.floor(x).toInt()

    fun fastFloor(x: Float): Int = floor(x)

    fun ceil(x: Float): Int = kotlin.math.ceil(x).toInt()

    fun fastCeil(x: Float): Int = ceil(x)

    fun round(x: Float): Int = x.roundToInt()

    fun ceilPowerOf2(x: Int): Int = nextPowerOfTwo(x)

    fun max(a: Float, b: Float): Float = kotlin.math.max(a, b)

    fun max(a: Int, b: Int): Int = kotlin.math.max(a, b)

    fun min(a: Float, b: Float): Float = kotlin.math.min(a, b)

    fun min(a: Int, b: Int): Int = kotlin.math.min(a, b)

    fun map(valValue: Float, fromMin: Float, fromMax: Float, toMin: Float, toMax: Float): Float {
        if (fromMax == fromMin) {
            return toMin
        }
        val ratio = (valValue - fromMin) / (fromMax - fromMin)
        return toMin + ratio * (toMax - toMin)
    }

    fun clamp(a: Float, low: Float, high: Float): Float = max(low, min(a, high))

    fun clamp(a: Vec2, low: Vec2, high: Vec2): Vec2 = Vec2(
        clamp(a.x, low.x, high.x),
        clamp(a.y, low.y, high.y),
    )

    fun clampToOut(a: Vec2, low: Vec2, high: Vec2, dest: Vec2) {
        dest.x = clamp(a.x, low.x, high.x)
        dest.y = clamp(a.y, low.y, high.y)
    }

    fun nextPowerOfTwo(x: Int): Int {
        if (x <= 0) {
            return 1
        }
        var value = x - 1
        value = value or (value shr 1)
        value = value or (value shr 2)
        value = value or (value shr 4)
        value = value or (value shr 8)
        value = value or (value shr 16)
        return value + 1
    }

    fun isPowerOfTwo(x: Int): Boolean = x > 0 && (x and (x - 1)) == 0

    fun pow(a: Float, b: Float): Float = PlatformMathUtils.fastPow(a, b)

    fun atan2(y: Float, x: Float): Float = kotlin.math.atan2(y.toDouble(), x.toDouble()).toFloat()

    fun fastAtan2(y: Float, x: Float): Float = atan2(y, x)

    fun reduceAngle(theta: Float): Float {
        var angle = theta % TWOPI
        if (angle > PI) {
            angle -= TWOPI
        } else if (angle < -PI) {
            angle += TWOPI
        }
        return angle
    }

    fun randomFloat(argLow: Float, argHigh: Float): Float = randomFloat(Random, argLow, argHigh)

    fun randomFloat(random: Random, argLow: Float, argHigh: Float): Float =
        argLow + random.nextFloat() * (argHigh - argLow)

    fun sqrt(x: Float): Float = kotlin.math.sqrt(x)

    fun distanceSquared(v1: Vec2, v2: Vec2): Float {
        val dx = v1.x - v2.x
        val dy = v1.y - v2.y
        return dx * dx + dy * dy
    }

    fun distance(v1: Vec2, v2: Vec2): Float = sqrt(distanceSquared(v1, v2))
}
