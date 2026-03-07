package xyz.thewind.kbox2d.common

import kotlin.math.pow

internal object PlatformMathUtils {
    fun fastPow(a: Float, b: Float): Float = a.toDouble().pow(b.toDouble()).toFloat()
}
