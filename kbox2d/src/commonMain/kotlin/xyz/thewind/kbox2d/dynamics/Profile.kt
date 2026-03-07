package xyz.thewind.kbox2d.dynamics

import xyz.thewind.kbox2d.common.MathUtils

class Profile {
    class ProfileEntry {
        var longAvg: Float = 0f
        var shortAvg: Float = 0f
        var min: Float = Float.MAX_VALUE
        var max: Float = -Float.MAX_VALUE
        var accum: Float = 0f

        fun record(value: Float) {
            longAvg = longAvg * (1 - LONG_FRACTION) + value * LONG_FRACTION
            shortAvg = shortAvg * (1 - SHORT_FRACTION) + value * SHORT_FRACTION
            min = MathUtils.min(value, min)
            max = MathUtils.max(value, max)
        }

        fun startAccum() {
            accum = 0f
        }

        fun accum(value: Float) {
            accum += value
        }

        fun endAccum() {
            record(accum)
        }

        override fun toString(): String {
            fun Float.format2(): String = "%.2f".format(this)
            return "${shortAvg.format2()} (${longAvg.format2()}) [${min.format2()},${max.format2()}]"
        }
    }

    val step = ProfileEntry()
    val stepInit = ProfileEntry()
    val collide = ProfileEntry()
    val solveParticleSystem = ProfileEntry()
    val solve = ProfileEntry()
    val solveInit = ProfileEntry()
    val solveVelocity = ProfileEntry()
    val solvePosition = ProfileEntry()
    val broadphase = ProfileEntry()
    val solveTOI = ProfileEntry()

    fun toDebugStrings(strings: MutableList<String>) {
        strings.add("Profile:")
        strings.add(" step: $step")
        strings.add("  init: $stepInit")
        strings.add("  collide: $collide")
        strings.add("  particles: $solveParticleSystem")
        strings.add("  solve: $solve")
        strings.add("   solveInit: $solveInit")
        strings.add("   solveVelocity: $solveVelocity")
        strings.add("   solvePosition: $solvePosition")
        strings.add("   broadphase: $broadphase")
        strings.add("  solveTOI: $solveTOI")
    }

    companion object {
        private const val LONG_AVG_NUMS = 20
        private const val LONG_FRACTION = 1f / LONG_AVG_NUMS
        private const val SHORT_AVG_NUMS = 5
        private const val SHORT_FRACTION = 1f / SHORT_AVG_NUMS
    }
}
