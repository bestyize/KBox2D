package xyz.thewind.kbox2d.common

import kotlin.time.TimeMark
import kotlin.time.TimeSource

class Timer {
    private var resetMark: TimeMark = TimeSource.Monotonic.markNow()

    fun reset() {
        resetMark = TimeSource.Monotonic.markNow()
    }

    fun getMilliseconds(): Float = resetMark.elapsedNow().inWholeNanoseconds.toFloat() / 1_000_000f
}
