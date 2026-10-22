package xyz.thewind.kbox2d.common

/** Common-source invariant check, available to JVM, Android and Kotlin/Native. */
internal fun assert(condition: Boolean) {
    check(condition) { "KBox2D internal invariant violated" }
}
