package xyz.thewind.kbox2d.collision.shapes

// Keep existing assertion call sites portable across KMP compilation targets.
internal fun assert(condition: Boolean) = xyz.thewind.kbox2d.common.assert(condition)
