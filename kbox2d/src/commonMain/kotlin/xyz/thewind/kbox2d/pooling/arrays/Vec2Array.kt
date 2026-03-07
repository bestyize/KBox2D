package xyz.thewind.kbox2d.pooling.arrays

import xyz.thewind.kbox2d.common.Vec2

class Vec2Array {
    private val map = mutableMapOf<Int, Array<Vec2>>()

    fun get(argLength: Int): Array<Vec2> {
        require(argLength > 0)
        return map.getOrPut(argLength) { getInitializedArray(argLength) }
    }

    protected fun getInitializedArray(argLength: Int): Array<Vec2> = Array(argLength) { Vec2() }
}
