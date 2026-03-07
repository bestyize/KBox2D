package xyz.thewind.kbox2d.dynamics

class Filter {
    var categoryBits: Int = 0x0001
    var maskBits: Int = 0xFFFF
    var groupIndex: Int = 0

    fun set(other: Filter) {
        categoryBits = other.categoryBits
        maskBits = other.maskBits
        groupIndex = other.groupIndex
    }
}
