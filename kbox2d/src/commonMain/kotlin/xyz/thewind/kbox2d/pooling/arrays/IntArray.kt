package xyz.thewind.kbox2d.pooling.arrays

class IntArray {
    private val map = mutableMapOf<Int, kotlin.IntArray>()

    fun get(argLength: Int): kotlin.IntArray {
        require(argLength > 0)
        return map.getOrPut(argLength) { getInitializedArray(argLength) }
    }

    protected fun getInitializedArray(argLength: Int): kotlin.IntArray = kotlin.IntArray(argLength)
}
