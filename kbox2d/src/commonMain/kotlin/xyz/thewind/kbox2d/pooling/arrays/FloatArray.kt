package xyz.thewind.kbox2d.pooling.arrays

class FloatArray {
    private val map = mutableMapOf<Int, kotlin.FloatArray>()

    fun get(argLength: Int): kotlin.FloatArray {
        require(argLength > 0)
        return map.getOrPut(argLength) { getInitializedArray(argLength) }
    }

    protected fun getInitializedArray(argLength: Int): kotlin.FloatArray = kotlin.FloatArray(argLength)
}
