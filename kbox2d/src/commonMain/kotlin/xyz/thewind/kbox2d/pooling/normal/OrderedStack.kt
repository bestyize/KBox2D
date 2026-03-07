package xyz.thewind.kbox2d.pooling.normal

import xyz.thewind.kbox2d.pooling.IOrderedStack

abstract class OrderedStack<E>(argStackSize: Int, argContainerSize: Int) : IOrderedStack<E> {
    private val pool: Array<E> = newArray(argStackSize)
    private var index: Int = 0
    private val size: Int = argStackSize
    private val container: Array<E> = newArray(argContainerSize)

    init {
        for (poolIndex in 0 until argStackSize) {
            pool[poolIndex] = newInstance()
        }
    }

    override fun pop(): E {
        assert(index < size) { "End of stack reached, there is probably a leak somewhere" }
        return pool[index++]
    }

    override fun pop(argNum: Int): Array<E> {
        assert(index + argNum < size) { "End of stack reached, there is probably a leak somewhere" }
        assert(argNum <= container.size) { "Container array is too small" }
        pool.copyInto(container, destinationOffset = 0, startIndex = index, endIndex = index + argNum)
        index += argNum
        return container
    }

    override fun push(argNum: Int) {
        index -= argNum
        assert(index >= 0) { "Beginning of stack reached, push/pops are unmatched" }
    }

    protected abstract fun newInstance(): E
    protected abstract fun newArray(size: Int): Array<E>
}
