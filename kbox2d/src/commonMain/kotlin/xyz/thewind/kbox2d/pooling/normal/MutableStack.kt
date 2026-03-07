package xyz.thewind.kbox2d.pooling.normal

import xyz.thewind.kbox2d.pooling.IDynamicStack

abstract class MutableStack<E>(argInitSize: Int) : IDynamicStack<E> {
    private var stack: Array<E>? = null
    private var index: Int = 0
    private var size: Int = 0

    init {
        extendStack(argInitSize)
    }

    override fun pop(): E {
        if (index >= size) {
            extendStack(size * 2)
        }
        return stack!![index++]
    }

    override fun push(argObject: E) {
        assert(index > 0)
        stack!![--index] = argObject
    }

    private fun extendStack(argSize: Int) {
        val newStack = newArray(argSize)
        val oldSize = size
        stack?.copyInto(newStack, endIndex = oldSize)
        for (arrayIndex in oldSize until newStack.size) {
            newStack[arrayIndex] = newInstance()
        }
        stack = newStack
        size = newStack.size
    }

    protected abstract fun newInstance(): E
    protected abstract fun newArray(size: Int): Array<E>
}
