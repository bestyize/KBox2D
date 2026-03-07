package xyz.thewind.kbox2d.pooling

interface IDynamicStack<E> {
    fun pop(): E
    fun push(argObject: E)
}
