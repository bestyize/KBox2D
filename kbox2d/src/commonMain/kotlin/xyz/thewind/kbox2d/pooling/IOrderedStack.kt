package xyz.thewind.kbox2d.pooling

interface IOrderedStack<E> {
    fun pop(): E
    fun pop(argNum: Int): Array<E>
    fun push(argNum: Int)
}
