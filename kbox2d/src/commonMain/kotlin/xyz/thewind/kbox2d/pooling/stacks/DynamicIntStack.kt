package xyz.thewind.kbox2d.pooling.stacks

class DynamicIntStack(initialSize: Int) {
    private var stack: IntArray = IntArray(initialSize)
    private var size: Int = initialSize
    private var position: Int = 0

    fun reset() {
        position = 0
    }

    fun pop(): Int {
        assert(position > 0)
        return stack[--position]
    }

    fun push(value: Int) {
        if (position == size) {
            stack = stack.copyOf(size * 2)
            size = stack.size
        }
        stack[position++] = value
    }

    fun getCount(): Int = position
}
