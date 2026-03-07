package xyz.thewind.kbox2d.common

object BufferUtils {
    @Suppress("UNCHECKED_CAST")
    fun <T> reallocateBuffer(oldBuffer: Array<T>?, oldCapacity: Int, newCapacity: Int, create: () -> T): Array<T> {
        require(newCapacity > oldCapacity)
        val newBuffer = arrayOfNulls<Any?>(newCapacity) as Array<T>
        for (index in 0 until oldCapacity) {
            if (oldBuffer != null) {
                newBuffer[index] = oldBuffer[index]
            }
        }
        for (index in oldCapacity until newCapacity) {
            newBuffer[index] = create()
        }
        return newBuffer
    }

    fun reallocateBuffer(oldBuffer: IntArray?, oldCapacity: Int, newCapacity: Int): IntArray {
        require(newCapacity > oldCapacity)
        val newBuffer = IntArray(newCapacity)
        if (oldBuffer != null) {
            oldBuffer.copyInto(newBuffer, endIndex = oldCapacity)
        }
        return newBuffer
    }

    fun reallocateBuffer(oldBuffer: kotlin.FloatArray?, oldCapacity: Int, newCapacity: Int): kotlin.FloatArray {
        require(newCapacity > oldCapacity)
        val newBuffer = kotlin.FloatArray(newCapacity)
        if (oldBuffer != null) {
            oldBuffer.copyInto(newBuffer, endIndex = oldCapacity)
        }
        return newBuffer
    }

    fun <T> reallocateBuffer(
        buffer: Array<T>?,
        userSuppliedCapacity: Int,
        oldCapacity: Int,
        newCapacity: Int,
        deferred: Boolean,
        create: () -> T,
    ): Array<T>? {
        require(newCapacity > oldCapacity)
        require(userSuppliedCapacity == 0 || newCapacity <= userSuppliedCapacity)
        return if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
            reallocateBuffer(buffer, oldCapacity, newCapacity, create)
        } else {
            buffer
        }
    }

    fun reallocateBuffer(
        buffer: IntArray?,
        userSuppliedCapacity: Int,
        oldCapacity: Int,
        newCapacity: Int,
        deferred: Boolean,
    ): IntArray? {
        require(newCapacity > oldCapacity)
        require(userSuppliedCapacity == 0 || newCapacity <= userSuppliedCapacity)
        return if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
            reallocateBuffer(buffer, oldCapacity, newCapacity)
        } else {
            buffer
        }
    }

    fun reallocateBuffer(
        buffer: kotlin.FloatArray?,
        userSuppliedCapacity: Int,
        oldCapacity: Int,
        newCapacity: Int,
        deferred: Boolean,
    ): kotlin.FloatArray? {
        require(newCapacity > oldCapacity)
        require(userSuppliedCapacity == 0 || newCapacity <= userSuppliedCapacity)
        return if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
            reallocateBuffer(buffer, oldCapacity, newCapacity)
        } else {
            buffer
        }
    }

    fun <T> rotate(ray: Array<T>, first: Int, newFirst: Int, last: Int) {
        var localFirst = first
        var localNewFirst = newFirst
        var next = localNewFirst
        while (next != localFirst) {
            val temp = ray[localFirst]
            ray[localFirst] = ray[next]
            ray[next] = temp
            localFirst++
            next++
            if (next == last) {
                next = localNewFirst
            } else if (localFirst == localNewFirst) {
                localNewFirst = next
            }
        }
    }

    fun rotate(ray: IntArray, first: Int, newFirst: Int, last: Int) {
        var localFirst = first
        var localNewFirst = newFirst
        var next = localNewFirst
        while (next != localFirst) {
            val temp = ray[localFirst]
            ray[localFirst] = ray[next]
            ray[next] = temp
            localFirst++
            next++
            if (next == last) {
                next = localNewFirst
            } else if (localFirst == localNewFirst) {
                localNewFirst = next
            }
        }
    }

    fun rotate(ray: kotlin.FloatArray, first: Int, newFirst: Int, last: Int) {
        var localFirst = first
        var localNewFirst = newFirst
        var next = localNewFirst
        while (next != localFirst) {
            val temp = ray[localFirst]
            ray[localFirst] = ray[next]
            ray[next] = temp
            localFirst++
            next++
            if (next == last) {
                next = localNewFirst
            } else if (localFirst == localNewFirst) {
                localNewFirst = next
            }
        }
    }
}
