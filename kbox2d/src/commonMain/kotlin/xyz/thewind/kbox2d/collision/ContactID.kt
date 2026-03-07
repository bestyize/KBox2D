package xyz.thewind.kbox2d.collision

class ContactID() : Comparable<ContactID> {
    enum class Type {
        VERTEX,
        FACE,
    }

    var indexA: Byte = 0
    var indexB: Byte = 0
    var typeA: Byte = 0
    var typeB: Byte = 0

    constructor(c: ContactID) : this() {
        set(c)
    }

    fun getKey(): Int =
        (indexA.toInt() shl 24) or (indexB.toInt() shl 16) or (typeA.toInt() shl 8) or typeB.toInt()

    fun isEqual(cid: ContactID): Boolean = getKey() == cid.getKey()

    fun set(c: ContactID) {
        indexA = c.indexA
        indexB = c.indexB
        typeA = c.typeA
        typeB = c.typeB
    }

    fun flip() {
        var temp = indexA
        indexA = indexB
        indexB = temp
        temp = typeA
        typeA = typeB
        typeB = temp
    }

    fun zero() {
        indexA = 0
        indexB = 0
        typeA = 0
        typeB = 0
    }

    override fun compareTo(other: ContactID): Int = getKey() - other.getKey()
}
