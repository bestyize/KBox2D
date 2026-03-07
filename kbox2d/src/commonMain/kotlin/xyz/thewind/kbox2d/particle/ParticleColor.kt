package xyz.thewind.kbox2d.particle

import xyz.thewind.kbox2d.common.Color3f

class ParticleColor(
    var r: Byte = 127,
    var g: Byte = 127,
    var b: Byte = 127,
    var a: Byte = 50,
) {
    constructor(color: Color3f) : this() {
        set(color)
    }

    fun set(color: Color3f) {
        r = (255 * color.x).toInt().toByte()
        g = (255 * color.y).toInt().toByte()
        b = (255 * color.z).toInt().toByte()
        a = 255.toByte()
    }

    fun set(color: ParticleColor) {
        r = color.r
        g = color.g
        b = color.b
        a = color.a
    }

    fun isZero(): Boolean = r == 0.toByte() && g == 0.toByte() && b == 0.toByte() && a == 0.toByte()

    fun set(r: Byte, g: Byte, b: Byte, a: Byte) {
        this.r = r
        this.g = g
        this.b = b
        this.a = a
    }
}
