package xyz.thewind.kbox2d.common

class Color3f(var x: Float = 0f, var y: Float = 0f, var z: Float = 0f) {
    fun set(r: Float, g: Float, b: Float) {
        x = r
        y = g
        z = b
    }

    fun set(argColor: Color3f) {
        x = argColor.x
        y = argColor.y
        z = argColor.z
    }

    companion object {
        val WHITE = Color3f(1f, 1f, 1f)
        val BLACK = Color3f(0f, 0f, 0f)
        val BLUE = Color3f(0f, 0f, 1f)
        val GREEN = Color3f(0f, 1f, 0f)
        val RED = Color3f(1f, 0f, 0f)
    }
}
