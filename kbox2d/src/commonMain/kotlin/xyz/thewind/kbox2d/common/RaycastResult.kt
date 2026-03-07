package xyz.thewind.kbox2d.common

class RaycastResult {
    var lambda: Float = 0.0f
    val normal: Vec2 = Vec2()

    fun set(argOther: RaycastResult): RaycastResult {
        lambda = argOther.lambda
        normal.set(argOther.normal)
        return this
    }
}
