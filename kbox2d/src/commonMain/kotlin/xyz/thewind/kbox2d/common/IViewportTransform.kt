package xyz.thewind.kbox2d.common

interface IViewportTransform {
    fun isYFlip(): Boolean
    fun setYFlip(yFlip: Boolean)
    fun getExtents(): Vec2
    fun setExtents(extents: Vec2)
    fun setExtents(halfWidth: Float, halfHeight: Float)
    fun getCenter(): Vec2
    fun setCenter(pos: Vec2)
    fun setCenter(x: Float, y: Float)
    fun setCamera(x: Float, y: Float, scale: Float)
    fun getWorldVectorToScreen(world: Vec2, screen: Vec2)
    fun getScreenVectorToWorld(screen: Vec2, world: Vec2)
    fun getMat22Representation(): Mat22
    fun getWorldToScreen(world: Vec2, screen: Vec2)
    fun getScreenToWorld(screen: Vec2, world: Vec2)
    fun mulByTransform(transform: Mat22)
}
