package xyz.thewind.kbox2d.common

class OBBViewportTransform : IViewportTransform {
    class OBB {
        val R: Mat22 = Mat22()
        val center: Vec2 = Vec2()
        val extents: Vec2 = Vec2()
    }

    protected val box: OBB = OBB()
    private var yFlip: Boolean = false
    private val yFlipMat: Mat22 = Mat22(1f, 0f, 0f, -1f)
    private val inv: Mat22 = Mat22()
    private val inv2: Mat22 = Mat22()

    init {
        box.R.setIdentity()
    }

    fun set(vpt: OBBViewportTransform) {
        box.center.set(vpt.box.center)
        box.extents.set(vpt.box.extents)
        box.R.set(vpt.box.R)
        yFlip = vpt.yFlip
    }

    override fun setCamera(x: Float, y: Float, scale: Float) {
        box.center.set(x, y)
        Mat22.createScaleTransform(scale, box.R)
    }

    override fun getExtents(): Vec2 = box.extents

    override fun getMat22Representation(): Mat22 = box.R

    override fun setExtents(extents: Vec2) {
        box.extents.set(extents)
    }

    override fun setExtents(halfWidth: Float, halfHeight: Float) {
        box.extents.set(halfWidth, halfHeight)
    }

    override fun getCenter(): Vec2 = box.center

    override fun setCenter(pos: Vec2) {
        box.center.set(pos)
    }

    override fun setCenter(x: Float, y: Float) {
        box.center.set(x, y)
    }

    fun getTransform(): Mat22 = box.R

    fun setTransform(transform: Mat22) {
        box.R.set(transform)
    }

    override fun mulByTransform(transform: Mat22) {
        box.R.mulLocal(transform)
    }

    override fun isYFlip(): Boolean = yFlip

    override fun setYFlip(yFlip: Boolean) {
        this.yFlip = yFlip
    }

    override fun getScreenVectorToWorld(screen: Vec2, world: Vec2) {
        box.R.invertToOut(inv)
        inv.mulToOut(screen, world)
        if (yFlip) {
            yFlipMat.mulToOut(world, world)
        }
    }

    override fun getWorldVectorToScreen(world: Vec2, screen: Vec2) {
        box.R.mulToOut(world, screen)
        if (yFlip) {
            yFlipMat.mulToOut(screen, screen)
        }
    }

    override fun getWorldToScreen(world: Vec2, screen: Vec2) {
        screen.x = world.x - box.center.x
        screen.y = world.y - box.center.y
        box.R.mulToOut(screen, screen)
        if (yFlip) {
            yFlipMat.mulToOut(screen, screen)
        }
        screen.x += box.extents.x
        screen.y += box.extents.y
    }

    override fun getScreenToWorld(screen: Vec2, world: Vec2) {
        world.x = screen.x - box.extents.x
        world.y = screen.y - box.extents.y
        if (yFlip) {
            yFlipMat.mulToOut(world, world)
        }
        box.R.invertToOut(inv2)
        inv2.mulToOut(world, world)
        world.x += box.center.x
        world.y += box.center.y
    }
}
