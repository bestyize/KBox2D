package xyz.thewind.kbox2d.callbacks

import xyz.thewind.kbox2d.common.Color3f
import xyz.thewind.kbox2d.common.IViewportTransform
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.particle.ParticleColor

abstract class DebugDraw(viewport: IViewportTransform? = null) {
    var m_drawFlags: Int = 0
    protected var viewportTransformInternal: IViewportTransform? = viewport

    fun setViewportTransform(viewportTransform: IViewportTransform?) {
        this.viewportTransformInternal = viewportTransform
    }

    fun setFlags(flags: Int) {
        m_drawFlags = flags
    }

    fun getFlags(): Int = m_drawFlags

    fun appendFlags(flags: Int) {
        m_drawFlags = m_drawFlags or flags
    }

    fun clearFlags(flags: Int) {
        m_drawFlags = m_drawFlags and flags.inv()
    }

    fun drawPolygon(vertices: Array<Vec2>, vertexCount: Int, color: Color3f) {
        if (vertexCount == 1) {
            drawSegment(vertices[0], vertices[0], color)
            return
        }
        for (index in 0 until vertexCount - 1) {
            drawSegment(vertices[index], vertices[index + 1], color)
        }
        if (vertexCount > 2) {
            drawSegment(vertices[vertexCount - 1], vertices[0], color)
        }
    }

    abstract fun drawPoint(argPoint: Vec2, argRadiusOnScreen: Float, argColor: Color3f)
    abstract fun drawSolidPolygon(vertices: Array<Vec2>, vertexCount: Int, color: Color3f)
    abstract fun drawCircle(center: Vec2, radius: Float, color: Color3f)

    fun drawCircle(center: Vec2, radius: Float, axis: Vec2, color: Color3f) {
        drawCircle(center, radius, color)
    }

    abstract fun drawSolidCircle(center: Vec2, radius: Float, axis: Vec2, color: Color3f)
    abstract fun drawSegment(p1: Vec2, p2: Vec2, color: Color3f)
    abstract fun drawTransform(xf: Transform)
    abstract fun drawString(x: Float, y: Float, s: String, color: Color3f)
    abstract fun drawParticles(centers: Array<Vec2>, radius: Float, colors: Array<ParticleColor>?, count: Int)
    abstract fun drawParticlesWireframe(centers: Array<Vec2>, radius: Float, colors: Array<ParticleColor>?, count: Int)

    open fun flush() {
    }

    fun drawString(pos: Vec2, s: String, color: Color3f) {
        drawString(pos.x, pos.y, s, color)
    }

    fun getViewportTranform(): IViewportTransform? = viewportTransformInternal

    fun setCamera(x: Float, y: Float, scale: Float) {
        viewportTransformInternal?.setCamera(x, y, scale)
    }

    fun getScreenToWorldToOut(argScreen: Vec2, argWorld: Vec2) {
        viewportTransformInternal?.getScreenToWorld(argScreen, argWorld)
    }

    fun getWorldToScreenToOut(argWorld: Vec2, argScreen: Vec2) {
        viewportTransformInternal?.getWorldToScreen(argWorld, argScreen)
    }

    fun getWorldToScreenToOut(worldX: Float, worldY: Float, argScreen: Vec2) {
        argScreen.set(worldX, worldY)
        viewportTransformInternal?.getWorldToScreen(argScreen, argScreen)
    }

    fun getWorldToScreen(argWorld: Vec2): Vec2 {
        val screen = Vec2()
        viewportTransformInternal?.getWorldToScreen(argWorld, screen)
        return screen
    }

    fun getWorldToScreen(worldX: Float, worldY: Float): Vec2 {
        val argScreen = Vec2(worldX, worldY)
        viewportTransformInternal?.getWorldToScreen(argScreen, argScreen)
        return argScreen
    }

    fun getScreenToWorldToOut(screenX: Float, screenY: Float, argWorld: Vec2) {
        argWorld.set(screenX, screenY)
        viewportTransformInternal?.getScreenToWorld(argWorld, argWorld)
    }

    fun getScreenToWorld(argScreen: Vec2): Vec2 {
        val world = Vec2()
        viewportTransformInternal?.getScreenToWorld(argScreen, world)
        return world
    }

    fun getScreenToWorld(screenX: Float, screenY: Float): Vec2 {
        val screen = Vec2(screenX, screenY)
        viewportTransformInternal?.getScreenToWorld(screen, screen)
        return screen
    }

    companion object {
        const val e_shapeBit: Int = 1 shl 1
        const val e_jointBit: Int = 1 shl 2
        const val e_aabbBit: Int = 1 shl 3
        const val e_pairBit: Int = 1 shl 4
        const val e_centerOfMassBit: Int = 1 shl 5
        const val e_dynamicTreeBit: Int = 1 shl 6
        const val e_wireframeDrawingBit: Int = 1 shl 7
    }
}
