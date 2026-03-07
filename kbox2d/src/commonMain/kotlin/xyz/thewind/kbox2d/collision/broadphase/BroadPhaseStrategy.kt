package xyz.thewind.kbox2d.collision.broadphase

import xyz.thewind.kbox2d.callbacks.DebugDraw
import xyz.thewind.kbox2d.callbacks.TreeCallback
import xyz.thewind.kbox2d.callbacks.TreeRayCastCallback
import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.collision.RayCastInput
import xyz.thewind.kbox2d.common.Vec2

interface BroadPhaseStrategy {
    fun createProxy(aabb: AABB, userData: Any?): Int
    fun destroyProxy(proxyId: Int)
    fun moveProxy(proxyId: Int, aabb: AABB, displacement: Vec2): Boolean
    fun getUserData(proxyId: Int): Any?
    fun getFatAABB(proxyId: Int): AABB
    fun query(callback: TreeCallback, aabb: AABB)
    fun raycast(callback: TreeRayCastCallback, input: RayCastInput)
    fun computeHeight(): Int
    fun getHeight(): Int
    fun getMaxBalance(): Int
    fun getAreaRatio(): Float
    fun drawTree(draw: DebugDraw)
}
