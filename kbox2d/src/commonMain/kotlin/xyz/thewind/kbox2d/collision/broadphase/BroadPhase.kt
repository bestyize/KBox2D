package xyz.thewind.kbox2d.collision.broadphase

import xyz.thewind.kbox2d.callbacks.DebugDraw
import xyz.thewind.kbox2d.callbacks.PairCallback
import xyz.thewind.kbox2d.callbacks.TreeCallback
import xyz.thewind.kbox2d.callbacks.TreeRayCastCallback
import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.collision.RayCastInput
import xyz.thewind.kbox2d.common.Vec2

interface BroadPhase {
    fun createProxy(aabb: AABB, userData: Any?): Int
    fun destroyProxy(proxyId: Int)
    fun moveProxy(proxyId: Int, aabb: AABB, displacement: Vec2)
    fun touchProxy(proxyId: Int)
    fun getUserData(proxyId: Int): Any?
    fun getFatAABB(proxyId: Int): AABB
    fun testOverlap(proxyIdA: Int, proxyIdB: Int): Boolean
    fun getProxyCount(): Int
    fun drawTree(argDraw: DebugDraw)
    fun updatePairs(callback: PairCallback)
    fun query(callback: TreeCallback, aabb: AABB)
    fun raycast(callback: TreeRayCastCallback, input: RayCastInput)
    fun getTreeHeight(): Int
    fun getTreeBalance(): Int
    fun getTreeQuality(): Float

    companion object {
        const val NULL_PROXY: Int = -1
    }}
