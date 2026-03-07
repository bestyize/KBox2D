package xyz.thewind.kbox2d.collision.broadphase

import xyz.thewind.kbox2d.callbacks.DebugDraw
import xyz.thewind.kbox2d.callbacks.PairCallback
import xyz.thewind.kbox2d.callbacks.TreeCallback
import xyz.thewind.kbox2d.callbacks.TreeRayCastCallback
import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.collision.RayCastInput
import xyz.thewind.kbox2d.common.Vec2

class DefaultBroadPhaseBuffer(private val m_tree: BroadPhaseStrategy) : TreeCallback, BroadPhase {
    private var m_proxyCount: Int = 0
    private var m_moveBuffer: IntArray = IntArray(16)
    private var m_moveCapacity: Int = 16
    private var m_moveCount: Int = 0
    private var m_pairBuffer: LongArray = LongArray(16)
    private var m_pairCapacity: Int = 16
    private var m_pairCount: Int = 0
    private var m_queryProxyId: Int = BroadPhase.NULL_PROXY

    override fun createProxy(aabb: AABB, userData: Any?): Int {
        val proxyId = m_tree.createProxy(aabb, userData)
        m_proxyCount++
        bufferMove(proxyId)
        return proxyId
    }

    override fun destroyProxy(proxyId: Int) {
        unbufferMove(proxyId)
        m_proxyCount--
        m_tree.destroyProxy(proxyId)
    }

    override fun moveProxy(proxyId: Int, aabb: AABB, displacement: Vec2) {
        if (m_tree.moveProxy(proxyId, aabb, displacement)) {
            bufferMove(proxyId)
        }
    }

    override fun touchProxy(proxyId: Int) {
        bufferMove(proxyId)
    }

    override fun getUserData(proxyId: Int): Any? = m_tree.getUserData(proxyId)

    override fun getFatAABB(proxyId: Int): AABB = m_tree.getFatAABB(proxyId)

    override fun testOverlap(proxyIdA: Int, proxyIdB: Int): Boolean {
        val a = m_tree.getFatAABB(proxyIdA)
        val b = m_tree.getFatAABB(proxyIdB)
        if (b.lowerBound.x - a.upperBound.x > 0f || b.lowerBound.y - a.upperBound.y > 0f) {
            return false
        }
        if (a.lowerBound.x - b.upperBound.x > 0f || a.lowerBound.y - b.upperBound.y > 0f) {
            return false
        }
        return true
    }

    override fun getProxyCount(): Int = m_proxyCount

    override fun drawTree(argDraw: DebugDraw) {
        m_tree.drawTree(argDraw)
    }

    override fun updatePairs(callback: PairCallback) {
        m_pairCount = 0
        for (index in 0 until m_moveCount) {
            m_queryProxyId = m_moveBuffer[index]
            if (m_queryProxyId == BroadPhase.NULL_PROXY) {
                continue
            }
            val fatAABB = m_tree.getFatAABB(m_queryProxyId)
            m_tree.query(this, fatAABB)
        }

        m_moveCount = 0
        m_pairBuffer.sort(0, m_pairCount)

        var index = 0
        while (index < m_pairCount) {
            val primaryPair = m_pairBuffer[index]
            val userDataA = m_tree.getUserData((primaryPair shr 32).toInt())
            val userDataB = m_tree.getUserData(primaryPair.toInt())
            callback.addPair(userDataA, userDataB)
            index++
            while (index < m_pairCount && m_pairBuffer[index] == primaryPair) {
                index++
            }
        }
    }

    override fun query(callback: TreeCallback, aabb: AABB) {
        m_tree.query(callback, aabb)
    }

    override fun raycast(callback: TreeRayCastCallback, input: RayCastInput) {
        m_tree.raycast(callback, input)
    }

    override fun getTreeHeight(): Int = m_tree.getHeight()

    override fun getTreeBalance(): Int = m_tree.getMaxBalance()

    override fun getTreeQuality(): Float = m_tree.getAreaRatio()

    override fun treeCallback(proxyId: Int): Boolean {
        if (proxyId == m_queryProxyId) {
            return true
        }
        if (m_pairCount == m_pairCapacity) {
            m_pairCapacity *= 2
            m_pairBuffer = m_pairBuffer.copyOf(m_pairCapacity)
        }
        m_pairBuffer[m_pairCount] = if (proxyId < m_queryProxyId) {
            (proxyId.toLong() shl 32) or (m_queryProxyId.toLong() and 0xffffffffL)
        } else {
            (m_queryProxyId.toLong() shl 32) or (proxyId.toLong() and 0xffffffffL)
        }
        m_pairCount++
        return true
    }

    protected fun bufferMove(proxyId: Int) {
        if (m_moveCount == m_moveCapacity) {
            m_moveCapacity *= 2
            m_moveBuffer = m_moveBuffer.copyOf(m_moveCapacity)
        }
        m_moveBuffer[m_moveCount] = proxyId
        m_moveCount++
    }

    protected fun unbufferMove(proxyId: Int) {
        for (index in 0 until m_moveCount) {
            if (m_moveBuffer[index] == proxyId) {
                m_moveBuffer[index] = BroadPhase.NULL_PROXY
            }
        }
    }
}
