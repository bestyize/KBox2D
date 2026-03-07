package xyz.thewind.kbox2d.collision.broadphase

import xyz.thewind.kbox2d.callbacks.DebugDraw
import xyz.thewind.kbox2d.callbacks.TreeCallback
import xyz.thewind.kbox2d.callbacks.TreeRayCastCallback
import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.collision.RayCastInput
import xyz.thewind.kbox2d.common.Color3f
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Vec2

class DynamicTree : BroadPhaseStrategy {
    private var m_root: DynamicTreeNode? = null
    private var m_nodes: Array<DynamicTreeNode?> = arrayOfNulls(16)
    private var m_nodeCount: Int = 0
    private var m_nodeCapacity: Int = 16
    private var m_freeList: Int = 0

    private val drawVecs: Array<Vec2> = Array(4) { Vec2() }
    private var nodeStack: Array<DynamicTreeNode?> = arrayOfNulls(20)
    private var nodeStackIndex: Int = 0

    private val r = Vec2()
    private val aabb = AABB()
    private val subInput = RayCastInput()
    private val combinedAABB = AABB()
    private val color = Color3f()
    private val textVec = Vec2()

    init {
        for (index in m_nodeCapacity - 1 downTo 0) {
            val node = DynamicTreeNode(index)
            node.parent = if (index == m_nodeCapacity - 1) null else m_nodes[index + 1]
            node.height = -1
            m_nodes[index] = node
        }
    }

    override fun createProxy(aabb: AABB, userData: Any?): Int {
        assert(aabb.isValid())
        val node = allocateNode()
        val proxyId = node.id
        val nodeAABB = node.aabb
        nodeAABB.lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension
        nodeAABB.lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension
        nodeAABB.upperBound.x = aabb.upperBound.x + Settings.aabbExtension
        nodeAABB.upperBound.y = aabb.upperBound.y + Settings.aabbExtension
        node.userData = userData
        insertLeaf(proxyId)
        return proxyId
    }

    override fun destroyProxy(proxyId: Int) {
        assert(proxyId in 0 until m_nodeCapacity)
        val node = m_nodes[proxyId]!!
        assert(node.child1 == null)
        removeLeaf(node)
        freeNode(node)
    }

    override fun moveProxy(proxyId: Int, aabb: AABB, displacement: Vec2): Boolean {
        assert(aabb.isValid())
        assert(proxyId in 0 until m_nodeCapacity)
        val node = m_nodes[proxyId]!!
        assert(node.child1 == null)

        val nodeAABB = node.aabb
        if (
            nodeAABB.lowerBound.x <= aabb.lowerBound.x &&
            nodeAABB.lowerBound.y <= aabb.lowerBound.y &&
            aabb.upperBound.x <= nodeAABB.upperBound.x &&
            aabb.upperBound.y <= nodeAABB.upperBound.y
        ) {
            return false
        }

        removeLeaf(node)

        val lowerBound = nodeAABB.lowerBound
        val upperBound = nodeAABB.upperBound
        lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension
        lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension
        upperBound.x = aabb.upperBound.x + Settings.aabbExtension
        upperBound.y = aabb.upperBound.y + Settings.aabbExtension

        val dx = displacement.x * Settings.aabbMultiplier
        val dy = displacement.y * Settings.aabbMultiplier
        if (dx < 0f) {
            lowerBound.x += dx
        } else {
            upperBound.x += dx
        }
        if (dy < 0f) {
            lowerBound.y += dy
        } else {
            upperBound.y += dy
        }

        insertLeaf(proxyId)
        return true
    }

    override fun getUserData(proxyId: Int): Any? {
        assert(proxyId in 0 until m_nodeCapacity)
        return m_nodes[proxyId]!!.userData
    }

    override fun getFatAABB(proxyId: Int): AABB {
        assert(proxyId in 0 until m_nodeCapacity)
        return m_nodes[proxyId]!!.aabb
    }

    override fun query(callback: TreeCallback, aabb: AABB) {
        assert(aabb.isValid())
        nodeStackIndex = 0
        nodeStack[nodeStackIndex++] = m_root

        while (nodeStackIndex > 0) {
            val node = nodeStack[--nodeStackIndex] ?: continue
            if (AABB.testOverlap(node.aabb, aabb)) {
                if (node.child1 == null) {
                    if (!callback.treeCallback(node.id)) {
                        return
                    }
                } else {
                    ensureNodeStackCapacity(2)
                    nodeStack[nodeStackIndex++] = node.child1
                    nodeStack[nodeStackIndex++] = node.child2
                }
            }
        }
    }

    override fun raycast(callback: TreeRayCastCallback, input: RayCastInput) {
        val p1 = input.p1
        val p2 = input.p2
        val p1x = p1.x
        val p2x = p2.x
        val p1y = p1.y
        val p2y = p2.y

        r.x = p2x - p1x
        r.y = p2y - p1y
        assert(r.x * r.x + r.y * r.y > 0f)
        r.normalize()

        val rx = r.x
        val ry = r.y
        val vx = -ry
        val vy = rx
        val absVx = MathUtils.abs(vx)
        val absVy = MathUtils.abs(vy)
        var maxFraction = input.maxFraction

        val segAABB = aabb
        var tempX = (p2x - p1x) * maxFraction + p1x
        var tempY = (p2y - p1y) * maxFraction + p1y
        segAABB.lowerBound.x = if (p1x < tempX) p1x else tempX
        segAABB.lowerBound.y = if (p1y < tempY) p1y else tempY
        segAABB.upperBound.x = if (p1x > tempX) p1x else tempX
        segAABB.upperBound.y = if (p1y > tempY) p1y else tempY

        nodeStackIndex = 0
        nodeStack[nodeStackIndex++] = m_root
        while (nodeStackIndex > 0) {
            val node = nodeStack[--nodeStackIndex] ?: continue
            val nodeAABB = node.aabb
            if (!AABB.testOverlap(nodeAABB, segAABB)) {
                continue
            }

            val cx = (nodeAABB.lowerBound.x + nodeAABB.upperBound.x) * 0.5f
            val cy = (nodeAABB.lowerBound.y + nodeAABB.upperBound.y) * 0.5f
            val hx = (nodeAABB.upperBound.x - nodeAABB.lowerBound.x) * 0.5f
            val hy = (nodeAABB.upperBound.y - nodeAABB.lowerBound.y) * 0.5f
            tempX = p1x - cx
            tempY = p1y - cy
            val separation = MathUtils.abs(vx * tempX + vy * tempY) - (absVx * hx + absVy * hy)
            if (separation > 0f) {
                continue
            }

            if (node.child1 == null) {
                subInput.p1.x = p1x
                subInput.p1.y = p1y
                subInput.p2.x = p2x
                subInput.p2.y = p2y
                subInput.maxFraction = maxFraction

                val value = callback.raycastCallback(subInput, node.id)
                if (value == 0f) {
                    return
                }
                if (value > 0f) {
                    maxFraction = value
                    tempX = (p2x - p1x) * maxFraction + p1x
                    tempY = (p2y - p1y) * maxFraction + p1y
                    segAABB.lowerBound.x = if (p1x < tempX) p1x else tempX
                    segAABB.lowerBound.y = if (p1y < tempY) p1y else tempY
                    segAABB.upperBound.x = if (p1x > tempX) p1x else tempX
                    segAABB.upperBound.y = if (p1y > tempY) p1y else tempY
                }
            } else {
                ensureNodeStackCapacity(2)
                nodeStack[nodeStackIndex++] = node.child1
                nodeStack[nodeStackIndex++] = node.child2
            }
        }
    }

    override fun shiftOrigin(newOrigin: Vec2) {
        for (node in m_nodes) {
            val current = node ?: continue
            if (current.height < 0) continue
            current.aabb.lowerBound.subLocal(newOrigin)
            current.aabb.upperBound.subLocal(newOrigin)
        }
    }

    override fun computeHeight(): Int = computeHeight(m_root)

    override fun getHeight(): Int = m_root?.height ?: 0

    override fun getMaxBalance(): Int {
        var maxBalance = 0
        for (index in 0 until m_nodeCapacity) {
            val node = m_nodes[index] ?: continue
            if (node.height <= 1) {
                continue
            }
            val child1 = node.child1 ?: continue
            val child2 = node.child2 ?: continue
            val balance = MathUtils.abs(child2.height - child1.height)
            maxBalance = MathUtils.max(maxBalance, balance)
        }
        return maxBalance
    }

    override fun getAreaRatio(): Float {
        val root = m_root ?: return 0f
        val rootArea = root.aabb.getPerimeter()
        var totalArea = 0f
        for (index in 0 until m_nodeCapacity) {
            val node = m_nodes[index] ?: continue
            if (node.height < 0) {
                continue
            }
            totalArea += node.aabb.getPerimeter()
        }
        return totalArea / rootArea
    }

    fun validate() {
        validateStructure(m_root)
        validateMetrics(m_root)

        var freeCount = 0
        var freeNode = if (m_freeList != NULL_NODE) m_nodes[m_freeList] else null
        while (freeNode != null) {
            assert(freeNode.id in 0 until m_nodeCapacity)
            assert(freeNode === m_nodes[freeNode.id])
            freeNode = freeNode.parent
            freeCount++
        }

        assert(getHeight() == computeHeight())
        assert(m_nodeCount + freeCount == m_nodeCapacity)
    }

    fun rebuildBottomUp() {
        val nodes = IntArray(m_nodeCount)
        var count = 0

        for (index in 0 until m_nodeCapacity) {
            val node = m_nodes[index] ?: continue
            if (node.height < 0) {
                continue
            }
            if (node.child1 == null) {
                node.parent = null
                nodes[count++] = index
            } else {
                freeNode(node)
            }
        }

        val b = AABB()
        while (count > 1) {
            var minCost = Float.MAX_VALUE
            var iMin = -1
            var jMin = -1
            for (i in 0 until count) {
                val aabbI = m_nodes[nodes[i]]!!.aabb
                for (j in i + 1 until count) {
                    val aabbJ = m_nodes[nodes[j]]!!.aabb
                    b.combine(aabbI, aabbJ)
                    val cost = b.getPerimeter()
                    if (cost < minCost) {
                        iMin = i
                        jMin = j
                        minCost = cost
                    }
                }
            }

            val child1 = m_nodes[nodes[iMin]]!!
            val child2 = m_nodes[nodes[jMin]]!!
            val parent = allocateNode()
            parent.child1 = child1
            parent.child2 = child2
            parent.height = 1 + MathUtils.max(child1.height, child2.height)
            parent.aabb.combine(child1.aabb, child2.aabb)
            parent.parent = null
            child1.parent = parent
            child2.parent = parent

            nodes[jMin] = nodes[count - 1]
            nodes[iMin] = parent.id
            count--
        }

        m_root = m_nodes[nodes[0]]
        validate()
    }

    override fun drawTree(draw: DebugDraw) {
        val root = m_root ?: return
        val height = computeHeight()
        drawTree(draw, root, 0, height)
    }

    fun drawTree(draw: DebugDraw, node: DynamicTreeNode, spot: Int, height: Int) {
        node.aabb.getVertices(drawVecs)
        color.set(1f, (height - spot) * 1f / height, (height - spot) * 1f / height)
        draw.drawPolygon(drawVecs, 4, color)
        textVec.set(node.aabb.upperBound)
        draw.getViewportTranform()?.getWorldToScreen(node.aabb.upperBound, textVec)
        draw.drawString(textVec.x, textVec.y, "${node.id}-${spot + 1}/$height", color)
        node.child1?.let { drawTree(draw, it, spot + 1, height) }
        node.child2?.let { drawTree(draw, it, spot + 1, height) }
    }

    private fun computeHeight(node: DynamicTreeNode?): Int {
        node ?: return 0
        if (node.child1 == null) {
            return 0
        }
        val height1 = computeHeight(node.child1)
        val height2 = computeHeight(node.child2)
        return 1 + MathUtils.max(height1, height2)
    }

    private fun allocateNode(): DynamicTreeNode {
        if (m_freeList == NULL_NODE) {
            val old = m_nodes
            m_nodeCapacity *= 2
            m_nodes = arrayOfNulls(m_nodeCapacity)
            old.copyInto(m_nodes, endIndex = old.size)
            for (index in m_nodeCapacity - 1 downTo m_nodeCount) {
                val node = DynamicTreeNode(index)
                node.parent = if (index == m_nodeCapacity - 1) null else m_nodes[index + 1]
                node.height = -1
                m_nodes[index] = node
            }
            m_freeList = m_nodeCount
        }

        val nodeId = m_freeList
        val treeNode = m_nodes[nodeId]!!
        m_freeList = treeNode.parent?.id ?: NULL_NODE
        treeNode.parent = null
        treeNode.child1 = null
        treeNode.child2 = null
        treeNode.height = 0
        treeNode.userData = null
        m_nodeCount++
        return treeNode
    }

    private fun freeNode(node: DynamicTreeNode) {
        assert(m_nodeCount > 0)
        node.parent = if (m_freeList != NULL_NODE) m_nodes[m_freeList] else null
        node.height = -1
        m_freeList = node.id
        m_nodeCount--
    }

    private fun insertLeaf(leafIndex: Int) {
        val leaf = m_nodes[leafIndex]!!
        if (m_root == null) {
            m_root = leaf
            m_root!!.parent = null
            return
        }

        val leafAABB = leaf.aabb
        var index = m_root!!
        while (index.child1 != null) {
            val child1 = index.child1!!
            val child2 = index.child2!!
            val area = index.aabb.getPerimeter()
            combinedAABB.combine(index.aabb, leafAABB)
            val combinedArea = combinedAABB.getPerimeter()
            val cost = 2f * combinedArea
            val inheritanceCost = 2f * (combinedArea - area)

            val cost1 = if (child1.child1 == null) {
                combinedAABB.combine(leafAABB, child1.aabb)
                combinedAABB.getPerimeter() + inheritanceCost
            } else {
                combinedAABB.combine(leafAABB, child1.aabb)
                val oldArea = child1.aabb.getPerimeter()
                val newArea = combinedAABB.getPerimeter()
                (newArea - oldArea) + inheritanceCost
            }

            val cost2 = if (child2.child1 == null) {
                combinedAABB.combine(leafAABB, child2.aabb)
                combinedAABB.getPerimeter() + inheritanceCost
            } else {
                combinedAABB.combine(leafAABB, child2.aabb)
                val oldArea = child2.aabb.getPerimeter()
                val newArea = combinedAABB.getPerimeter()
                newArea - oldArea + inheritanceCost
            }

            if (cost < cost1 && cost < cost2) {
                break
            }
            index = if (cost1 < cost2) child1 else child2
        }

        val sibling = index
        val oldParent = sibling.parent
        val newParent = allocateNode()
        newParent.parent = oldParent
        newParent.userData = null
        newParent.aabb.combine(leafAABB, sibling.aabb)
        newParent.height = sibling.height + 1

        if (oldParent != null) {
            if (oldParent.child1 === sibling) {
                oldParent.child1 = newParent
            } else {
                oldParent.child2 = newParent
            }
            newParent.child1 = sibling
            newParent.child2 = leaf
            sibling.parent = newParent
            leaf.parent = newParent
        } else {
            newParent.child1 = sibling
            newParent.child2 = leaf
            sibling.parent = newParent
            leaf.parent = newParent
            m_root = newParent
        }

        var walk = leaf.parent
        while (walk != null) {
            walk = balance(walk)
            val child1 = walk.child1!!
            val child2 = walk.child2!!
            walk.height = 1 + MathUtils.max(child1.height, child2.height)
            walk.aabb.combine(child1.aabb, child2.aabb)
            walk = walk.parent
        }
    }

    private fun removeLeaf(leaf: DynamicTreeNode) {
        if (leaf === m_root) {
            m_root = null
            return
        }

        val parent = leaf.parent!!
        val grandParent = parent.parent
        val sibling = if (parent.child1 === leaf) parent.child2!! else parent.child1!!

        if (grandParent != null) {
            if (grandParent.child1 === parent) {
                grandParent.child1 = sibling
            } else {
                grandParent.child2 = sibling
            }
            sibling.parent = grandParent
            freeNode(parent)

            var index = grandParent
            while (index != null) {
                index = balance(index)
                val child1 = index.child1!!
                val child2 = index.child2!!
                index.aabb.combine(child1.aabb, child2.aabb)
                index.height = 1 + MathUtils.max(child1.height, child2.height)
                index = index.parent
            }
        } else {
            m_root = sibling
            sibling.parent = null
            freeNode(parent)
        }
    }

    private fun balance(nodeA: DynamicTreeNode): DynamicTreeNode {
        if (nodeA.child1 == null || nodeA.height < 2) {
            return nodeA
        }

        val nodeB = nodeA.child1!!
        val nodeC = nodeA.child2!!
        val balance = nodeC.height - nodeB.height

        if (balance > 1) {
            val nodeF = nodeC.child1!!
            val nodeG = nodeC.child2!!
            nodeC.child1 = nodeA
            nodeC.parent = nodeA.parent
            nodeA.parent = nodeC

            if (nodeC.parent != null) {
                if (nodeC.parent!!.child1 === nodeA) {
                    nodeC.parent!!.child1 = nodeC
                } else {
                    nodeC.parent!!.child2 = nodeC
                }
            } else {
                m_root = nodeC
            }

            if (nodeF.height > nodeG.height) {
                nodeC.child2 = nodeF
                nodeA.child2 = nodeG
                nodeG.parent = nodeA
                nodeA.aabb.combine(nodeB.aabb, nodeG.aabb)
                nodeC.aabb.combine(nodeA.aabb, nodeF.aabb)
                nodeA.height = 1 + MathUtils.max(nodeB.height, nodeG.height)
                nodeC.height = 1 + MathUtils.max(nodeA.height, nodeF.height)
            } else {
                nodeC.child2 = nodeG
                nodeA.child2 = nodeF
                nodeF.parent = nodeA
                nodeA.aabb.combine(nodeB.aabb, nodeF.aabb)
                nodeC.aabb.combine(nodeA.aabb, nodeG.aabb)
                nodeA.height = 1 + MathUtils.max(nodeB.height, nodeF.height)
                nodeC.height = 1 + MathUtils.max(nodeA.height, nodeG.height)
            }
            return nodeC
        }

        if (balance < -1) {
            val nodeD = nodeB.child1!!
            val nodeE = nodeB.child2!!
            nodeB.child1 = nodeA
            nodeB.parent = nodeA.parent
            nodeA.parent = nodeB

            if (nodeB.parent != null) {
                if (nodeB.parent!!.child1 === nodeA) {
                    nodeB.parent!!.child1 = nodeB
                } else {
                    nodeB.parent!!.child2 = nodeB
                }
            } else {
                m_root = nodeB
            }

            if (nodeD.height > nodeE.height) {
                nodeB.child2 = nodeD
                nodeA.child1 = nodeE
                nodeE.parent = nodeA
                nodeA.aabb.combine(nodeC.aabb, nodeE.aabb)
                nodeB.aabb.combine(nodeA.aabb, nodeD.aabb)
                nodeA.height = 1 + MathUtils.max(nodeC.height, nodeE.height)
                nodeB.height = 1 + MathUtils.max(nodeA.height, nodeD.height)
            } else {
                nodeB.child2 = nodeE
                nodeA.child1 = nodeD
                nodeD.parent = nodeA
                nodeA.aabb.combine(nodeC.aabb, nodeD.aabb)
                nodeB.aabb.combine(nodeA.aabb, nodeE.aabb)
                nodeA.height = 1 + MathUtils.max(nodeC.height, nodeD.height)
                nodeB.height = 1 + MathUtils.max(nodeA.height, nodeE.height)
            }
            return nodeB
        }

        return nodeA
    }

    private fun validateStructure(node: DynamicTreeNode?) {
        node ?: return
        if (node === m_root) {
            assert(node.parent == null)
        }
        val child1 = node.child1
        val child2 = node.child2
        if (child1 == null) {
            assert(child2 == null)
            assert(node.height == 0)
            return
        }
        assert(child2 != null)
        assert(child1.parent === node)
        assert(child2!!.parent === node)
        validateStructure(child1)
        validateStructure(child2)
    }

    private fun validateMetrics(node: DynamicTreeNode?) {
        node ?: return
        val child1 = node.child1
        val child2 = node.child2
        if (child1 == null) {
            assert(child2 == null)
            assert(node.height == 0)
            return
        }
        val resolvedChild2 = child2!!
        val height = 1 + MathUtils.max(child1.height, resolvedChild2.height)
        assert(node.height == height)
        val aabb = AABB()
        aabb.combine(child1.aabb, resolvedChild2.aabb)
        assert(aabb.lowerBound == node.aabb.lowerBound)
        assert(aabb.upperBound == node.aabb.upperBound)
        validateMetrics(child1)
        validateMetrics(resolvedChild2)
    }

    private fun ensureNodeStackCapacity(requiredSlots: Int) {
        if (nodeStack.size - nodeStackIndex - requiredSlots <= 0) {
            nodeStack = nodeStack.copyOf(nodeStack.size * 2)
        }
    }

    companion object {
        const val MAX_STACK_SIZE: Int = 64
        const val NULL_NODE: Int = -1
    }
}
