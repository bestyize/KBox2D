package xyz.thewind.kbox2d.collision.broadphase

import xyz.thewind.kbox2d.collision.AABB

open class DynamicTreeNode(val id: Int) {
    val aabb: AABB = AABB()
    var userData: Any? = null
    var parent: DynamicTreeNode? = null
    var child1: DynamicTreeNode? = null
    var child2: DynamicTreeNode? = null
    var height: Int = 0
}
