package xyz.thewind.kbox2d.callbacks

import xyz.thewind.kbox2d.collision.RayCastInput

fun interface TreeRayCastCallback {
    fun raycastCallback(input: RayCastInput, nodeId: Int): Float
}
