package xyz.thewind.kbox2d.collision

import xyz.thewind.kbox2d.common.Transform

class DistanceInput {
    var proxyA: Distance.DistanceProxy = Distance.DistanceProxy()
    var proxyB: Distance.DistanceProxy = Distance.DistanceProxy()
    var transformA: Transform = Transform()
    var transformB: Transform = Transform()
    var useRadii: Boolean = false
}
