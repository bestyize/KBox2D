package xyz.thewind.kbox2d.dynamics

class TimeStep {
    var dt: Float = 0f
    var inv_dt: Float = 0f
    var dtRatio: Float = 0f
    var velocityIterations: Int = 0
    var positionIterations: Int = 0
    var warmStarting: Boolean = false
}
