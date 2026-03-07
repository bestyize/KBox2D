package xyz.thewind.kbox2d.callbacks

import xyz.thewind.kbox2d.common.Settings

class ContactImpulse {
    var normalImpulses: kotlin.FloatArray = kotlin.FloatArray(Settings.maxManifoldPoints)
    var tangentImpulses: kotlin.FloatArray = kotlin.FloatArray(Settings.maxManifoldPoints)
    var count: Int = 0
}
