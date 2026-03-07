package xyz.thewind.kbox2d.callbacks

import xyz.thewind.kbox2d.dynamics.Fixture
import xyz.thewind.kbox2d.dynamics.joints.Joint

interface DestructionListener {
    fun sayGoodbye(joint: Joint)
    fun sayGoodbye(fixture: Fixture)
}
