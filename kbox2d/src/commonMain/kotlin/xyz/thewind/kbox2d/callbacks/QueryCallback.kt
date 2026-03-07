package xyz.thewind.kbox2d.callbacks

import xyz.thewind.kbox2d.dynamics.Fixture

fun interface QueryCallback {
    fun reportFixture(fixture: Fixture): Boolean
}
