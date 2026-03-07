package xyz.thewind.kbox2d.callbacks

import xyz.thewind.kbox2d.dynamics.Filter
import xyz.thewind.kbox2d.dynamics.Fixture

open class ContactFilter {
    open fun shouldCollide(fixtureA: Fixture, fixtureB: Fixture): Boolean {
        val filterA: Filter = fixtureA.getFilterData()
        val filterB: Filter = fixtureB.getFilterData()
        if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0) {
            return filterA.groupIndex > 0
        }
        return (filterA.maskBits and filterB.categoryBits) != 0 &&
            (filterA.categoryBits and filterB.maskBits) != 0
    }
}
