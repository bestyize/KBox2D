package xyz.thewind.kbox2d.callbacks

import kotlin.test.Test
import kotlin.test.assertFalse
import kotlin.test.assertTrue
import xyz.thewind.kbox2d.dynamics.Filter
import xyz.thewind.kbox2d.dynamics.Fixture

class ContactFilterTest {
    @Test
    fun positiveGroupIndexAlwaysCollides() {
        val fixtureA = Fixture()
        val fixtureB = Fixture()
        fixtureA.setFilterData(Filter().apply { groupIndex = 2 })
        fixtureB.setFilterData(Filter().apply { groupIndex = 2 })

        assertTrue(ContactFilter().shouldCollide(fixtureA, fixtureB))
    }

    @Test
    fun maskFilteringCanRejectCollision() {
        val fixtureA = Fixture()
        val fixtureB = Fixture()
        fixtureA.setFilterData(Filter().apply {
            categoryBits = 0x0002
            maskBits = 0x0004
        })
        fixtureB.setFilterData(Filter().apply {
            categoryBits = 0x0008
            maskBits = 0x0002
        })

        assertFalse(ContactFilter().shouldCollide(fixtureA, fixtureB))
    }
}
