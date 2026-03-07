package xyz.thewind.kbox2d.dynamics

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.common.Vec2

class ContactManagerTest {
    @Test
    fun findNewContactsCreatesCircleContactForOverlappingFixtures() {
        val world = World()
        val bodyA = Body(world, BodyType.DYNAMIC)
        val bodyB = Body(world, BodyType.DYNAMIC)
        bodyA.m_xf.p.set(0f, 0f)
        bodyB.m_xf.p.set(1f, 0f)

        val fixtureA = bodyA.createFixture(FixtureDef().apply {
            shape = CircleShape().apply { m_radius = 1f }
            density = 1f
        })!!
        val fixtureB = bodyB.createFixture(FixtureDef().apply {
            shape = CircleShape().apply { m_radius = 1f }
            density = 1f
        })!!

        world.m_contactManager.findNewContacts()
        world.m_contactManager.collide()

        val contact = world.m_contactManager.m_contactList
        assertNotNull(contact)
        assertTrue(contact.getFixtureA() === fixtureA || contact.getFixtureB() === fixtureA)
        assertTrue(contact.getFixtureA() === fixtureB || contact.getFixtureB() === fixtureB)
        assertEquals(1, world.m_contactManager.m_contactCount)
        assertTrue(contact.m_manifold.pointCount > 0)
    }
}
