package xyz.thewind.kbox2d.dynamics

import xyz.thewind.kbox2d.callbacks.ContactFilter
import xyz.thewind.kbox2d.callbacks.ContactListener
import xyz.thewind.kbox2d.callbacks.PairCallback
import xyz.thewind.kbox2d.collision.broadphase.BroadPhase
import xyz.thewind.kbox2d.dynamics.contacts.Contact
import xyz.thewind.kbox2d.dynamics.contacts.ContactEdge

class ContactManager(private val pool: World, val m_broadPhase: BroadPhase) : PairCallback {
    var m_contactList: Contact? = null
    var m_contactCount: Int = 0
    var m_contactFilter: ContactFilter = ContactFilter()
    var m_contactListener: ContactListener? = null

    override fun addPair(userDataA: Any?, userDataB: Any?) {
        val proxyA = userDataA as FixtureProxy
        val proxyB = userDataB as FixtureProxy
        var fixtureA = proxyA.fixture!!
        var fixtureB = proxyB.fixture!!
        var indexA = proxyA.childIndex
        var indexB = proxyB.childIndex
        var bodyA = fixtureA.getBody()!!
        var bodyB = fixtureB.getBody()!!
        if (bodyA === bodyB) return
        var edge: ContactEdge? = bodyB.getContactList()
        while (edge != null) {
            if (edge.other === bodyA) {
                val contact = edge.contact!!
                val fA = contact.getFixtureA()
                val fB = contact.getFixtureB()
                val iA = contact.getChildIndexA()
                val iB = contact.getChildIndexB()
                if ((fA === fixtureA && iA == indexA && fB === fixtureB && iB == indexB) ||
                    (fA === fixtureB && iA == indexB && fB === fixtureA && iB == indexA)
                ) return
            }
            edge = edge.next
        }
        if (!bodyB.shouldCollide(bodyA)) return
        if (!m_contactFilter.shouldCollide(fixtureA, fixtureB)) return
        val c = pool.popContact(fixtureA, indexA, fixtureB, indexB) ?: return
        fixtureA = c.getFixtureA()
        fixtureB = c.getFixtureB()
        indexA = c.getChildIndexA()
        indexB = c.getChildIndexB()
        bodyA = fixtureA.getBody()!!
        bodyB = fixtureB.getBody()!!
        c.m_prev = null
        c.m_next = m_contactList
        m_contactList?.m_prev = c
        m_contactList = c
        c.m_nodeA.contact = c
        c.m_nodeA.other = bodyB
        c.m_nodeA.prev = null
        c.m_nodeA.next = bodyA.m_contactList
        bodyA.m_contactList?.prev = c.m_nodeA
        bodyA.m_contactList = c.m_nodeA
        c.m_nodeB.contact = c
        c.m_nodeB.other = bodyA
        c.m_nodeB.prev = null
        c.m_nodeB.next = bodyB.m_contactList
        bodyB.m_contactList?.prev = c.m_nodeB
        bodyB.m_contactList = c.m_nodeB
        if (!fixtureA.isSensor() && !fixtureB.isSensor()) {
            bodyA.setAwake(true)
            bodyB.setAwake(true)
        }
        m_contactCount++
    }

    fun findNewContacts() {
        m_broadPhase.updatePairs(this)
    }

    fun destroy(c: Contact) {
        val fixtureA = c.getFixtureA()
        val fixtureB = c.getFixtureB()
        val bodyA = fixtureA.getBody()!!
        val bodyB = fixtureB.getBody()!!
        if (m_contactListener != null && c.isTouching()) {
            m_contactListener!!.endContact(c)
        }
        c.m_prev?.m_next = c.m_next
        c.m_next?.m_prev = c.m_prev
        if (c === m_contactList) {
            m_contactList = c.m_next
        }
        c.m_nodeA.prev?.next = c.m_nodeA.next
        c.m_nodeA.next?.prev = c.m_nodeA.prev
        if (c.m_nodeA === bodyA.m_contactList) {
            bodyA.m_contactList = c.m_nodeA.next
        }
        c.m_nodeB.prev?.next = c.m_nodeB.next
        c.m_nodeB.next?.prev = c.m_nodeB.prev
        if (c.m_nodeB === bodyB.m_contactList) {
            bodyB.m_contactList = c.m_nodeB.next
        }
        pool.pushContact(c)
        m_contactCount--
    }

    fun collide() {
        var c = m_contactList
        while (c != null) {
            val fixtureA = c.getFixtureA()
            val fixtureB = c.getFixtureB()
            val indexA = c.getChildIndexA()
            val indexB = c.getChildIndexB()
            val bodyA = fixtureA.getBody()!!
            val bodyB = fixtureB.getBody()!!
            if ((c.m_flags and Contact.FILTER_FLAG) == Contact.FILTER_FLAG) {
                if (!bodyB.shouldCollide(bodyA) || !m_contactFilter.shouldCollide(fixtureA, fixtureB)) {
                    val cNuke = c
                    c = cNuke.getNext()
                    destroy(cNuke)
                    continue
                }
                c.m_flags = c.m_flags and Contact.FILTER_FLAG.inv()
            }
            val activeA = bodyA.isAwake() && bodyA.m_type != BodyType.STATIC
            val activeB = bodyB.isAwake() && bodyB.m_type != BodyType.STATIC
            if (!activeA && !activeB) {
                c = c.getNext()
                continue
            }
            val proxyIdA = fixtureA.m_proxies!![indexA].proxyId
            val proxyIdB = fixtureB.m_proxies!![indexB].proxyId
            val overlap = m_broadPhase.testOverlap(proxyIdA, proxyIdB)
            if (!overlap) {
                val cNuke = c
                c = cNuke.getNext()
                destroy(cNuke)
                continue
            }
            c.update(m_contactListener)
            c = c.getNext()
        }
    }
}
