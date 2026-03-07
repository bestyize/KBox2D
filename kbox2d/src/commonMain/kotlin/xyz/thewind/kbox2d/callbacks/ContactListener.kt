package xyz.thewind.kbox2d.callbacks

import xyz.thewind.kbox2d.collision.Manifold
import xyz.thewind.kbox2d.dynamics.contacts.Contact

interface ContactListener {
    fun beginContact(contact: Contact)
    fun endContact(contact: Contact)
    fun preSolve(contact: Contact, oldManifold: Manifold)
    fun postSolve(contact: Contact, impulse: ContactImpulse)
}
