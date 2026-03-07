package xyz.thewind.kbox2d.dynamics.contacts

import xyz.thewind.kbox2d.dynamics.Body

class ContactEdge {
    var other: Body? = null
    var contact: Contact? = null
    var prev: ContactEdge? = null
    var next: ContactEdge? = null
}
