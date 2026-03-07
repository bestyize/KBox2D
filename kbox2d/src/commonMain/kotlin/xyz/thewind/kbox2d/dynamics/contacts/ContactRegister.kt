package xyz.thewind.kbox2d.dynamics.contacts

import xyz.thewind.kbox2d.pooling.IDynamicStack

class ContactRegister {
    var creator: IDynamicStack<Contact>? = null
    var primary: Boolean = false
}
