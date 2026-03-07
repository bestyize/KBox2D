package xyz.thewind.kbox2d.dynamics.contacts

import xyz.thewind.kbox2d.dynamics.Fixture
import xyz.thewind.kbox2d.pooling.IWorldPool

interface ContactCreator {
    fun contactCreateFcn(argPool: IWorldPool, fixtureA: Fixture, fixtureB: Fixture): Contact
    fun contactDestroyFcn(argPool: IWorldPool, contact: Contact)
}
