package xyz.thewind.kbox2d.dynamics

import xyz.thewind.kbox2d.dynamics.contacts.Position
import xyz.thewind.kbox2d.dynamics.contacts.Velocity

class SolverData {
    var step: TimeStep? = null
    var positions: Array<Position> = emptyArray()
    var velocities: Array<Velocity> = emptyArray()
}
