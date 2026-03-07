package xyz.thewind.kbox2d.dynamics.joints

import xyz.thewind.kbox2d.dynamics.Body

class JointEdge {
    var other: Body? = null
    var joint: Joint? = null
    var prev: JointEdge? = null
    var next: JointEdge? = null
}
