package xyz.thewind.kbox2d.pooling

import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.collision.Collision
import xyz.thewind.kbox2d.collision.Distance
import xyz.thewind.kbox2d.collision.TimeOfImpact
import xyz.thewind.kbox2d.common.Mat22
import xyz.thewind.kbox2d.common.Mat33
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.common.Vec3
import xyz.thewind.kbox2d.dynamics.contacts.Contact

interface IWorldPool {
    fun getPolyContactStack(): IDynamicStack<Contact>
    fun getCircleContactStack(): IDynamicStack<Contact>
    fun getPolyCircleContactStack(): IDynamicStack<Contact>
    fun getEdgeCircleContactStack(): IDynamicStack<Contact>
    fun getEdgePolyContactStack(): IDynamicStack<Contact>
    fun getChainCircleContactStack(): IDynamicStack<Contact>
    fun getChainPolyContactStack(): IDynamicStack<Contact>
    fun popVec2(): Vec2
    fun popVec2(num: Int): Array<Vec2>
    fun pushVec2(num: Int)
    fun popVec3(): Vec3
    fun popVec3(num: Int): Array<Vec3>
    fun pushVec3(num: Int)
    fun popMat22(): Mat22
    fun popMat22(num: Int): Array<Mat22>
    fun pushMat22(num: Int)
    fun popMat33(): Mat33
    fun pushMat33(num: Int)
    fun popAABB(): AABB
    fun popAABB(num: Int): Array<AABB>
    fun pushAABB(num: Int)
    fun popRot(): Rot
    fun pushRot(num: Int)
    fun getCollision(): Collision
    fun getTimeOfImpact(): TimeOfImpact
    fun getDistance(): Distance
    fun getFloatArray(argLength: Int): kotlin.FloatArray
    fun getIntArray(argLength: Int): kotlin.IntArray
    fun getVec2Array(argLength: Int): Array<Vec2>
}
