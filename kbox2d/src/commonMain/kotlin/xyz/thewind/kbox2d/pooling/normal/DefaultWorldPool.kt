package xyz.thewind.kbox2d.pooling.normal

import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.collision.Collision
import xyz.thewind.kbox2d.collision.Distance
import xyz.thewind.kbox2d.collision.TimeOfImpact
import xyz.thewind.kbox2d.common.Mat22
import xyz.thewind.kbox2d.common.Mat33
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.common.Vec3
import xyz.thewind.kbox2d.dynamics.contacts.ChainAndCircleContact
import xyz.thewind.kbox2d.dynamics.contacts.ChainAndPolygonContact
import xyz.thewind.kbox2d.dynamics.contacts.CircleContact
import xyz.thewind.kbox2d.dynamics.contacts.Contact
import xyz.thewind.kbox2d.dynamics.contacts.EdgeAndCircleContact
import xyz.thewind.kbox2d.dynamics.contacts.EdgeAndPolygonContact
import xyz.thewind.kbox2d.dynamics.contacts.PolygonAndCircleContact
import xyz.thewind.kbox2d.dynamics.contacts.PolygonContact
import xyz.thewind.kbox2d.pooling.IDynamicStack
import xyz.thewind.kbox2d.pooling.IWorldPool

class DefaultWorldPool(argSize: Int, argContainerSize: Int) : IWorldPool {
    private val vecs = object : OrderedStack<Vec2>(argSize, argContainerSize) {
        override fun newInstance(): Vec2 = Vec2()
        override fun newArray(size: Int): Array<Vec2> = Array(size) { Vec2() }
    }
    private val vec3s = object : OrderedStack<Vec3>(argSize, argContainerSize) {
        override fun newInstance(): Vec3 = Vec3()
        override fun newArray(size: Int): Array<Vec3> = Array(size) { Vec3() }
    }
    private val mats = object : OrderedStack<Mat22>(argSize, argContainerSize) {
        override fun newInstance(): Mat22 = Mat22()
        override fun newArray(size: Int): Array<Mat22> = Array(size) { Mat22() }
    }
    private val mat33s = object : OrderedStack<Mat33>(argSize, argContainerSize) {
        override fun newInstance(): Mat33 = Mat33()
        override fun newArray(size: Int): Array<Mat33> = Array(size) { Mat33() }
    }
    private val aabbs = object : OrderedStack<AABB>(argSize, argContainerSize) {
        override fun newInstance(): AABB = AABB()
        override fun newArray(size: Int): Array<AABB> = Array(size) { AABB() }
    }
    private val rots = object : OrderedStack<Rot>(argSize, argContainerSize) {
        override fun newInstance(): Rot = Rot()
        override fun newArray(size: Int): Array<Rot> = Array(size) { Rot() }
    }

    private val afloats = mutableMapOf<Int, kotlin.FloatArray>()
    private val aints = mutableMapOf<Int, kotlin.IntArray>()
    private val avecs = mutableMapOf<Int, Array<Vec2>>()

    private val world: IWorldPool = this

    private val pcstack = object : MutableStack<Contact>(16) {
        override fun newInstance(): Contact = PolygonContact(world)
        override fun newArray(size: Int): Array<Contact> = Array(size) { PolygonContact(world) }
    }
    private val ccstack = object : MutableStack<Contact>(16) {
        override fun newInstance(): Contact = CircleContact(world)
        override fun newArray(size: Int): Array<Contact> = Array(size) { CircleContact(world) }
    }
    private val cpstack = object : MutableStack<Contact>(16) {
        override fun newInstance(): Contact = PolygonAndCircleContact(world)
        override fun newArray(size: Int): Array<Contact> = Array(size) { PolygonAndCircleContact(world) }
    }
    private val ecstack = object : MutableStack<Contact>(16) {
        override fun newInstance(): Contact = EdgeAndCircleContact(world)
        override fun newArray(size: Int): Array<Contact> = Array(size) { EdgeAndCircleContact(world) }
    }
    private val epstack = object : MutableStack<Contact>(16) {
        override fun newInstance(): Contact = EdgeAndPolygonContact(world)
        override fun newArray(size: Int): Array<Contact> = Array(size) { EdgeAndPolygonContact(world) }
    }
    private val chcstack = object : MutableStack<Contact>(16) {
        override fun newInstance(): Contact = ChainAndCircleContact(world)
        override fun newArray(size: Int): Array<Contact> = Array(size) { ChainAndCircleContact(world) }
    }
    private val chpstack = object : MutableStack<Contact>(16) {
        override fun newInstance(): Contact = ChainAndPolygonContact(world)
        override fun newArray(size: Int): Array<Contact> = Array(size) { ChainAndPolygonContact(world) }
    }

    private val dist = Distance()
    private val collision = Collision(dist)
    private val toi = TimeOfImpact()

    override fun getPolyContactStack(): IDynamicStack<Contact> = pcstack
    override fun getCircleContactStack(): IDynamicStack<Contact> = ccstack
    override fun getPolyCircleContactStack(): IDynamicStack<Contact> = cpstack
    override fun getEdgeCircleContactStack(): IDynamicStack<Contact> = ecstack
    override fun getEdgePolyContactStack(): IDynamicStack<Contact> = epstack
    override fun getChainCircleContactStack(): IDynamicStack<Contact> = chcstack
    override fun getChainPolyContactStack(): IDynamicStack<Contact> = chpstack
    override fun popVec2(): Vec2 = vecs.pop()
    override fun popVec2(num: Int): Array<Vec2> = vecs.pop(num)
    override fun pushVec2(num: Int) { vecs.push(num) }
    override fun popVec3(): Vec3 = vec3s.pop()
    override fun popVec3(num: Int): Array<Vec3> = vec3s.pop(num)
    override fun pushVec3(num: Int) { vec3s.push(num) }
    override fun popMat22(): Mat22 = mats.pop()
    override fun popMat22(num: Int): Array<Mat22> = mats.pop(num)
    override fun pushMat22(num: Int) { mats.push(num) }
    override fun popMat33(): Mat33 = mat33s.pop()
    override fun pushMat33(num: Int) { mat33s.push(num) }
    override fun popAABB(): AABB = aabbs.pop()
    override fun popAABB(num: Int): Array<AABB> = aabbs.pop(num)
    override fun pushAABB(num: Int) { aabbs.push(num) }
    override fun popRot(): Rot = rots.pop()
    override fun pushRot(num: Int) { rots.push(num) }
    override fun getCollision(): Collision = collision
    override fun getTimeOfImpact(): TimeOfImpact = toi
    override fun getDistance(): Distance = dist

    override fun getFloatArray(argLength: Int): kotlin.FloatArray =
        afloats.getOrPut(argLength) { kotlin.FloatArray(argLength) }

    override fun getIntArray(argLength: Int): kotlin.IntArray =
        aints.getOrPut(argLength) { kotlin.IntArray(argLength) }

    override fun getVec2Array(argLength: Int): Array<Vec2> =
        avecs.getOrPut(argLength) { Array(argLength) { Vec2() } }
}
