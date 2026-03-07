package xyz.thewind.kbox2d.dynamics.joints

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.BodyDef
import xyz.thewind.kbox2d.dynamics.BodyType
import xyz.thewind.kbox2d.dynamics.World

class JointDefTest {
    @Test
    fun distanceJointDefComputesLocalAnchorsAndLength() {
        val world = World()
        val bodyA = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(1f, 0f)
        })
        val bodyB = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(4f, 0f)
        })
        assertNotNull(bodyA)
        assertNotNull(bodyB)

        val def = DistanceJointDef()
        def.initialize(bodyA, bodyB, Vec2(2f, 0f), Vec2(5f, 0f))

        assertEquals(1f, def.localAnchorA.x, 1e-5f)
        assertEquals(1f, def.localAnchorB.x, 1e-5f)
        assertEquals(3f, def.length, 1e-5f)
    }

    @Test
    fun motorAndPrismaticDefsUseBodyTransforms() {
        val world = World()
        val bodyA = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(2f, 3f)
            angle = MathUtils.HALF_PI
        })
        val bodyB = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(2f, 5f)
            angle = MathUtils.PI
        })
        assertNotNull(bodyA)
        assertNotNull(bodyB)

        val motorDef = MotorJointDef()
        motorDef.initialize(bodyA, bodyB)
        assertEquals(2f, motorDef.linearOffset.x, 1e-5f)
        assertEquals(0f, motorDef.linearOffset.y, 1e-5f)
        assertEquals(MathUtils.HALF_PI, motorDef.angularOffset, 1e-5f)

        val prismaticDef = PrismaticJointDef()
        prismaticDef.initialize(bodyA, bodyB, Vec2(2f, 4f), Vec2(0f, 1f))
        assertEquals(1f, prismaticDef.localAnchorA.x, 1e-5f)
        assertEquals(0f, prismaticDef.localAnchorA.y, 1e-5f)
        assertEquals(1f, prismaticDef.localAxisA.x, 1e-5f)
        assertEquals(0f, prismaticDef.localAxisA.y, 1e-5f)
        assertEquals(MathUtils.HALF_PI, prismaticDef.referenceAngle, 1e-5f)
    }
}
