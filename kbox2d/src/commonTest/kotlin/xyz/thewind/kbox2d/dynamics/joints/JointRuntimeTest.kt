package xyz.thewind.kbox2d.dynamics.joints

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.BodyDef
import xyz.thewind.kbox2d.dynamics.BodyType
import xyz.thewind.kbox2d.dynamics.FixtureDef
import xyz.thewind.kbox2d.dynamics.World

class JointRuntimeTest {
    @Test
    fun createAndDestroyDistanceJointUpdatesWorldAndBodyLists() {
        val world = World()
        val bodyA = world.createBody(BodyDef().apply { type = BodyType.DYNAMIC })!!
        val bodyB = world.createBody(BodyDef().apply { type = BodyType.DYNAMIC })!!
        val joint = world.createJoint(DistanceJointDef().apply {
            initialize(bodyA, bodyB, Vec2(0f, 0f), Vec2(1f, 0f))
        })
        assertNotNull(joint)
        assertEquals(1, world.getJointCount())
        assertTrue(bodyA.m_jointList?.joint === joint)
        assertTrue(bodyB.m_jointList?.joint === joint)

        world.destroyJoint(joint)

        assertEquals(0, world.getJointCount())
        assertEquals(null, bodyA.m_jointList)
        assertEquals(null, bodyB.m_jointList)
    }

    @Test
    fun distanceJointKeepsBodiesRoughlyTogetherUnderGravity() {
        val world = World(gravity = Vec2(0f, -10f))
        val bodyA = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(0f, 10f)
        })!!
        val bodyB = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(3f, 10f)
        })!!
        for (body in listOf(bodyA, bodyB)) {
            body.createFixture(FixtureDef().apply {
                shape = CircleShape().apply { m_radius = 0.5f }
                density = 1f
            })
        }
        world.createJoint(DistanceJointDef().apply {
            initialize(bodyA, bodyB, bodyA.getPosition().clone(), bodyB.getPosition().clone())
        })


        repeat(30) {
            world.step(1f / 60f, 8, 3)
        }

        val dx = bodyB.getPosition().x - bodyA.getPosition().x
        val dy = bodyB.getPosition().y - bodyA.getPosition().y
        val distance = kotlin.math.sqrt(dx * dx + dy * dy)
        assertTrue(distance > 2.5f)
        assertTrue(distance < 3.5f)
    }
    @Test
    fun ropeJointLimitsMaximumDistance() {
        val world = World(gravity = Vec2(0f, 0f))
        val bodyA = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(0f, 0f)
        })!!
        val bodyB = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(5f, 0f)
        })!!
        world.createJoint(RopeJointDef().apply {
            this.bodyA = bodyA
            this.bodyB = bodyB
            maxLength = 2f
        })

        repeat(20) {
            world.step(1f / 60f, 8, 3)
        }

        val dx = bodyB.getPosition().x - bodyA.getPosition().x
        val dy = bodyB.getPosition().y - bodyA.getPosition().y
        val distance = kotlin.math.sqrt(dx * dx + dy * dy)
        assertTrue(distance <= 2.1f)
    }

    @Test
    fun motorJointDrivesBodiesTowardOffset() {
        val world = World(gravity = Vec2(0f, 0f))
        val bodyA = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(0f, 0f)
        })!!
        val bodyB = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(5f, 0f)
        })!!
        val joint = world.createJoint(MotorJointDef().apply {
            initialize(bodyA, bodyB)
            linearOffset.set(1f, 0f)
            maxForce = 100f
            maxTorque = 100f
            correctionFactor = 0.8f
        }) as MotorJoint

        repeat(30) {
            world.step(1f / 60f, 8, 3)
        }

        val deltaX = bodyB.getPosition().x - bodyA.getPosition().x
        assertTrue(deltaX < 5f)
        assertEquals(1f, joint.getLinearOffset().x, 1e-5f)
    }

    @Test
    fun frictionJointReducesRelativeMotion() {
        val world = World(gravity = Vec2(0f, 0f))
        val bodyA = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(0f, 0f)
            linearVelocity = Vec2(5f, 0f)
        })!!
        val bodyB = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(0f, 0f)
            linearVelocity = Vec2(0f, 0f)
        })!!
        world.createJoint(FrictionJointDef().apply {
            initialize(bodyA, bodyB, Vec2(0f, 0f))
            maxForce = 50f
            maxTorque = 50f
        })

        repeat(20) {
            world.step(1f / 60f, 8, 3)
        }

        assertTrue(bodyA.getLinearVelocity().x < 5f)
        assertTrue(bodyB.getLinearVelocity().x > 0f)
    }

    @Test
    fun mouseJointPullsBodyTowardTarget() {
        val world = World(gravity = Vec2(0f, 0f))
        val ground = world.createBody(BodyDef().apply {
            type = BodyType.STATIC
        })!!
        val body = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(10f, 0f)
        })!!
        val joint = world.createJoint(MouseJointDef().apply {
            this.bodyA = ground
            this.bodyB = body
            target.set(10f, 0f)
            maxForce = 500f
            frequencyHz = 5f
            dampingRatio = 0.7f
        }) as MouseJoint

        joint.setTarget(Vec2(0f, 0f))

        repeat(30) {
            world.step(1f / 60f, 8, 3)
        }

        assertTrue(body.getPosition().x < 10f)
        assertEquals(0f, joint.getTarget().x, 1e-5f)
    }

    @Test
    fun weldJointPreservesRelativeOffset() {
        val world = World(gravity = Vec2(0f, -10f))
        val bodyA = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(0f, 10f)
        })!!
        val bodyB = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(2f, 10f)
        })!!
        for (body in listOf(bodyA, bodyB)) {
            body.createFixture(FixtureDef().apply {
                shape = CircleShape().apply { m_radius = 0.5f }
                density = 1f
            })
        }
        world.createJoint(WeldJointDef().apply {
            initialize(bodyA, bodyB, Vec2(1f, 10f))
        })

        repeat(30) {
            world.step(1f / 60f, 8, 3)
        }

        val dx = bodyB.getPosition().x - bodyA.getPosition().x
        val dy = bodyB.getPosition().y - bodyA.getPosition().y
        val distance = kotlin.math.sqrt(dx * dx + dy * dy)
        assertTrue(distance > 1.5f)
        assertTrue(distance < 2.5f)
    }


    @Test
    fun revoluteJointMotorAcceleratesRotation() {
        val world = World(gravity = Vec2(0f, 0f))
        val ground = world.createBody(BodyDef().apply {
            type = BodyType.STATIC
            position = Vec2(0f, 0f)
        })!!
        val body = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(0f, 0f)
        })!!
        body.createFixture(FixtureDef().apply {
            shape = CircleShape().apply { m_radius = 0.5f }
            density = 1f
        })
        val joint = world.createJoint(RevoluteJointDef().apply {
            initialize(ground, body, Vec2(0f, 0f))
            enableMotor = true
            motorSpeed = 4f
            maxMotorTorque = 200f
        }) as RevoluteJoint

        repeat(30) {
            world.step(1f / 60f, 8, 3)
        }

        assertTrue(joint.getJointSpeed() > 0.5f)
        assertTrue(body.getAngularVelocity() > 0.5f)
    }

    @Test
    fun prismaticJointMotorRespectsUpperLimit() {
        val world = World(gravity = Vec2(0f, 0f))
        val ground = world.createBody(BodyDef().apply {
            type = BodyType.STATIC
            position = Vec2(0f, 0f)
        })!!
        val body = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(0f, 0f)
        })!!
        body.createFixture(FixtureDef().apply {
            shape = CircleShape().apply { m_radius = 0.5f }
            density = 1f
        })
        val joint = world.createJoint(PrismaticJointDef().apply {
            initialize(ground, body, Vec2(0f, 0f), Vec2(1f, 0f))
            enableMotor = true
            motorSpeed = 6f
            maxMotorForce = 500f
            enableLimit = true
            lowerTranslation = 0f
            upperTranslation = 2f
        }) as PrismaticJoint

        repeat(60) {
            world.step(1f / 60f, 8, 3)
        }

        assertTrue(body.getPosition().x > 0.5f)
        assertTrue(joint.getJointTranslation() <= 2.1f)
    }


    @Test
    fun pulleyJointPreservesCombinedCableLength() {
        val world = World(gravity = Vec2(0f, -10f))
        val bodyA = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(-2f, 2f)
        })!!
        val bodyB = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(2f, 2f)
        })!!
        for (body in listOf(bodyA, bodyB)) {
            body.createFixture(FixtureDef().apply {
                shape = CircleShape().apply { m_radius = 0.5f }
                density = 1f
            })
        }
        val joint = world.createJoint(PulleyJointDef().apply {
            initialize(
                bodyA,
                bodyB,
                Vec2(-2f, 5f),
                Vec2(2f, 5f),
                bodyA.getPosition().clone(),
                bodyB.getPosition().clone(),
                1f,
            )
        }) as PulleyJoint
        val target = joint.getLengthA() + joint.getRatio() * joint.getLengthB()

        repeat(60) {
            world.step(1f / 60f, 8, 3)
        }

        val combined = joint.getCurrentLengthA() + joint.getRatio() * joint.getCurrentLengthB()
        assertTrue(kotlin.math.abs(combined - target) < 0.3f)
    }


    @Test
    fun wheelJointMotorSpinsBody() {
        val world = World(gravity = Vec2(0f, 0f))
        val ground = world.createBody(BodyDef().apply {
            type = BodyType.STATIC
            position = Vec2(0f, 0f)
        })!!
        val body = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(0f, 0f)
        })!!
        body.createFixture(FixtureDef().apply {
            shape = CircleShape().apply { m_radius = 0.5f }
            density = 1f
        })
        val joint = world.createJoint(WheelJointDef().apply {
            initialize(ground, body, Vec2(0f, 0f), Vec2(0f, 1f))
            enableMotor = true
            motorSpeed = 5f
            maxMotorTorque = 200f
            frequencyHz = 2f
            dampingRatio = 0.7f
        }) as WheelJoint

        repeat(30) {
            world.step(1f / 60f, 8, 3)
        }

        assertTrue(kotlin.math.abs(joint.getJointSpeed()) > 0.5f)
        assertTrue(kotlin.math.abs(body.getAngularVelocity()) > 0.5f)
    }


    @Test
    fun gearJointTransfersRotationRatio() {
        val world = World(gravity = Vec2(0f, 0f))
        val ground = world.createBody(BodyDef().apply {
            type = BodyType.STATIC
        })!!
        val bodyA = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(-2f, 0f)
        })!!
        val bodyB = world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(2f, 0f)
        })!!
        for (body in listOf(bodyA, bodyB)) {
            body.createFixture(FixtureDef().apply {
                shape = CircleShape().apply { m_radius = 0.5f }
                density = 1f
            })
        }
        val joint1 = world.createJoint(RevoluteJointDef().apply {
            initialize(ground, bodyA, bodyA.getPosition().clone())
            enableMotor = true
            motorSpeed = 6f
            maxMotorTorque = 200f
        }) as RevoluteJoint
        val joint2 = world.createJoint(RevoluteJointDef().apply {
            initialize(ground, bodyB, bodyB.getPosition().clone())
        }) as RevoluteJoint
        world.createJoint(GearJointDef().apply {
            this.joint1 = joint1
            this.joint2 = joint2
            ratio = 2f
        }) as GearJoint

        repeat(60) {
            world.step(1f / 60f, 8, 3)
        }

        val coupled = joint1.getJointAngle() + 2f * joint2.getJointAngle()
        assertTrue(kotlin.math.abs(joint2.getJointAngle()) > 0.1f)
        assertTrue(kotlin.math.abs(coupled) < 0.25f)
    }

    @Test
    fun constantVolumeJointMaintainsAreaAndDestroysInternalJoints() {
        fun polygonArea(points: List<Vec2>): Float {
            var area = 0f
            for (i in points.indices) {
                val next = if (i == points.lastIndex) 0 else i + 1
                area += points[i].x * points[next].y - points[next].x * points[i].y
            }
            return kotlin.math.abs(area * 0.5f)
        }

        val world = World(gravity = Vec2(0f, 0f))
        val bodies = listOf(
            world.createBody(BodyDef().apply { type = BodyType.DYNAMIC; position = Vec2(-1f, 0f) })!!,
            world.createBody(BodyDef().apply { type = BodyType.DYNAMIC; position = Vec2(1f, 0f) })!!,
            world.createBody(BodyDef().apply { type = BodyType.DYNAMIC; position = Vec2(0f, 1.6f) })!!,
        )
        for (body in bodies) {
            body.createFixture(FixtureDef().apply {
                shape = CircleShape().apply { m_radius = 0.35f }
                density = 1f
            })
        }
        val initialArea = polygonArea(bodies.map { it.getWorldCenter().clone() })
        val joint = world.createJoint(ConstantVolumeJointDef().apply {
            frequencyHz = 8f
            dampingRatio = 0.7f
            bodies.forEach { addBody(it) }
        }) as ConstantVolumeJoint

        assertEquals(4, world.getJointCount())
        bodies[0].setLinearVelocity(Vec2(4f, 0f))
        repeat(60) {
            world.step(1f / 60f, 8, 3)
        }

        val finalArea = polygonArea(bodies.map { it.getWorldCenter().clone() })
        assertTrue(kotlin.math.abs(finalArea - initialArea) < 0.8f)

        world.destroyJoint(joint)
        assertEquals(0, world.getJointCount())
    }

}
