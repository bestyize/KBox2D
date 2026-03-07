package xyz.thewind.kbox2d.particle

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertFalse
import kotlin.test.assertTrue
import xyz.thewind.kbox2d.callbacks.ParticleDestructionListener
import xyz.thewind.kbox2d.callbacks.ParticleQueryCallback
import xyz.thewind.kbox2d.callbacks.ParticleRaycastCallback
import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.TimeStep
import xyz.thewind.kbox2d.dynamics.World

class ParticleSystemTest {
    @Test
    fun worldCreatesAndStepsParticles() {
        val world = World(gravity = Vec2(0f, -10f))
        val index = world.createParticle(ParticleDef().apply {
            position.set(1f, 5f)
            velocity.set(0f, 0f)
        })
        assertEquals(0, index)
        assertEquals(1, world.getParticleCount())

        val startY = world.getParticlePositionBuffer()[0].y
        world.step(1f / 60f, 8, 3)

        assertTrue(world.getParticlePositionBuffer()[0].y < startY)
        assertTrue(world.getParticleVelocityBuffer()[0].y < 0f)
    }

    @Test
    fun worldCreatesParticleGroup() {
        val world = World()
        val group = world.createParticleGroup(ParticleGroupDef().apply {
            position.set(2f, 3f)
            linearVelocity.set(1f, 0f)
        })

        assertEquals(1, group.getParticleCount())
        assertNotNull(world.getParticleGroupList())
        assertEquals(1, world.getParticleCount())
    }

    @Test
    fun particleGroupUsesShapeToCreateMultipleParticles() {
        val world = World()
        world.setParticleRadius(0.3f)
        val group = world.createParticleGroup(ParticleGroupDef().apply {
            shape = CircleShape().apply {
                m_p.setZero()
                m_radius = 0.7f
            }
            position.set(0f, 0f)
        })

        assertTrue(group.getParticleCount() > 1)
        assertEquals(group.getParticleCount(), world.getParticleCount())
    }


    @Test
    fun rigidParticleGroupPreservesRelativeShape() {
        val world = World(gravity = Vec2(0f, 0f))
        world.setParticleRadius(0.25f)
        val group = world.createParticleGroup(ParticleGroupDef().apply {
            groupFlags = ParticleGroupType.b2_rigidParticleGroup
            shape = CircleShape().apply {
                m_p.setZero()
                m_radius = 0.8f
            }
            position.set(0f, 0f)
            linearVelocity.set(2f, 0f)
            angularVelocity = 3f
        })
        val before = world.getParticlePositionBuffer().map { it.clone() }
        val initialDistance = before[0].sub(before[1]).length()

        repeat(20) {
            world.step(1f / 60f, 8, 3)
        }

        val after = world.getParticlePositionBuffer()
        val finalDistance = after[0].sub(after[1]).length()
        assertTrue(group.getParticleCount() > 1)
        assertTrue(kotlin.math.abs(finalDistance - initialDistance) < 0.1f)
    }


    @Test
    fun worldCanJoinParticleGroups() {
        val world = World()
        world.setParticleRadius(0.25f)
        val groupA = world.createParticleGroup(ParticleGroupDef().apply {
            shape = CircleShape().apply { m_radius = 0.4f }
            position.set(-1f, 0f)
        })
        val groupB = world.createParticleGroup(ParticleGroupDef().apply {
            shape = CircleShape().apply { m_radius = 0.4f }
            position.set(1f, 0f)
        })
        val total = groupA.getParticleCount() + groupB.getParticleCount()

        world.joinParticleGroups(groupA, groupB)

        assertEquals(1, world.getParticleGroupCount())
        assertEquals(total, groupA.getParticleCount())
        assertTrue(world.getParticleGroupBuffer().count { it === groupA } >= total)
    }

    @Test
    fun collisionEnergyIsPositiveForApproachingParticles() {
        val world = World(gravity = Vec2(0f, 0f))
        world.setParticleRadius(0.5f)
        world.createParticle(ParticleDef().apply {
            position.set(-0.4f, 0f)
            velocity.set(1f, 0f)
        })
        world.createParticle(ParticleDef().apply {
            position.set(0.4f, 0f)
            velocity.set(-1f, 0f)
        })

        assertTrue(world.computeParticleCollisionEnergy() > 0f)
    }


    @Test
    fun particleSystemBuildsParticleContacts() {
        val world = World(gravity = Vec2(0f, 0f))
        world.setParticleRadius(0.5f)
        world.createParticle(ParticleDef().apply {
            position.set(0f, 0f)
            velocity.set(1f, 0f)
        })
        world.createParticle(ParticleDef().apply {
            position.set(0.7f, 0f)
            velocity.set(-1f, 0f)
        })

        world.step(1f / 60f, 8, 3)

        assertTrue(world.getParticleContactCount() > 0)
        assertTrue(world.getParticleContactBuffer().first().weight > 0f)
    }

    @Test
    fun destroyParticlesInShapeRemovesCoveredParticles() {
        val world = World()
        world.setParticleRadius(0.25f)
        world.createParticle(ParticleDef().apply { position.set(0f, 0f) })
        world.createParticle(ParticleDef().apply { position.set(2f, 0f) })

        val destroyed = world.destroyParticlesInShape(
            CircleShape().apply {
                m_p.setZero()
                m_radius = 0.6f
            },
            Transform(),
        )

        assertEquals(1, destroyed)
        assertEquals(1, world.getParticleCount())
        assertTrue(abs(world.getParticlePositionBuffer()[0].x - 2f) < 0.001f)
    }

    @Test
    fun particleQueryAabbAndRaycastReachParticles() {
        val world = World(gravity = Vec2(0f, 0f))
        world.setParticleRadius(0.25f)
        world.createParticle(ParticleDef().apply { position.set(0f, 0f) })
        world.createParticle(ParticleDef().apply { position.set(2f, 0f) })

        val queried = mutableListOf<Int>()
        world.queryParticleAABB(ParticleQueryCallback { index ->
            queried += index
            true
        }, AABB().apply {
            lowerBound.set(-0.5f, -0.5f)
            upperBound.set(0.5f, 0.5f)
        })

        var hitIndex = -1
        var hitFraction = 1f
        world.raycastParticle(ParticleRaycastCallback { index, _, _, fraction ->
            hitIndex = index
            hitFraction = fraction
            fraction
        }, Vec2(-1f, 0f), Vec2(1f, 0f))

        assertEquals(listOf(0), queried)
        assertEquals(0, hitIndex)
        assertTrue(hitFraction in 0f..1f)
    }

    @Test
    fun computeDepthForSolidGroupBuildsPositiveInteriorDepth() {
        val world = World(gravity = Vec2(0f, 0f))
        world.setParticleRadius(0.3f)
        val group = world.createParticleGroup(ParticleGroupDef().apply {
            groupFlags = ParticleGroupType.b2_solidParticleGroup
            shape = CircleShape().apply {
                m_p.setZero()
                m_radius = 1.1f
            }
        })

        world.computeDepthForGroup(group)

        val depths = world.getParticleDepthBuffer().sliceArray(group.getBufferIndex() until (group.getBufferIndex() + group.getParticleCount()))
        assertTrue(depths.any { it > 0f })
        assertFalse(depths.all { it == 0f })
    }

    @Test
    fun springGroupPullsStretchedParticlesBackTowardRestLength() {
        val system = ParticleSystem(World(gravity = Vec2(0f, 0f)))
        system.setParticleRadius(0.3f)
        val group = system.createParticleGroup(ParticleGroupDef().apply {
            flags = ParticleType.b2_springParticle
            shape = CircleShape().apply {
                m_p.setZero()
                m_radius = 0.9f
            }
        })
        val indices = (group.getBufferIndex() until (group.getBufferIndex() + group.getParticleCount())).toList()
        var pairA = indices[0]
        var pairB = indices[1]
        var bestDistance = Float.MAX_VALUE
        for (i in indices.indices) {
            for (j in i + 1 until indices.size) {
                val a = indices[i]
                val b = indices[j]
                val distance = system.positionBuffer[a].sub(system.positionBuffer[b]).length()
                if (distance < bestDistance) {
                    bestDistance = distance
                    pairA = a
                    pairB = b
                }
            }
        }
        val restDistance = system.positionBuffer[pairA].sub(system.positionBuffer[pairB]).length()
        system.positionBuffer[pairB].x += 0.45f
        val before = abs(system.positionBuffer[pairA].sub(system.positionBuffer[pairB]).length() - restDistance)

        val step = TimeStep().apply {
            dt = 1f / 60f
            inv_dt = 60f
        }
        repeat(8) {
            system.solve(step)
        }

        val after = abs(system.positionBuffer[pairA].sub(system.positionBuffer[pairB]).length() - restDistance)
        assertTrue(after < before)
    }

    @Test
    fun joinedSpringGroupsStillPullTowardRestLength() {
        val world = World(gravity = Vec2(0f, 0f))
        world.setParticleRadius(0.3f)
        val groupA = world.createParticleGroup(ParticleGroupDef().apply {
            flags = ParticleType.b2_springParticle
            shape = CircleShape().apply {
                m_p.setZero()
                m_radius = 0.7f
            }
            position.set(-0.6f, 0f)
        })
        val groupB = world.createParticleGroup(ParticleGroupDef().apply {
            flags = ParticleType.b2_springParticle
            shape = CircleShape().apply {
                m_p.setZero()
                m_radius = 0.7f
            }
            position.set(0.6f, 0f)
        })
        world.joinParticleGroups(groupA, groupB)

        val indices = (groupA.getBufferIndex() until (groupA.getBufferIndex() + groupA.getParticleCount())).toList()
        var pairA = indices[0]
        var pairB = indices[1]
        var bestDistance = Float.MAX_VALUE
        val positions = world.getParticlePositionBuffer()
        for (i in indices.indices) {
            for (j in i + 1 until indices.size) {
                val a = indices[i]
                val b = indices[j]
                val distance = positions[a].sub(positions[b]).length()
                if (distance < bestDistance) {
                    bestDistance = distance
                    pairA = a
                    pairB = b
                }
            }
        }
        val restDistance = positions[pairA].sub(positions[pairB]).length()
        world.getParticlePositionBuffer()[pairB].x += 0.35f
        val before = abs(world.getParticlePositionBuffer()[pairA].sub(world.getParticlePositionBuffer()[pairB]).length() - restDistance)

        repeat(10) {
            world.step(1f / 60f, 8, 3)
        }

        val after = abs(world.getParticlePositionBuffer()[pairA].sub(world.getParticlePositionBuffer()[pairB]).length() - restDistance)
        assertTrue(after < before)
    }

    @Test
    fun elasticGroupPullsDisplacedParticleTowardOriginalShape() {
        fun groupCenter(system: ParticleSystem, group: ParticleGroup): Vec2 {
            val count = group.getParticleCount()
            val center = Vec2()
            for (offset in 0 until count) {
                center.addLocal(system.positionBuffer[group.getBufferIndex() + offset])
            }
            center.mulLocal(1f / count)
            return center
        }

        val system = ParticleSystem(World(gravity = Vec2(0f, 0f)))
        system.setParticleRadius(0.3f)
        val group = system.createParticleGroup(ParticleGroupDef().apply {
            flags = ParticleType.b2_elasticParticle
            shape = CircleShape().apply {
                m_p.setZero()
                m_radius = 0.9f
            }
        })
        val index = group.getBufferIndex()
        val originalRadius = group.m_localPositions[0].length()
        system.positionBuffer[index].x += 0.6f
        system.positionBuffer[index].y += 0.2f
        val beforeCenter = groupCenter(system, group)
        val beforeRadius = system.positionBuffer[index].sub(beforeCenter).length()
        val before = abs(beforeRadius - originalRadius)

        val step = TimeStep().apply {
            dt = 1f / 60f
            inv_dt = 60f
        }
        repeat(12) {
            system.solve(step)
        }

        val afterCenter = groupCenter(system, group)
        val afterRadius = system.positionBuffer[index].sub(afterCenter).length()
        val after = abs(afterRadius - originalRadius)
        assertTrue(after < before)
    }

    @Test
    fun wallParticleStaysFixedUnderGravity() {
        val world = World(gravity = Vec2(0f, -10f))
        world.createParticle(ParticleDef().apply {
            flags = ParticleType.b2_wallParticle
            position.set(0f, 5f)
            velocity.set(3f, 4f)
        })

        repeat(10) {
            world.step(1f / 60f, 8, 3)
        }

        val position = world.getParticlePositionBuffer()[0]
        val velocity = world.getParticleVelocityBuffer()[0]
        assertTrue(abs(position.x) < 0.0001f)
        assertTrue(abs(position.y - 5f) < 0.0001f)
        assertTrue(abs(velocity.x) < 0.0001f)
        assertTrue(abs(velocity.y) < 0.0001f)
    }

    @Test
    fun viscousParticlesDampRelativeVelocityMoreThanDefaultParticles() {
        fun stepWorld(flags: Int): Float {
            val world = World(gravity = Vec2(0f, 0f))
            world.setParticleRadius(0.5f)
            world.createParticle(ParticleDef().apply {
                this.flags = flags
                position.set(0f, 0f)
                velocity.set(2f, 0f)
            })
            world.createParticle(ParticleDef().apply {
                this.flags = flags
                position.set(0.7f, 0f)
                velocity.set(-2f, 0f)
            })
            world.step(1f / 60f, 8, 3)
            val velocities = world.getParticleVelocityBuffer()
            return abs(velocities[1].x - velocities[0].x)
        }

        val defaultDelta = stepWorld(0)
        val viscousDelta = stepWorld(ParticleType.b2_viscousParticle)

        assertTrue(viscousDelta < defaultDelta)
    }

    @Test
    fun colorMixingParticlesBlendTowardEachOther() {
        val world = World(gravity = Vec2(0f, 0f))
        world.setParticleRadius(0.5f)
        world.createParticle(ParticleDef().apply {
            flags = ParticleType.b2_colorMixingParticle
            position.set(0f, 0f)
            velocity.set(0f, 0f)
            color = ParticleColor((-1).toByte(), 0, 0, (-1).toByte())
        })
        world.createParticle(ParticleDef().apply {
            flags = ParticleType.b2_colorMixingParticle
            position.set(0.7f, 0f)
            velocity.set(0f, 0f)
            color = ParticleColor(0, 0, (-1).toByte(), (-1).toByte())
        })

        val before = world.getParticleColorBuffer().map { it!! }
        val beforeRedGap = abs((before[0].r.toInt() and 0xFF) - (before[1].r.toInt() and 0xFF))
        val beforeBlueGap = abs((before[0].b.toInt() and 0xFF) - (before[1].b.toInt() and 0xFF))

        world.step(1f / 60f, 8, 3)

        val after = world.getParticleColorBuffer().map { it!! }
        val afterRedGap = abs((after[0].r.toInt() and 0xFF) - (after[1].r.toInt() and 0xFF))
        val afterBlueGap = abs((after[0].b.toInt() and 0xFF) - (after[1].b.toInt() and 0xFF))
        assertTrue(afterRedGap < beforeRedGap)
        assertTrue(afterBlueGap < beforeBlueGap)
    }

    @Test
    fun particleDestructionListenerReceivesDestroyedParticle() {
        val world = World()
        val destroyed = mutableListOf<Int>()
        world.setParticleDestructionListener(object : ParticleDestructionListener {
            override fun sayGoodbye(group: ParticleGroup) = Unit
            override fun sayGoodbye(index: Int) {
                destroyed += index
            }
        })
        world.createParticle(ParticleDef().apply { position.set(0f, 0f) })
        world.createParticle(ParticleDef().apply { position.set(2f, 0f) })

        val removed = world.destroyParticlesInShape(
            CircleShape().apply {
                m_p.setZero()
                m_radius = 0.6f
            },
            Transform(),
            true,
        )

        assertEquals(1, removed)
        assertEquals(listOf(0), destroyed)
    }

    @Test
    fun particleSystemBuildsBodyContactsNearStaticFixture() {
        val world = World(gravity = Vec2(0f, 0f))
        world.setParticleRadius(0.5f)
        val ground = world.createBody(xyz.thewind.kbox2d.dynamics.BodyDef().apply {
            type = xyz.thewind.kbox2d.dynamics.BodyType.STATIC
            position = Vec2(0f, 0f)
        })!!
        ground.createFixture(xyz.thewind.kbox2d.dynamics.FixtureDef().apply {
            shape = CircleShape().apply { m_radius = 1f }
            density = 0f
        })
        world.createParticle(ParticleDef().apply {
            position.set(1.2f, 0f)
            velocity.set(-1f, 0f)
        })

        world.step(1f / 60f, 8, 3)

        assertTrue(world.getParticleBodyContactCount() > 0)
        assertTrue(world.getParticleBodyContactBuffer().first().body === ground)
    }

}
