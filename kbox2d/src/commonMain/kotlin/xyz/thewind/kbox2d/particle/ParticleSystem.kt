package xyz.thewind.kbox2d.particle

import xyz.thewind.kbox2d.collision.AABB
import xyz.thewind.kbox2d.callbacks.ParticleQueryCallback
import xyz.thewind.kbox2d.callbacks.ParticleRaycastCallback
import xyz.thewind.kbox2d.collision.shapes.Shape
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.BodyType
import xyz.thewind.kbox2d.dynamics.TimeStep
import xyz.thewind.kbox2d.dynamics.World

class ParticleSystem(private val world: World) {
    private data class PairConnection(var indexA: Int, var indexB: Int, var flags: Int, var strength: Float, var distance: Float)
    private data class TriadConnection(
        var indexA: Int,
        var indexB: Int,
        var indexC: Int,
        var flags: Int,
        var strength: Float,
        val pa: Vec2 = Vec2(),
        val pb: Vec2 = Vec2(),
        val pc: Vec2 = Vec2(),
        var ka: Float = 0f,
        var kb: Float = 0f,
        var kc: Float = 0f,
        var s: Float = 0f,
    )
    internal val flagsBuffer: MutableList<Int> = mutableListOf()
    internal val positionBuffer: MutableList<Vec2> = mutableListOf()
    internal val velocityBuffer: MutableList<Vec2> = mutableListOf()
    internal val colorBuffer: MutableList<ParticleColor?> = mutableListOf()
    internal val groupBuffer: MutableList<ParticleGroup?> = mutableListOf()
    internal val userDataBuffer: MutableList<Any?> = mutableListOf()
    internal val depthBuffer: MutableList<Float> = mutableListOf()
    internal val contactBuffer: MutableList<ParticleContact> = mutableListOf()
    internal val bodyContactBuffer: MutableList<ParticleBodyContact> = mutableListOf()
    private val pairBuffer: MutableList<PairConnection> = mutableListOf()
    private val triadBuffer: MutableList<TriadConnection> = mutableListOf()
    internal var m_timestamp: Int = 0
    private var groupList: ParticleGroup? = null
    private var particleDensity: Float = 1f
    private var particleGravityScale: Float = 1f
    private var particleDamping: Float = 0.2f
    private var particleRadius: Float = 0.5f
    private var particleMaxCount: Int = 0
    private var viscousStrength: Float = 0.25f
    private var powderStrength: Float = 0.5f
    private var surfaceTensionStrengthA: Float = 0.1f
    private var surfaceTensionStrengthB: Float = 0.2f
    private var ejectionStrength: Float = 0.5f
    private var colorMixingStrength: Float = 0.5f
    private var springStrength: Float = 0.25f
    private var elasticStrength: Float = 0.25f

    fun shiftOrigin(newOrigin: Vec2) {
        for (position in positionBuffer) {
            position.subLocal(newOrigin)
        }
        var group = groupList
        while (group != null) {
            group.m_transform.p.subLocal(newOrigin)
            group = group.m_next
        }
        m_timestamp++
    }

    fun solve(step: TimeStep) {
        if (positionBuffer.isEmpty()) return
        val gravity = world.m_gravity
        val dampingFactor = 1f / (1f + step.dt * particleDamping)
        for (index in positionBuffer.indices) {
            if ((flagsBuffer[index] and ParticleType.b2_wallParticle) != 0) {
                velocityBuffer[index].setZero()
                continue
            }
            val group = groupBuffer[index]
            val rigid = group != null && (group.m_groupFlags and ParticleGroupType.b2_rigidParticleGroup) != 0
            if (rigid) {
                continue
            }
            val velocity = velocityBuffer[index]
            velocity.x += step.dt * gravity.x * particleGravityScale
            velocity.y += step.dt * gravity.y * particleGravityScale
            velocity.mulLocal(dampingFactor)
            positionBuffer[index].x += step.dt * velocity.x
            positionBuffer[index].y += step.dt * velocity.y
        }
        updateRigidGroups(step, gravity, dampingFactor)
        updateBodyContacts()
        updateContacts(false)
        updateSolidGroupDepths()
        solveWall()
        solveViscous(step)
        solvePowder(step)
        solveTensile(step)
        solveSpring(step)
        solveElastic(step)
        solveParticleContacts(step)
        solveSolid(step)
        solveColorMixing()
        solveBodyContacts(step)
        solveWall()
        m_timestamp++
    }

    fun createParticle(def: ParticleDef): Int {
        if (particleMaxCount > 0 && positionBuffer.size >= particleMaxCount) {
            return -1
        }
        flagsBuffer += def.flags
        positionBuffer += def.position.clone()
        velocityBuffer += def.velocity.clone()
        colorBuffer += def.color?.let { ParticleColor().also { copy -> copy.set(it) } }
        groupBuffer += null
        userDataBuffer += def.userData
        depthBuffer += 0f
        m_timestamp++
        return positionBuffer.lastIndex
    }

    fun destroyParticle(index: Int, callDestructionListener: Boolean = false) {
        if (index !in positionBuffer.indices) return
        val group = groupBuffer[index]
        val shouldNotify = callDestructionListener || (flagsBuffer[index] and ParticleType.b2_destructionListener) != 0
        if (group != null) {
            val localIndex = index - group.m_firstIndex
            if (localIndex in group.m_localPositions.indices) {
                group.m_localPositions.removeAt(localIndex)
            }
        }
        if (shouldNotify) {
            world.getParticleDestructionListener()?.sayGoodbye(index)
        }
        flagsBuffer.removeAt(index)
        positionBuffer.removeAt(index)
        velocityBuffer.removeAt(index)
        colorBuffer.removeAt(index)
        groupBuffer.removeAt(index)
        userDataBuffer.removeAt(index)
        depthBuffer.removeAt(index)
        adjustTopologyAfterParticleRemoval(index)
        if (group != null) {
            reindexGroup(group)
        }
        m_timestamp++
    }

    fun createParticleGroup(def: ParticleGroupDef): ParticleGroup {
        val group = ParticleGroup()
        group.m_system = this
        group.m_groupFlags = def.groupFlags
        group.m_strength = def.strength
        group.m_destroyAutomatically = def.destroyAutomatically
        group.m_userData = def.userData
        group.m_transform.set(def.position, def.angle)
        group.m_linearVelocity.set(def.linearVelocity)
        group.m_angularVelocity = def.angularVelocity
        group.m_prev = null
        group.m_next = groupList
        groupList?.m_prev = group
        groupList = group

        val createdIndices = mutableListOf<Int>()
        val colorCopy = def.color?.let { ParticleColor().also { copy -> copy.set(it) } }
        fun emitParticle(position: Vec2) {
            val offsetX = position.x - def.position.x
            val offsetY = position.y - def.position.y
            val index = createParticle(ParticleDef().apply {
                flags = def.flags
                this.position.set(position)
                velocity.set(
                    def.linearVelocity.x - def.angularVelocity * offsetY,
                    def.linearVelocity.y + def.angularVelocity * offsetX,
                )
                color = colorCopy?.let { ParticleColor().also { copy -> copy.set(it) } }
                userData = def.userData
            })
            if (index >= 0) {
                createdIndices += index
                groupBuffer[index] = group
            }
        }

        val shape = def.shape
        if (shape != null) {
            val aabb = AABB()
            shape.computeAABB(aabb, group.m_transform, 0)
            val spacing = kotlin.math.max(particleRadius * 1.5f, 0.05f)
            var y = aabb.lowerBound.y
            while (y <= aabb.upperBound.y + Settings.EPSILON) {
                var x = aabb.lowerBound.x
                while (x <= aabb.upperBound.x + Settings.EPSILON) {
                    val candidate = Vec2(x, y)
                    if (shape.testPoint(group.m_transform, candidate)) {
                        emitParticle(candidate)
                    }
                    x += spacing
                }
                y += spacing
            }
        }
        if (createdIndices.isEmpty()) {
            emitParticle(def.position)
        }

        val first = createdIndices.minOrNull() ?: 0
        val last = (createdIndices.maxOrNull() ?: -1) + 1
        group.m_firstIndex = first
        group.m_lastIndex = last
        group.m_localPositions.clear()
        for (index in first until last) {
            group.m_localPositions += Transform.mulTrans(group.m_transform, positionBuffer[index])
        }
        rebuildTopologyForGroup(group)
        m_timestamp++
        return group
    }



    fun joinParticleGroups(groupA: ParticleGroup, groupB: ParticleGroup) {
        require(groupA !== groupB)
        require(groupA.m_system === this && groupB.m_system === this)

        var primary = groupA
        var secondary = groupB
        if (primary.m_firstIndex > secondary.m_firstIndex) {
            primary = groupB
            secondary = groupA
        }

        for (index in secondary.m_firstIndex until secondary.m_lastIndex) {
            groupBuffer[index] = primary
        }
        primary.m_groupFlags = primary.m_groupFlags or secondary.m_groupFlags
        primary.m_strength = kotlin.math.min(primary.m_strength, secondary.m_strength)
        primary.m_firstIndex = kotlin.math.min(primary.m_firstIndex, secondary.m_firstIndex)
        primary.m_lastIndex = kotlin.math.max(primary.m_lastIndex, secondary.m_lastIndex)
        primary.m_localPositions.clear()
        for (index in primary.m_firstIndex until primary.m_lastIndex) {
            if (groupBuffer[index] === primary) {
                primary.m_localPositions += Transform.mulTrans(primary.m_transform, positionBuffer[index])
            }
        }
        destroyGroup(secondary)
        secondary.m_firstIndex = secondary.m_lastIndex
        secondary.m_localPositions.clear()
        primary.m_timestamp = -1
        rebuildTopologyForGroup(primary)
        m_timestamp++
    }

    fun getParticleGroupCount(): Int {
        var count = 0
        var group = groupList
        while (group != null) {
            count++
            group = group.m_next
        }
        return count
    }

    fun computeParticleCollisionEnergy(): Float {
        updateContacts(false)
        var sumV2 = 0f
        for (contact in contactBuffer) {
            val va = velocityBuffer[contact.indexA]
            val vb = velocityBuffer[contact.indexB]
            val vx = vb.x - va.x
            val vy = vb.y - va.y
            val vn = vx * contact.normal.x + vy * contact.normal.y
            if (vn < 0f) {
                sumV2 += vn * vn
            }
        }
        return 0.5f * getParticleMass() * sumV2
    }

    fun destroyParticlesInGroup(group: ParticleGroup, callDestructionListener: Boolean = false) {
        val first = group.m_firstIndex
        val count = group.getParticleCount()
        repeat(count) {
            destroyParticle(first, callDestructionListener)
        }
        destroyGroup(group)
    }

    fun destroyParticlesInShape(shape: Shape, xf: Transform, callDestructionListener: Boolean = false): Int {
        var destroyed = 0
        var index = positionBuffer.lastIndex
        while (index >= 0) {
            if (shape.testPoint(xf, positionBuffer[index])) {
                destroyParticle(index, callDestructionListener)
                destroyed++
            }
            index--
        }
        return destroyed
    }

    fun computeDepthForGroup(group: ParticleGroup) {
        require(group.m_system === this)
        updateContacts(true)
        computeDepthForGroupInternal(group)
        group.m_timestamp = -1
        m_timestamp++
    }

    fun queryAABB(callback: ParticleQueryCallback, aabb: AABB) {
        for (index in positionBuffer.indices) {
            val position = positionBuffer[index]
            if (position.x > aabb.lowerBound.x &&
                position.x < aabb.upperBound.x &&
                position.y > aabb.lowerBound.y &&
                position.y < aabb.upperBound.y
            ) {
                if (!callback.reportParticle(index)) {
                    return
                }
            }
        }
    }

    fun raycast(callback: ParticleRaycastCallback, point1: Vec2, point2: Vec2) {
        var fraction = 1f
        val vx = point2.x - point1.x
        val vy = point2.y - point1.y
        var v2 = vx * vx + vy * vy
        if (v2 < Settings.EPSILON) {
            v2 = Float.MAX_VALUE
        }
        val diameter = particleRadius * 2f
        val diameterSq = diameter * diameter
        for (index in positionBuffer.indices) {
            val position = positionBuffer[index]
            val px = point1.x - position.x
            val py = point1.y - position.y
            val pv = px * vx + py * vy
            val p2 = px * px + py * py
            val determinant = pv * pv - v2 * (p2 - diameterSq)
            if (determinant < 0f) continue
            val sqrtDeterminant = MathUtils.sqrt(determinant)
            var t = (-pv - sqrtDeterminant) / v2
            if (t > fraction) {
                continue
            }
            if (t < 0f) {
                t = (-pv + sqrtDeterminant) / v2
                if (t < 0f || t > fraction) {
                    continue
                }
            }
            val normal = Vec2(px + t * vx, py + t * vy)
            if (normal.lengthSquared() > Settings.EPSILON * Settings.EPSILON) {
                normal.normalize()
            } else {
                normal.set(0f, 1f)
            }
            val point = Vec2(point1.x + t * vx, point1.y + t * vy)
            fraction = kotlin.math.min(fraction, callback.reportParticle(index, point, normal, t))
            if (fraction <= 0f) {
                return
            }
        }
    }

    private fun destroyGroup(group: ParticleGroup) {
        removeTopologyForGroup(group)
        world.getParticleDestructionListener()?.sayGoodbye(group)
        group.m_prev?.m_next = group.m_next
        group.m_next?.m_prev = group.m_prev
        if (group === groupList) {
            groupList = group.m_next
        }
        group.m_prev = null
        group.m_next = null
        group.m_system = null
    }

    private fun reindexGroup(group: ParticleGroup) {
        var first = Int.MAX_VALUE
        var last = -1
        for (index in groupBuffer.indices) {
            if (groupBuffer[index] === group) {
                first = kotlin.math.min(first, index)
                last = kotlin.math.max(last, index + 1)
            }
        }
        if (last < 0) {
            destroyGroup(group)
            return
        }
        group.m_firstIndex = first
        group.m_lastIndex = last
    }

    private fun getParticleDiameter(): Float = particleRadius * 2f

    private fun getParticleStride(): Float = Settings.particleStride * getParticleDiameter()

    private fun removeTopologyForGroup(group: ParticleGroup) {
        pairBuffer.removeAll { pair ->
            val aGroup = groupBuffer.getOrNull(pair.indexA)
            val bGroup = groupBuffer.getOrNull(pair.indexB)
            aGroup === group || bGroup === group
        }
        triadBuffer.removeAll { triad ->
            val aGroup = groupBuffer.getOrNull(triad.indexA)
            val bGroup = groupBuffer.getOrNull(triad.indexB)
            val cGroup = groupBuffer.getOrNull(triad.indexC)
            aGroup === group || bGroup === group || cGroup === group
        }
    }

    private fun adjustTopologyAfterParticleRemoval(index: Int) {
        pairBuffer.removeAll { it.indexA == index || it.indexB == index }
        triadBuffer.removeAll { it.indexA == index || it.indexB == index || it.indexC == index }
        for (pair in pairBuffer) {
            if (pair.indexA > index) pair.indexA--
            if (pair.indexB > index) pair.indexB--
        }
        for (triad in triadBuffer) {
            if (triad.indexA > index) triad.indexA--
            if (triad.indexB > index) triad.indexB--
            if (triad.indexC > index) triad.indexC--
        }
    }

    private fun rebuildTopologyForGroup(group: ParticleGroup) {
        removeTopologyForGroup(group)
        if (group.getParticleCount() <= 1) return
        updateContacts(true)
        val hasSpring = (group.m_firstIndex until group.m_lastIndex).any { index ->
            index in flagsBuffer.indices && groupBuffer[index] === group && (flagsBuffer[index] and ParticleType.b2_springParticle) != 0
        }
        val hasElastic = (group.m_firstIndex until group.m_lastIndex).any { index ->
            index in flagsBuffer.indices && groupBuffer[index] === group && (flagsBuffer[index] and ParticleType.b2_elasticParticle) != 0
        }
        if (hasSpring) {
            buildSpringPairsForGroup(group)
        }
        if (hasElastic) {
            buildElasticTriadsForGroup(group)
        }
    }

    private fun buildSpringPairsForGroup(group: ParticleGroup) {
        for (contact in contactBuffer) {
            var a = contact.indexA
            var b = contact.indexB
            if (a > b) {
                val temp = a
                a = b
                b = temp
            }
            if (a !in flagsBuffer.indices || b !in flagsBuffer.indices) continue
            if (groupBuffer[a] !== group || groupBuffer[b] !== group) continue
            if ((flagsBuffer[a] and ParticleType.b2_springParticle) == 0 || (flagsBuffer[b] and ParticleType.b2_springParticle) == 0) continue
            pairBuffer += PairConnection(
                indexA = a,
                indexB = b,
                flags = flagsBuffer[a] or flagsBuffer[b],
                strength = group.m_strength,
                distance = positionBuffer[a].sub(positionBuffer[b]).length(),
            )
        }
    }

    private fun appendElasticTriad(a: Int, b: Int, c: Int, strength: Float) {
        if (a !in positionBuffer.indices || b !in positionBuffer.indices || c !in positionBuffer.indices) return
        val pa = positionBuffer[a]
        val pb = positionBuffer[b]
        val pc = positionBuffer[c]
        val dabx = pa.x - pb.x
        val daby = pa.y - pb.y
        val dbcx = pb.x - pc.x
        val dbcy = pb.y - pc.y
        val dcax = pc.x - pa.x
        val dcay = pc.y - pa.y
        val maxDistanceSquared = Settings.maxTriadDistanceSquared * getParticleDiameter() * getParticleDiameter()
        if (dabx * dabx + daby * daby >= maxDistanceSquared ||
            dbcx * dbcx + dbcy * dbcy >= maxDistanceSquared ||
            dcax * dcax + dcay * dcay >= maxDistanceSquared
        ) {
            return
        }
        val midPointX = (pa.x + pb.x + pc.x) / 3f
        val midPointY = (pa.y + pb.y + pc.y) / 3f
        triadBuffer += TriadConnection(
            indexA = a,
            indexB = b,
            indexC = c,
            flags = flagsBuffer[a] or flagsBuffer[b] or flagsBuffer[c],
            strength = strength,
            pa = Vec2(pa.x - midPointX, pa.y - midPointY),
            pb = Vec2(pb.x - midPointX, pb.y - midPointY),
            pc = Vec2(pc.x - midPointX, pc.y - midPointY),
            ka = -(dcax * dabx + dcay * daby),
            kb = -(dabx * dbcx + daby * dbcy),
            kc = -(dbcx * dcax + dbcy * dcay),
            s = Vec2.cross(pa, pb) + Vec2.cross(pb, pc) + Vec2.cross(pc, pa),
        )
    }

    private fun buildElasticTriadsForGroup(group: ParticleGroup) {
        val diagram = VoronoiDiagram(group.getParticleCount())
        for (index in group.m_firstIndex until group.m_lastIndex) {
            if (index in positionBuffer.indices && groupBuffer[index] === group) {
                diagram.addGenerator(positionBuffer[index], index)
            }
        }
        diagram.generate(getParticleStride() / 2f)
        val seen = mutableSetOf<String>()
        diagram.getNodes { a, b, c ->
            if (groupBuffer.getOrNull(a) !== group || groupBuffer.getOrNull(b) !== group || groupBuffer.getOrNull(c) !== group) return@getNodes
            if ((flagsBuffer[a] and flagsBuffer[b] and flagsBuffer[c] and ParticleType.b2_elasticParticle) == 0) return@getNodes
            val sorted = intArrayOf(a, b, c).apply { sort() }
            val key = "${sorted[0]}:${sorted[1]}:${sorted[2]}"
            if (seen.add(key)) {
                appendElasticTriad(sorted[0], sorted[1], sorted[2], group.m_strength)
            }
        }
    }

    private fun ensureDepthCapacity() {
        while (depthBuffer.size < positionBuffer.size) {
            depthBuffer += 0f
        }
        while (depthBuffer.size > positionBuffer.size) {
            depthBuffer.removeAt(depthBuffer.lastIndex)
        }
    }

    private fun computeDepthForGroupInternal(group: ParticleGroup) {
        ensureDepthCapacity()
        for (index in group.m_firstIndex until group.m_lastIndex) {
            depthBuffer[index] = 0f
        }
        for (contact in contactBuffer) {
            val a = contact.indexA
            val b = contact.indexB
            if (a in group.m_firstIndex until group.m_lastIndex && b in group.m_firstIndex until group.m_lastIndex) {
                depthBuffer[a] += contact.weight
                depthBuffer[b] += contact.weight
            }
        }
        for (index in group.m_firstIndex until group.m_lastIndex) {
            depthBuffer[index] = if (depthBuffer[index] < 0.8f) 0f else Float.MAX_VALUE
        }
        repeat(group.getParticleCount()) {
            var updated = false
            for (contact in contactBuffer) {
                val a = contact.indexA
                val b = contact.indexB
                if (a in group.m_firstIndex until group.m_lastIndex && b in group.m_firstIndex until group.m_lastIndex) {
                    val offset = 1f - contact.weight
                    val nextA = depthBuffer[b] + offset
                    val nextB = depthBuffer[a] + offset
                    if (depthBuffer[a] > nextA) {
                        depthBuffer[a] = nextA
                        updated = true
                    }
                    if (depthBuffer[b] > nextB) {
                        depthBuffer[b] = nextB
                        updated = true
                    }
                }
            }
            if (!updated) return@repeat
        }
        for (index in group.m_firstIndex until group.m_lastIndex) {
            val depth = depthBuffer[index]
            depthBuffer[index] = if (depth < Float.MAX_VALUE) depth * particleRadius * 2f else 0f
        }
    }

    private fun updateSolidGroupDepths() {
        var group = groupList
        while (group != null) {
            if ((group.m_groupFlags and ParticleGroupType.b2_solidParticleGroup) != 0 && group.getParticleCount() > 0) {
                computeDepthForGroupInternal(group)
            }
            group = group.m_next
        }
    }

    private fun updateRigidGroups(step: TimeStep, gravity: Vec2, dampingFactor: Float) {
        var group = groupList
        while (group != null) {
            if ((group.m_groupFlags and ParticleGroupType.b2_rigidParticleGroup) != 0 && group.getParticleCount() > 0) {
                group.m_timestamp = -1
                group.updateStatistics()
                group.m_linearVelocity.x += step.dt * gravity.x * particleGravityScale
                group.m_linearVelocity.y += step.dt * gravity.y * particleGravityScale
                group.m_linearVelocity.mulLocal(dampingFactor)
                group.m_angularVelocity *= dampingFactor
                val angle = group.m_transform.q.getAngle() + step.dt * group.m_angularVelocity
                group.m_transform.q.set(angle)
                group.m_transform.p.x += step.dt * group.m_linearVelocity.x
                group.m_transform.p.y += step.dt * group.m_linearVelocity.y
                val center = group.m_transform.p
                val count = kotlin.math.min(group.m_localPositions.size, group.getParticleCount())
                for (offset in 0 until count) {
                    val index = group.m_firstIndex + offset
                    val local = group.m_localPositions[offset]
                    val worldPos = Transform.mul(group.m_transform, local)
                    positionBuffer[index].set(worldPos)
                    val relX = worldPos.x - center.x
                    val relY = worldPos.y - center.y
                    velocityBuffer[index].set(
                        group.m_linearVelocity.x - group.m_angularVelocity * relY,
                        group.m_linearVelocity.y + group.m_angularVelocity * relX,
                    )
                }
                group.m_timestamp = -1
            }
            group = group.m_next
        }
    }

    private fun updateContacts(exceptZombie: Boolean) {
        contactBuffer.clear()
        val diameter = particleRadius * 2f
        val diameterSq = diameter * diameter
        for (a in 0 until positionBuffer.size) {
            if (exceptZombie && (flagsBuffer[a] and ParticleType.b2_zombieParticle) != 0) continue
            for (b in a + 1 until positionBuffer.size) {
                if (exceptZombie && (flagsBuffer[b] and ParticleType.b2_zombieParticle) != 0) continue
                val dx = positionBuffer[b].x - positionBuffer[a].x
                val dy = positionBuffer[b].y - positionBuffer[a].y
                val distSq = dx * dx + dy * dy
                if (distSq >= diameterSq || distSq < Settings.EPSILON * Settings.EPSILON) {
                    continue
                }
                val invDist = 1f / MathUtils.sqrt(distSq)
                val distance = 1f / invDist
                val weight = 1f - distance / diameter
                val contact = ParticleContact()
                contact.indexA = a
                contact.indexB = b
                contact.flags = flagsBuffer[a] or flagsBuffer[b]
                contact.weight = MathUtils.clamp(weight, 0f, 1f)
                contact.normal.set(dx * invDist, dy * invDist)
                contactBuffer += contact
            }
        }
    }

    private fun solveWall() {
        for (index in flagsBuffer.indices) {
            if ((flagsBuffer[index] and ParticleType.b2_wallParticle) != 0) {
                velocityBuffer[index].setZero()
            }
        }
    }

    private fun getCriticalVelocity(step: TimeStep): Float = if (step.inv_dt > 0f) particleRadius * 2f * step.inv_dt else 0f

    private fun getParticleInvMass(): Float {
        val density = kotlin.math.max(particleDensity, Settings.EPSILON)
        val diameter = kotlin.math.max(particleRadius * 2f, Settings.EPSILON)
        return 1.777777f / (density * diameter * diameter)
    }

    private fun solveViscous(step: TimeStep) {
        if (bodyContactBuffer.isEmpty() && contactBuffer.isEmpty()) return
        for (contact in bodyContactBuffer) {
            val index = contact.index
            if ((flagsBuffer[index] and ParticleType.b2_viscousParticle) == 0) continue
            val body = contact.body ?: continue
            val particleVelocity = velocityBuffer[index]
            val particlePosition = positionBuffer[index]
            val bodyVelocity = body.getLinearVelocityFromWorldPoint(particlePosition)
            val vx = bodyVelocity.x - particleVelocity.x
            val vy = bodyVelocity.y - particleVelocity.y
            val scale = viscousStrength * contact.mass * contact.weight
            val impulse = Vec2(scale * vx, scale * vy)
            val invMass = getParticleInvMass()
            particleVelocity.x += invMass * impulse.x
            particleVelocity.y += invMass * impulse.y
            impulse.negateLocal()
            body.applyLinearImpulse(impulse, particlePosition)
        }
        for (contact in contactBuffer) {
            if ((contact.flags and ParticleType.b2_viscousParticle) == 0) continue
            val a = contact.indexA
            val b = contact.indexB
            val va = velocityBuffer[a]
            val vb = velocityBuffer[b]
            val fx = viscousStrength * contact.weight * (vb.x - va.x)
            val fy = viscousStrength * contact.weight * (vb.y - va.y)
            va.x += fx
            va.y += fy
            vb.x -= fx
            vb.y -= fy
        }
    }

    private fun solvePowder(step: TimeStep) {
        val powderVelocity = powderStrength * getCriticalVelocity(step)
        val minWeight = 1f - Settings.particleStride
        for (contact in bodyContactBuffer) {
            val index = contact.index
            if ((flagsBuffer[index] and ParticleType.b2_powderParticle) == 0 || contact.weight <= minWeight) continue
            val body = contact.body ?: continue
            val inter = powderVelocity * (contact.weight - minWeight)
            val force = Vec2(inter * contact.normal.x, inter * contact.normal.y)
            val velocity = velocityBuffer[index]
            val invMass = getParticleInvMass()
            velocity.x -= invMass * force.x
            velocity.y -= invMass * force.y
            body.applyLinearImpulse(force, positionBuffer[index])
        }
        for (contact in contactBuffer) {
            if ((contact.flags and ParticleType.b2_powderParticle) == 0 || contact.weight <= minWeight) continue
            val a = contact.indexA
            val b = contact.indexB
            val inter = powderVelocity * (contact.weight - minWeight)
            val fx = inter * contact.normal.x
            val fy = inter * contact.normal.y
            velocityBuffer[a].x -= fx
            velocityBuffer[a].y -= fy
            velocityBuffer[b].x += fx
            velocityBuffer[b].y += fy
        }
    }

    private fun solveTensile(step: TimeStep) {
        if (contactBuffer.isEmpty()) return
        val accumulation = kotlin.FloatArray(positionBuffer.size)
        val accumulation2 = MutableList(positionBuffer.size) { Vec2() }
        for (contact in contactBuffer) {
            if ((contact.flags and ParticleType.b2_tensileParticle) == 0) continue
            val a = contact.indexA
            val b = contact.indexB
            val w = contact.weight
            accumulation[a] += w
            accumulation[b] += w
            val inter = (1f - w) * w
            accumulation2[a].x -= inter * contact.normal.x
            accumulation2[a].y -= inter * contact.normal.y
            accumulation2[b].x += inter * contact.normal.x
            accumulation2[b].y += inter * contact.normal.y
        }
        val strengthA = surfaceTensionStrengthA * getCriticalVelocity(step)
        val strengthB = surfaceTensionStrengthB * getCriticalVelocity(step)
        for (contact in contactBuffer) {
            if ((contact.flags and ParticleType.b2_tensileParticle) == 0) continue
            val a = contact.indexA
            val b = contact.indexB
            val h = accumulation[a] + accumulation[b]
            val sx = accumulation2[b].x - accumulation2[a].x
            val sy = accumulation2[b].y - accumulation2[a].y
            val fn = (strengthA * (h - 2f) + strengthB * (sx * contact.normal.x + sy * contact.normal.y)) * contact.weight
            val fx = fn * contact.normal.x
            val fy = fn * contact.normal.y
            velocityBuffer[a].x -= fx
            velocityBuffer[a].y -= fy
            velocityBuffer[b].x += fx
            velocityBuffer[b].y += fy
        }
    }


    private fun solveSpring(step: TimeStep) {
        if (pairBuffer.isEmpty()) return
        val strengthScale = if (step.inv_dt > 0f) step.inv_dt * springStrength else springStrength
        for (pair in pairBuffer) {
            if (pair.indexA !in positionBuffer.indices || pair.indexB !in positionBuffer.indices) continue
            if ((pair.flags and ParticleType.b2_springParticle) == 0) continue
            val pa = positionBuffer[pair.indexA]
            val pb = positionBuffer[pair.indexB]
            val dx = pb.x - pa.x
            val dy = pb.y - pa.y
            var distance = MathUtils.sqrt(dx * dx + dy * dy)
            if (distance < Settings.EPSILON) continue
            val strength = strengthScale * pair.strength
            val fx = strength * (pair.distance - distance) / distance * dx
            val fy = strength * (pair.distance - distance) / distance * dy
            velocityBuffer[pair.indexA].x -= fx
            velocityBuffer[pair.indexA].y -= fy
            velocityBuffer[pair.indexB].x += fx
            velocityBuffer[pair.indexB].y += fy
        }
    }

    private fun solveElastic(step: TimeStep) {
        if (triadBuffer.isEmpty()) return
        val elasticVelocity = if (step.inv_dt > 0f) step.inv_dt * elasticStrength else elasticStrength
        for (triad in triadBuffer) {
            if ((triad.flags and ParticleType.b2_elasticParticle) == 0) continue
            val a = triad.indexA
            val b = triad.indexB
            val c = triad.indexC
            if (a !in positionBuffer.indices || b !in positionBuffer.indices || c !in positionBuffer.indices) continue
            val oa = triad.pa
            val ob = triad.pb
            val oc = triad.pc
            val pa = positionBuffer[a]
            val pb = positionBuffer[b]
            val pc = positionBuffer[c]
            val px = (pa.x + pb.x + pc.x) / 3f
            val py = (pa.y + pb.y + pc.y) / 3f
            var rs = Vec2.cross(oa, pa) + Vec2.cross(ob, pb) + Vec2.cross(oc, pc)
            var rc = Vec2.dot(oa, pa) + Vec2.dot(ob, pb) + Vec2.dot(oc, pc)
            val r2 = rs * rs + rc * rc
            val invR = if (r2 == 0f) Float.MAX_VALUE else MathUtils.sqrt(1f / r2)
            rs *= invR
            rc *= invR
            val strength = elasticVelocity * triad.strength
            val roax = rc * oa.x - rs * oa.y
            val roay = rs * oa.x + rc * oa.y
            val robx = rc * ob.x - rs * ob.y
            val roby = rs * ob.x + rc * ob.y
            val rocx = rc * oc.x - rs * oc.y
            val rocy = rs * oc.x + rc * oc.y
            val va = velocityBuffer[a]
            val vb = velocityBuffer[b]
            val vc = velocityBuffer[c]
            va.x += strength * (roax - (pa.x - px))
            va.y += strength * (roay - (pa.y - py))
            vb.x += strength * (robx - (pb.x - px))
            vb.y += strength * (roby - (pb.y - py))
            vc.x += strength * (rocx - (pc.x - px))
            vc.y += strength * (rocy - (pc.y - py))
        }
    }

    private fun solveSolid(step: TimeStep) {
        val interStrength = if (step.inv_dt > 0f) step.inv_dt * ejectionStrength else 0f
        for (contact in contactBuffer) {
            val a = contact.indexA
            val b = contact.indexB
            if (groupBuffer[a] === groupBuffer[b]) continue
            val h = depthBuffer.getOrElse(a) { 0f } + depthBuffer.getOrElse(b) { 0f }
            if (h <= 0f) continue
            val inter = interStrength * h * contact.weight
            val fx = inter * contact.normal.x
            val fy = inter * contact.normal.y
            velocityBuffer[a].x -= fx
            velocityBuffer[a].y -= fy
            velocityBuffer[b].x += fx
            velocityBuffer[b].y += fy
        }
    }

    private fun mixColorChannel(a: Byte, b: Byte): Pair<Byte, Byte> {
        val delta = ((256f * colorMixingStrength).toInt() * ((b.toInt() and 0xFF) - (a.toInt() and 0xFF))) shr 8
        return ((a.toInt() + delta).toByte()) to ((b.toInt() - delta).toByte())
    }

    private fun solveColorMixing() {
        if (contactBuffer.isEmpty()) return
        for (contact in contactBuffer) {
            val a = contact.indexA
            val b = contact.indexB
            if ((flagsBuffer[a] and flagsBuffer[b] and ParticleType.b2_colorMixingParticle) == 0) continue
            val colorA = colorBuffer[a] ?: ParticleColor().also { colorBuffer[a] = it }
            val colorB = colorBuffer[b] ?: ParticleColor().also { colorBuffer[b] = it }
            val (rA, rB) = mixColorChannel(colorA.r, colorB.r)
            val (gA, gB) = mixColorChannel(colorA.g, colorB.g)
            val (bA, bB) = mixColorChannel(colorA.b, colorB.b)
            val (aA, aB) = mixColorChannel(colorA.a, colorB.a)
            colorA.r = rA
            colorB.r = rB
            colorA.g = gA
            colorB.g = gB
            colorA.b = bA
            colorB.b = bB
            colorA.a = aA
            colorB.a = aB
        }
    }

    private fun solveParticleContacts(step: TimeStep) {
        if (contactBuffer.isEmpty()) return
        val diameter = particleRadius * 2f
        for (contact in contactBuffer) {
            val a = contact.indexA
            val b = contact.indexB
            val pa = positionBuffer[a]
            val pb = positionBuffer[b]
            val va = velocityBuffer[a]
            val vb = velocityBuffer[b]
            val dx = pb.x - pa.x
            val dy = pb.y - pa.y
            val dist = MathUtils.sqrt(dx * dx + dy * dy)
            if (dist < Settings.EPSILON || dist >= diameter) continue
            val penetration = diameter - dist
            val correction = 0.5f * penetration
            pa.x -= contact.normal.x * correction
            pa.y -= contact.normal.y * correction
            pb.x += contact.normal.x * correction
            pb.y += contact.normal.y * correction
            val relVx = vb.x - va.x
            val relVy = vb.y - va.y
            val vn = relVx * contact.normal.x + relVy * contact.normal.y
            if (vn < 0f) {
                val impulse = -0.5f * vn * contact.weight
                va.x -= contact.normal.x * impulse
                va.y -= contact.normal.y * impulse
                vb.x += contact.normal.x * impulse
                vb.y += contact.normal.y * impulse
            }
        }
    }

    private fun updateBodyContacts() {
        bodyContactBuffer.clear()
        val bestContacts = linkedMapOf<Pair<Int, Any>, ParticleBodyContact>()
        var body = world.getBodyList()
        while (body != null) {
            if (!body.isActive()) {
                body = body.getNext()
                continue
            }
            var fixture = body.getFixtureList()
            while (fixture != null) {
                val childCount = fixture.getShape().getChildCount()
                for (particleIndex in positionBuffer.indices) {
                    val position = positionBuffer[particleIndex]
                    val normal = Vec2()
                    var bestDistance = Float.MAX_VALUE
                    for (childIndex in 0 until childCount) {
                        val distance = fixture.computeDistance(position, childIndex, normal)
                        if (distance < bestDistance) {
                            bestDistance = distance
                        }
                    }
                    if (bestDistance <= particleRadius) {
                        val contact = ParticleBodyContact()
                        contact.index = particleIndex
                        contact.body = body
                        contact.mass = body.getMass()
                        contact.weight = MathUtils.clamp(1f - bestDistance / kotlin.math.max(particleRadius, Settings.EPSILON), 0f, 1f)
                        if (normal.lengthSquared() > Settings.EPSILON * Settings.EPSILON) {
                            contact.normal.set(normal)
                        } else {
                            contact.normal.set(0f, 1f)
                        }
                        val key = particleIndex to body
                        val existing = bestContacts[key]
                        if (existing == null || contact.weight > existing.weight) {
                            bestContacts[key] = contact
                        }
                    }
                }
                fixture = fixture.getNext()
            }
            body = body.getNext()
        }
        bodyContactBuffer += bestContacts.values
    }

    private fun solveBodyContacts(step: TimeStep) {
        for (contact in bodyContactBuffer) {
            val index = contact.index
            val body = contact.body ?: continue
            if (body.getType() == BodyType.STATIC) {
                val velocity = velocityBuffer[index]
                val vn = velocity.x * contact.normal.x + velocity.y * contact.normal.y
                if (vn < 0f) {
                    velocity.x -= (1f + contact.weight) * vn * contact.normal.x
                    velocity.y -= (1f + contact.weight) * vn * contact.normal.y
                }
                positionBuffer[index].x += contact.normal.x * particleRadius * 0.25f * contact.weight
                positionBuffer[index].y += contact.normal.y * particleRadius * 0.25f * contact.weight
            }
        }
    }

    fun getParticleCount(): Int = positionBuffer.size
    fun getParticleMass(): Float = particleDensity * MathUtils.PI * particleRadius * particleRadius
    fun getParticlePositionBuffer(): Array<Vec2> = positionBuffer.toTypedArray()
    fun getParticleVelocityBuffer(): Array<Vec2> = velocityBuffer.toTypedArray()
    fun getParticleColorBuffer(): Array<ParticleColor?> = colorBuffer.toTypedArray()
    fun getParticleGroupBuffer(): Array<ParticleGroup?> = groupBuffer.toTypedArray()
    fun getParticleUserDataBuffer(): Array<Any?> = userDataBuffer.toTypedArray()
    fun getParticleDepthBuffer(): FloatArray = depthBuffer.toFloatArray()
    fun getParticleContactBuffer(): Array<ParticleContact> = contactBuffer.toTypedArray()
    fun getParticleBodyContactBuffer(): Array<ParticleBodyContact> = bodyContactBuffer.toTypedArray()
    fun getParticleContactCount(): Int = contactBuffer.size
    fun getParticleBodyContactCount(): Int = bodyContactBuffer.size
    fun getParticleFlagsBuffer(): IntArray = flagsBuffer.toIntArray()
    fun getParticleGroupList(): ParticleGroup? = groupList
    fun setParticleMaxCount(count: Int) { particleMaxCount = count }
    fun getParticleMaxCount(): Int = particleMaxCount
    fun setParticleDensity(density: Float) { particleDensity = density }
    fun getParticleDensity(): Float = particleDensity
    fun setParticleGravityScale(scale: Float) { particleGravityScale = scale }
    fun getParticleGravityScale(): Float = particleGravityScale
    fun setParticleDamping(damping: Float) { particleDamping = damping }
    fun getParticleDamping(): Float = particleDamping
    fun setParticleRadius(radius: Float) { particleRadius = radius }
    fun getParticleRadius(): Float = particleRadius
}
