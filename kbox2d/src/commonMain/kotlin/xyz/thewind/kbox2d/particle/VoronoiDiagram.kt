package xyz.thewind.kbox2d.particle

import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.pooling.normal.MutableStack

class VoronoiDiagram(generatorCapacity: Int) {
    class Generator {
        val center: Vec2 = Vec2()
        var tag: Int = 0
    }

    class VoronoiDiagramTask {
        var m_x: Int = 0
        var m_y: Int = 0
        var m_i: Int = 0
        var m_generator: Generator? = null

        constructor()

        constructor(x: Int, y: Int, i: Int, g: Generator) {
            set(x, y, i, g)
        }

        fun set(x: Int, y: Int, i: Int, g: Generator): VoronoiDiagramTask {
            m_x = x
            m_y = y
            m_i = i
            m_generator = g
            return this
        }
    }

    fun interface VoronoiDiagramCallback {
        fun callback(aTag: Int, bTag: Int, cTag: Int)
    }

    private val m_generatorBuffer: Array<Generator> = Array(generatorCapacity) { Generator() }
    private var m_generatorCount: Int = 0
    private var m_countX: Int = 0
    private var m_countY: Int = 0
    private var m_diagram: Array<Generator?>? = null

    private val lower = Vec2()
    private val upper = Vec2()
    private val taskPool = object : MutableStack<VoronoiDiagramTask>(50) {
        override fun newInstance(): VoronoiDiagramTask = VoronoiDiagramTask()

        override fun newArray(size: Int): Array<VoronoiDiagramTask> = Array(size) { VoronoiDiagramTask() }
    }
    private val queue = StackQueue<VoronoiDiagramTask>()

    fun getNodes(callback: VoronoiDiagramCallback) {
        val diagram = m_diagram ?: return
        for (y in 0 until m_countY - 1) {
            for (x in 0 until m_countX - 1) {
                val i = x + y * m_countX
                val a = diagram[i]!!
                val b = diagram[i + 1]!!
                val c = diagram[i + m_countX]!!
                val d = diagram[i + 1 + m_countX]!!
                if (b !== c) {
                    if (a !== b && a !== c) {
                        callback.callback(a.tag, b.tag, c.tag)
                    }
                    if (d !== b && d !== c) {
                        callback.callback(b.tag, d.tag, c.tag)
                    }
                }
            }
        }
    }

    fun addGenerator(center: Vec2, tag: Int) {
        val generator = m_generatorBuffer[m_generatorCount++]
        generator.center.x = center.x
        generator.center.y = center.y
        generator.tag = tag
    }

    fun generate(radius: Float) {
        assert(m_diagram == null)
        val inverseRadius = 1f / radius
        lower.x = Float.MAX_VALUE
        lower.y = Float.MAX_VALUE
        upper.x = -Float.MAX_VALUE
        upper.y = -Float.MAX_VALUE
        for (index in 0 until m_generatorCount) {
            val generator = m_generatorBuffer[index]
            Vec2.minToOut(lower, generator.center, lower)
            Vec2.maxToOut(upper, generator.center, upper)
        }
        m_countX = 1 + (inverseRadius * (upper.x - lower.x)).toInt()
        m_countY = 1 + (inverseRadius * (upper.y - lower.y)).toInt()
        m_diagram = arrayOfNulls(m_countX * m_countY)
        queue.reset(Array(4 * m_countX * m_countX) { VoronoiDiagramTask() })

        for (index in 0 until m_generatorCount) {
            val generator = m_generatorBuffer[index]
            generator.center.x = inverseRadius * (generator.center.x - lower.x)
            generator.center.y = inverseRadius * (generator.center.y - lower.y)
            val x = MathUtils.max(0, MathUtils.min(generator.center.x.toInt(), m_countX - 1))
            val y = MathUtils.max(0, MathUtils.min(generator.center.y.toInt(), m_countY - 1))
            queue.push(taskPool.pop().set(x, y, x + y * m_countX, generator))
        }

        val diagram = m_diagram!!
        while (!queue.empty()) {
            val front = queue.pop()
            val x = front.m_x
            val y = front.m_y
            val i = front.m_i
            val generator = front.m_generator!!
            if (diagram[i] == null) {
                diagram[i] = generator
                if (x > 0) queue.push(taskPool.pop().set(x - 1, y, i - 1, generator))
                if (y > 0) queue.push(taskPool.pop().set(x, y - 1, i - m_countX, generator))
                if (x < m_countX - 1) queue.push(taskPool.pop().set(x + 1, y, i + 1, generator))
                if (y < m_countY - 1) queue.push(taskPool.pop().set(x, y + 1, i + m_countX, generator))
            }
            taskPool.push(front)
        }

        val maxIteration = m_countX + m_countY
        for (iteration in 0 until maxIteration) {
            for (y in 0 until m_countY) {
                for (x in 0 until m_countX - 1) {
                    val i = x + y * m_countX
                    val a = diagram[i]
                    val b = diagram[i + 1]
                    if (a !== b) {
                        queue.push(taskPool.pop().set(x, y, i, b!!))
                        queue.push(taskPool.pop().set(x + 1, y, i + 1, a!!))
                    }
                }
            }
            for (y in 0 until m_countY - 1) {
                for (x in 0 until m_countX) {
                    val i = x + y * m_countX
                    val a = diagram[i]
                    val b = diagram[i + m_countX]
                    if (a !== b) {
                        queue.push(taskPool.pop().set(x, y, i, b!!))
                        queue.push(taskPool.pop().set(x, y + 1, i + m_countX, a!!))
                    }
                }
            }
            var updated = false
            while (!queue.empty()) {
                val front = queue.pop()
                val x = front.m_x
                val y = front.m_y
                val i = front.m_i
                val candidate = front.m_generator!!
                val current = diagram[i]!!
                if (current !== candidate) {
                    val ax = current.center.x - x
                    val ay = current.center.y - y
                    val bx = candidate.center.x - x
                    val by = candidate.center.y - y
                    val a2 = ax * ax + ay * ay
                    val b2 = bx * bx + by * by
                    if (a2 > b2) {
                        diagram[i] = candidate
                        if (x > 0) queue.push(taskPool.pop().set(x - 1, y, i - 1, candidate))
                        if (y > 0) queue.push(taskPool.pop().set(x, y - 1, i - m_countX, candidate))
                        if (x < m_countX - 1) queue.push(taskPool.pop().set(x + 1, y, i + 1, candidate))
                        if (y < m_countY - 1) queue.push(taskPool.pop().set(x, y + 1, i + m_countX, candidate))
                        updated = true
                    }
                }
                taskPool.push(front)
            }
            if (!updated) {
                break
            }
        }
    }
}
