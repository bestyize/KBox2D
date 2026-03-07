package xyz.thewind.kbox2d.collision

import xyz.thewind.kbox2d.collision.Distance.SimplexCache
import xyz.thewind.kbox2d.collision.Manifold.ManifoldType
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.collision.shapes.EdgeShape
import xyz.thewind.kbox2d.collision.shapes.PolygonShape
import xyz.thewind.kbox2d.collision.shapes.Shape
import xyz.thewind.kbox2d.common.Color3f
import xyz.thewind.kbox2d.common.MathUtils
import xyz.thewind.kbox2d.common.Rot
import xyz.thewind.kbox2d.common.Settings
import xyz.thewind.kbox2d.common.Transform
import xyz.thewind.kbox2d.common.Vec2

class Collision(private val distance: Distance = Distance()) {
    private val input = DistanceInput()
    private val cache = SimplexCache()
    private val output = DistanceOutput()

    private val temp = Vec2()
    private val xf = Transform()
    private val n = Vec2()
    private val v1 = Vec2()

    private val results1 = EdgeResults()
    private val results2 = EdgeResults()
    private val incidentEdge = Array(2) { ClipVertex() }
    private val localTangent = Vec2()
    private val localNormal = Vec2()
    private val planePoint = Vec2()
    private val tangent = Vec2()
    private val v11 = Vec2()
    private val v12 = Vec2()
    private val clipPoints1 = Array(2) { ClipVertex() }
    private val clipPoints2 = Array(2) { ClipVertex() }

    private val q = Vec2()
    private val e = Vec2()
    private val cf = ContactID()
    private val e1 = Vec2()
    private val p = Vec2()

    private val collider = EPCollider()

    fun testOverlap(shapeA: Shape, indexA: Int, shapeB: Shape, indexB: Int, xfA: Transform, xfB: Transform): Boolean {
        input.proxyA.set(shapeA, indexA)
        input.proxyB.set(shapeB, indexB)
        input.transformA.set(xfA)
        input.transformB.set(xfB)
        input.useRadii = true
        cache.count = 0
        distance.distance(output, cache, input)
        return output.distance < 10.0f * Settings.EPSILON
    }

    fun collideCircles(manifold: Manifold, circle1: CircleShape, xfA: Transform, circle2: CircleShape, xfB: Transform) {
        manifold.pointCount = 0
        val circle1p = circle1.m_p
        val circle2p = circle2.m_p
        val pAx = (xfA.q.c * circle1p.x - xfA.q.s * circle1p.y) + xfA.p.x
        val pAy = (xfA.q.s * circle1p.x + xfA.q.c * circle1p.y) + xfA.p.y
        val pBx = (xfB.q.c * circle2p.x - xfB.q.s * circle2p.y) + xfB.p.x
        val pBy = (xfB.q.s * circle2p.x + xfB.q.c * circle2p.y) + xfB.p.y
        val dx = pBx - pAx
        val dy = pBy - pAy
        val distSqr = dx * dx + dy * dy
        val radius = circle1.m_radius + circle2.m_radius
        if (distSqr > radius * radius) {
            return
        }
        manifold.type = ManifoldType.CIRCLES
        manifold.localPoint.set(circle1p)
        manifold.localNormal.setZero()
        manifold.pointCount = 1
        manifold.points[0].localPoint.set(circle2p)
        manifold.points[0].id.zero()
    }

    fun collidePolygonAndCircle(manifold: Manifold, polygon: PolygonShape, xfA: Transform, circle: CircleShape, xfB: Transform) {
        manifold.pointCount = 0
        val circlep = circle.m_p
        val xfBq = xfB.q
        val xfAq = xfA.q
        val cx = (xfBq.c * circlep.x - xfBq.s * circlep.y) + xfB.p.x
        val cy = (xfBq.s * circlep.x + xfBq.c * circlep.y) + xfB.p.y
        val px = cx - xfA.p.x
        val py = cy - xfA.p.y
        val cLocalx = xfAq.c * px + xfAq.s * py
        val cLocaly = -xfAq.s * px + xfAq.c * py

        var normalIndex = 0
        var separation = -Float.MAX_VALUE
        val radius = polygon.m_radius + circle.m_radius
        val vertexCount = polygon.m_count
        val vertices = polygon.m_vertices
        val normals = polygon.m_normals

        for (index in 0 until vertexCount) {
            val vertex = vertices[index]
            val tempX = cLocalx - vertex.x
            val tempY = cLocaly - vertex.y
            val s = normals[index].x * tempX + normals[index].y * tempY
            if (s > radius) {
                return
            }
            if (s > separation) {
                separation = s
                normalIndex = index
            }
        }

        val vertIndex1 = normalIndex
        val vertIndex2 = if (vertIndex1 + 1 < vertexCount) vertIndex1 + 1 else 0
        val localV1 = vertices[vertIndex1]
        val localV2 = vertices[vertIndex2]

        if (separation < Settings.EPSILON) {
            manifold.pointCount = 1
            manifold.type = ManifoldType.FACE_A
            val normal = normals[normalIndex]
            manifold.localNormal.x = normal.x
            manifold.localNormal.y = normal.y
            manifold.localPoint.x = (localV1.x + localV2.x) * 0.5f
            manifold.localPoint.y = (localV1.y + localV2.y) * 0.5f
            val mpoint = manifold.points[0]
            mpoint.localPoint.x = circlep.x
            mpoint.localPoint.y = circlep.y
            mpoint.id.zero()
            return
        }

        val tempX = cLocalx - localV1.x
        val tempY = cLocaly - localV1.y
        val temp2X = localV2.x - localV1.x
        val temp2Y = localV2.y - localV1.y
        val u1 = tempX * temp2X + tempY * temp2Y

        val temp3X = cLocalx - localV2.x
        val temp3Y = cLocaly - localV2.y
        val temp4X = localV1.x - localV2.x
        val temp4Y = localV1.y - localV2.y
        val u2 = temp3X * temp4X + temp3Y * temp4Y

        if (u1 <= 0f) {
            val dx = cLocalx - localV1.x
            val dy = cLocaly - localV1.y
            if (dx * dx + dy * dy > radius * radius) {
                return
            }
            manifold.pointCount = 1
            manifold.type = ManifoldType.FACE_A
            manifold.localNormal.x = cLocalx - localV1.x
            manifold.localNormal.y = cLocaly - localV1.y
            manifold.localNormal.normalize()
            manifold.localPoint.set(localV1)
            manifold.points[0].localPoint.set(circlep)
            manifold.points[0].id.zero()
        } else if (u2 <= 0f) {
            val dx = cLocalx - localV2.x
            val dy = cLocaly - localV2.y
            if (dx * dx + dy * dy > radius * radius) {
                return
            }
            manifold.pointCount = 1
            manifold.type = ManifoldType.FACE_A
            manifold.localNormal.x = cLocalx - localV2.x
            manifold.localNormal.y = cLocaly - localV2.y
            manifold.localNormal.normalize()
            manifold.localPoint.set(localV2)
            manifold.points[0].localPoint.set(circlep)
            manifold.points[0].id.zero()
        } else {
            val fcx = (localV1.x + localV2.x) * 0.5f
            val fcy = (localV1.y + localV2.y) * 0.5f
            val tx = cLocalx - fcx
            val ty = cLocaly - fcy
            val normal = normals[vertIndex1]
            separation = tx * normal.x + ty * normal.y
            if (separation > radius) {
                return
            }
            manifold.pointCount = 1
            manifold.type = ManifoldType.FACE_A
            manifold.localNormal.set(normal)
            manifold.localPoint.x = fcx
            manifold.localPoint.y = fcy
            manifold.points[0].localPoint.set(circlep)
            manifold.points[0].id.zero()
        }
    }

    fun findMaxSeparation(results: EdgeResults, poly1: PolygonShape, xf1: Transform, poly2: PolygonShape, xf2: Transform) {
        val count1 = poly1.m_count
        val count2 = poly2.m_count
        val n1s = poly1.m_normals
        val v1s = poly1.m_vertices
        val v2s = poly2.m_vertices

        Transform.mulTransToOutUnsafe(xf2, xf1, xf)
        val xfq = xf.q
        var bestIndex = 0
        var maxSeparation = -Float.MAX_VALUE
        for (index in 0 until count1) {
            Rot.mulToOutUnsafe(xfq, n1s[index], n)
            Transform.mulToOutUnsafe(xf, v1s[index], v1)
            var si = Float.MAX_VALUE
            for (j in 0 until count2) {
                val v2sj = v2s[j]
                val sij = n.x * (v2sj.x - v1.x) + n.y * (v2sj.y - v1.y)
                if (sij < si) {
                    si = sij
                }
            }
            if (si > maxSeparation) {
                maxSeparation = si
                bestIndex = index
            }
        }
        results.edgeIndex = bestIndex
        results.separation = maxSeparation
    }

    fun findIncidentEdge(c: Array<ClipVertex>, poly1: PolygonShape, xf1: Transform, edge1: Int, poly2: PolygonShape, xf2: Transform) {
        val count1 = poly1.m_count
        val normals1 = poly1.m_normals
        val count2 = poly2.m_count
        val vertices2 = poly2.m_vertices
        val normals2 = poly2.m_normals
        assert(edge1 in 0 until count1)
        val c0 = c[0]
        val c1 = c[1]
        val xf1q = xf1.q
        val xf2q = xf2.q

        val v = normals1[edge1]
        val tempX = xf1q.c * v.x - xf1q.s * v.y
        val tempY = xf1q.s * v.x + xf1q.c * v.y
        val normal1x = xf2q.c * tempX + xf2q.s * tempY
        val normal1y = -xf2q.s * tempX + xf2q.c * tempY

        var index = 0
        var minDot = Float.MAX_VALUE
        for (i in 0 until count2) {
            val b = normals2[i]
            val dot = normal1x * b.x + normal1y * b.y
            if (dot < minDot) {
                minDot = dot
                index = i
            }
        }

        val i1 = index
        val i2 = if (i1 + 1 < count2) i1 + 1 else 0

        val localV1 = vertices2[i1]
        val out = c0.v
        out.x = (xf2q.c * localV1.x - xf2q.s * localV1.y) + xf2.p.x
        out.y = (xf2q.s * localV1.x + xf2q.c * localV1.y) + xf2.p.y
        c0.id.indexA = edge1.toByte()
        c0.id.indexB = i1.toByte()
        c0.id.typeA = ContactID.Type.FACE.ordinal.toByte()
        c0.id.typeB = ContactID.Type.VERTEX.ordinal.toByte()

        val localV2 = vertices2[i2]
        val out1 = c1.v
        out1.x = (xf2q.c * localV2.x - xf2q.s * localV2.y) + xf2.p.x
        out1.y = (xf2q.s * localV2.x + xf2q.c * localV2.y) + xf2.p.y
        c1.id.indexA = edge1.toByte()
        c1.id.indexB = i2.toByte()
        c1.id.typeA = ContactID.Type.FACE.ordinal.toByte()
        c1.id.typeB = ContactID.Type.VERTEX.ordinal.toByte()
    }

    fun collidePolygons(manifold: Manifold, polyA: PolygonShape, xfA: Transform, polyB: PolygonShape, xfB: Transform) {
        manifold.pointCount = 0
        val totalRadius = polyA.m_radius + polyB.m_radius
        findMaxSeparation(results1, polyA, xfA, polyB, xfB)
        if (results1.separation > totalRadius) {
            return
        }
        findMaxSeparation(results2, polyB, xfB, polyA, xfA)
        if (results2.separation > totalRadius) {
            return
        }

        val poly1: PolygonShape
        val poly2: PolygonShape
        val xf1: Transform
        val xf2: Transform
        val edge1: Int
        val flip: Boolean
        val kTol = 0.1f * Settings.linearSlop
        if (results2.separation > results1.separation + kTol) {
            poly1 = polyB
            poly2 = polyA
            xf1 = xfB
            xf2 = xfA
            edge1 = results2.edgeIndex
            manifold.type = ManifoldType.FACE_B
            flip = true
        } else {
            poly1 = polyA
            poly2 = polyB
            xf1 = xfA
            xf2 = xfB
            edge1 = results1.edgeIndex
            manifold.type = ManifoldType.FACE_A
            flip = false
        }
        val xf1q = xf1.q
        findIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2)

        val count1 = poly1.m_count
        val vertices1 = poly1.m_vertices
        val iv1 = edge1
        val iv2 = if (edge1 + 1 < count1) edge1 + 1 else 0
        v11.set(vertices1[iv1])
        v12.set(vertices1[iv2])
        localTangent.x = v12.x - v11.x
        localTangent.y = v12.y - v11.y
        localTangent.normalize()
        localNormal.x = localTangent.y
        localNormal.y = -localTangent.x
        planePoint.x = (v11.x + v12.x) * 0.5f
        planePoint.y = (v11.y + v12.y) * 0.5f
        tangent.x = xf1q.c * localTangent.x - xf1q.s * localTangent.y
        tangent.y = xf1q.s * localTangent.x + xf1q.c * localTangent.y
        val normalx = tangent.y
        val normaly = -tangent.x
        Transform.mulToOut(xf1, v11, v11)
        Transform.mulToOut(xf1, v12, v12)
        val frontOffset = normalx * v11.x + normaly * v11.y
        val sideOffset1 = -(tangent.x * v11.x + tangent.y * v11.y) + totalRadius
        val sideOffset2 = tangent.x * v12.x + tangent.y * v12.y + totalRadius

        tangent.negateLocal()
        var np = clipSegmentToLine(clipPoints1, incidentEdge, tangent, sideOffset1, iv1)
        tangent.negateLocal()
        if (np < 2) {
            return
        }
        np = clipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2)
        if (np < 2) {
            return
        }

        manifold.localNormal.set(localNormal)
        manifold.localPoint.set(planePoint)
        var pointCount = 0
        for (index in 0 until Settings.maxManifoldPoints) {
            val separation = normalx * clipPoints2[index].v.x + normaly * clipPoints2[index].v.y - frontOffset
            if (separation <= totalRadius) {
                val cp = manifold.points[pointCount]
                val out = cp.localPoint
                val localPx = clipPoints2[index].v.x - xf2.p.x
                val localPy = clipPoints2[index].v.y - xf2.p.y
                out.x = xf2.q.c * localPx + xf2.q.s * localPy
                out.y = -xf2.q.s * localPx + xf2.q.c * localPy
                cp.id.set(clipPoints2[index].id)
                if (flip) {
                    cp.id.flip()
                }
                pointCount++
            }
        }
        manifold.pointCount = pointCount
    }

    fun collideEdgeAndCircle(manifold: Manifold, edgeA: EdgeShape, xfA: Transform, circleB: CircleShape, xfB: Transform) {
        manifold.pointCount = 0
        Transform.mulToOutUnsafe(xfB, circleB.m_p, temp)
        Transform.mulTransToOutUnsafe(xfA, temp, q)

        val a = edgeA.m_vertex1
        val b = edgeA.m_vertex2
        e.set(b).subLocal(a)
        val u = Vec2.dot(e, temp.set(b).subLocal(q))
        val v = Vec2.dot(e, temp.set(q).subLocal(a))
        val radius = edgeA.m_radius + circleB.m_radius

        cf.indexB = 0
        cf.typeB = ContactID.Type.VERTEX.ordinal.toByte()

        if (v <= 0f) {
            val localP = a
            d.set(q).subLocal(localP)
            val dd = Vec2.dot(d, d)
            if (dd > radius * radius) {
                return
            }
            if (edgeA.m_hasVertex0) {
                val a1 = edgeA.m_vertex0
                val b1 = a
                e1.set(b1).subLocal(a1)
                val u1 = Vec2.dot(e1, temp.set(b1).subLocal(q))
                if (u1 > 0f) {
                    return
                }
            }
            cf.indexA = 0
            cf.typeA = ContactID.Type.VERTEX.ordinal.toByte()
            manifold.pointCount = 1
            manifold.type = ManifoldType.CIRCLES
            manifold.localNormal.setZero()
            manifold.localPoint.set(localP)
            manifold.points[0].id.set(cf)
            manifold.points[0].localPoint.set(circleB.m_p)
            return
        }

        if (u <= 0f) {
            val localP = b
            d.set(q).subLocal(localP)
            val dd = Vec2.dot(d, d)
            if (dd > radius * radius) {
                return
            }
            if (edgeA.m_hasVertex3) {
                val b2 = edgeA.m_vertex3
                val a2 = b
                val e2 = e1
                e2.set(b2).subLocal(a2)
                val v2 = Vec2.dot(e2, temp.set(q).subLocal(a2))
                if (v2 > 0f) {
                    return
                }
            }
            cf.indexA = 1
            cf.typeA = ContactID.Type.VERTEX.ordinal.toByte()
            manifold.pointCount = 1
            manifold.type = ManifoldType.CIRCLES
            manifold.localNormal.setZero()
            manifold.localPoint.set(localP)
            manifold.points[0].id.set(cf)
            manifold.points[0].localPoint.set(circleB.m_p)
            return
        }

        val den = Vec2.dot(e, e)
        assert(den > 0f)
        p.set(a).mulLocal(u).addLocal(temp.set(b).mulLocal(v))
        p.mulLocal(1f / den)
        d.set(q).subLocal(p)
        val dd = Vec2.dot(d, d)
        if (dd > radius * radius) {
            return
        }

        n.x = -e.y
        n.y = e.x
        if (Vec2.dot(n, temp.set(q).subLocal(a)) < 0f) {
            n.set(-n.x, -n.y)
        }
        n.normalize()
        cf.indexA = 0
        cf.typeA = ContactID.Type.FACE.ordinal.toByte()
        manifold.pointCount = 1
        manifold.type = ManifoldType.FACE_A
        manifold.localNormal.set(n)
        manifold.localPoint.set(a)
        manifold.points[0].id.set(cf)
        manifold.points[0].localPoint.set(circleB.m_p)
    }

    fun collideEdgeAndPolygon(manifold: Manifold, edgeA: EdgeShape, xfA: Transform, polygonB: PolygonShape, xfB: Transform) {
        collider.collide(manifold, edgeA, xfA, polygonB, xfB)
    }

    class EdgeResults {
        var separation: Float = 0f
        var edgeIndex: Int = 0
    }

    class ClipVertex {
        val v: Vec2 = Vec2()
        val id: ContactID = ContactID()

        fun set(cv: ClipVertex) {
            v.x = cv.v.x
            v.y = cv.v.y
            id.indexA = cv.id.indexA
            id.indexB = cv.id.indexB
            id.typeA = cv.id.typeA
            id.typeB = cv.id.typeB
        }
    }

    enum class PointState {
        NULL_STATE,
        ADD_STATE,
        PERSIST_STATE,
        REMOVE_STATE,
    }

    class EPAxis {
        enum class Type {
            UNKNOWN,
            EDGE_A,
            EDGE_B,
        }

        var type: Type = Type.UNKNOWN
        var index: Int = 0
        var separation: Float = 0f
    }

    class TempPolygon {
        val vertices: Array<Vec2> = Array(Settings.maxPolygonVertices) { Vec2() }
        val normals: Array<Vec2> = Array(Settings.maxPolygonVertices) { Vec2() }
        var count: Int = 0
    }

    class ReferenceFace {
        var i1: Int = 0
        var i2: Int = 0
        val v1: Vec2 = Vec2()
        val v2: Vec2 = Vec2()
        val normal: Vec2 = Vec2()
        val sideNormal1: Vec2 = Vec2()
        var sideOffset1: Float = 0f
        val sideNormal2: Vec2 = Vec2()
        var sideOffset2: Float = 0f
    }

    class EPCollider {
        enum class VertexType {
            ISOLATED,
            CONCAVE,
            CONVEX,
        }

        val m_polygonB = TempPolygon()
        val m_xf = Transform()
        val m_centroidB = Vec2()
        var m_v0 = Vec2()
        var m_v1 = Vec2()
        var m_v2 = Vec2()
        var m_v3 = Vec2()
        val m_normal0 = Vec2()
        val m_normal1 = Vec2()
        val m_normal2 = Vec2()
        val m_normal = Vec2()
        var m_type1: VertexType = VertexType.ISOLATED
        var m_type2: VertexType = VertexType.ISOLATED
        val m_lowerLimit = Vec2()
        val m_upperLimit = Vec2()
        var m_radius: Float = 0f
        var m_front: Boolean = false

        private val edge1 = Vec2()
        private val temp = Vec2()
        private val edge0 = Vec2()
        private val edge2 = Vec2()
        private val ie = Array(2) { ClipVertex() }
        private val clipPoints1 = Array(2) { ClipVertex() }
        private val clipPoints2 = Array(2) { ClipVertex() }
        private val rf = ReferenceFace()
        private val edgeAxis = EPAxis()
        private val polygonAxis = EPAxis()
        private val perp = Vec2()
        private val n = Vec2()

        fun collide(manifold: Manifold, edgeA: EdgeShape, xfA: Transform, polygonB: PolygonShape, xfB: Transform) {
            Transform.mulTransToOutUnsafe(xfA, xfB, m_xf)
            Transform.mulToOutUnsafe(m_xf, polygonB.m_centroid, m_centroidB)
            m_v0 = edgeA.m_vertex0
            m_v1 = edgeA.m_vertex1
            m_v2 = edgeA.m_vertex2
            m_v3 = edgeA.m_vertex3
            val hasVertex0 = edgeA.m_hasVertex0
            val hasVertex3 = edgeA.m_hasVertex3
            edge1.set(m_v2).subLocal(m_v1)
            edge1.normalize()
            m_normal1.set(edge1.y, -edge1.x)
            val offset1 = Vec2.dot(m_normal1, temp.set(m_centroidB).subLocal(m_v1))
            var offset0 = 0f
            var offset2 = 0f
            var convex1 = false
            var convex2 = false

            if (hasVertex0) {
                edge0.set(m_v1).subLocal(m_v0)
                edge0.normalize()
                m_normal0.set(edge0.y, -edge0.x)
                convex1 = Vec2.cross(edge0, edge1) >= 0f
                offset0 = Vec2.dot(m_normal0, temp.set(m_centroidB).subLocal(m_v0))
            }
            if (hasVertex3) {
                edge2.set(m_v3).subLocal(m_v2)
                edge2.normalize()
                m_normal2.set(edge2.y, -edge2.x)
                convex2 = Vec2.cross(edge1, edge2) > 0f
                offset2 = Vec2.dot(m_normal2, temp.set(m_centroidB).subLocal(m_v2))
            }

            if (hasVertex0 && hasVertex3) {
                if (convex1 && convex2) {
                    m_front = offset0 >= 0f || offset1 >= 0f || offset2 >= 0f
                    if (m_front) {
                        m_normal.set(m_normal1)
                        m_lowerLimit.set(m_normal0)
                        m_upperLimit.set(m_normal2)
                    } else {
                        m_normal.set(-m_normal1.x, -m_normal1.y)
                        m_lowerLimit.set(-m_normal1.x, -m_normal1.y)
                        m_upperLimit.set(-m_normal1.x, -m_normal1.y)
                    }
                } else if (convex1) {
                    m_front = offset0 >= 0f || (offset1 >= 0f && offset2 >= 0f)
                    if (m_front) {
                        m_normal.set(m_normal1)
                        m_lowerLimit.set(m_normal0)
                        m_upperLimit.set(m_normal1)
                    } else {
                        m_normal.set(-m_normal1.x, -m_normal1.y)
                        m_lowerLimit.set(-m_normal2.x, -m_normal2.y)
                        m_upperLimit.set(-m_normal1.x, -m_normal1.y)
                    }
                } else if (convex2) {
                    m_front = offset2 >= 0f || (offset0 >= 0f && offset1 >= 0f)
                    if (m_front) {
                        m_normal.set(m_normal1)
                        m_lowerLimit.set(m_normal1)
                        m_upperLimit.set(m_normal2)
                    } else {
                        m_normal.set(-m_normal1.x, -m_normal1.y)
                        m_lowerLimit.set(-m_normal1.x, -m_normal1.y)
                        m_upperLimit.set(-m_normal0.x, -m_normal0.y)
                    }
                } else {
                    m_front = offset0 >= 0f && offset1 >= 0f && offset2 >= 0f
                    if (m_front) {
                        m_normal.set(m_normal1)
                        m_lowerLimit.set(m_normal1)
                        m_upperLimit.set(m_normal1)
                    } else {
                        m_normal.set(-m_normal1.x, -m_normal1.y)
                        m_lowerLimit.set(-m_normal2.x, -m_normal2.y)
                        m_upperLimit.set(-m_normal0.x, -m_normal0.y)
                    }
                }
            } else if (hasVertex0) {
                if (convex1) {
                    m_front = offset0 >= 0f || offset1 >= 0f
                    if (m_front) {
                        m_normal.set(m_normal1)
                        m_lowerLimit.set(m_normal0)
                        m_upperLimit.set(-m_normal1.x, -m_normal1.y)
                    } else {
                        m_normal.set(-m_normal1.x, -m_normal1.y)
                        m_lowerLimit.set(m_normal1)
                        m_upperLimit.set(-m_normal1.x, -m_normal1.y)
                    }
                } else {
                    m_front = offset0 >= 0f && offset1 >= 0f
                    if (m_front) {
                        m_normal.set(m_normal1)
                        m_lowerLimit.set(m_normal1)
                        m_upperLimit.set(-m_normal1.x, -m_normal1.y)
                    } else {
                        m_normal.set(-m_normal1.x, -m_normal1.y)
                        m_lowerLimit.set(m_normal1)
                        m_upperLimit.set(-m_normal0.x, -m_normal0.y)
                    }
                }
            } else if (hasVertex3) {
                if (convex2) {
                    m_front = offset1 >= 0f || offset2 >= 0f
                    if (m_front) {
                        m_normal.set(m_normal1)
                        m_lowerLimit.set(-m_normal1.x, -m_normal1.y)
                        m_upperLimit.set(m_normal2)
                    } else {
                        m_normal.set(-m_normal1.x, -m_normal1.y)
                        m_lowerLimit.set(-m_normal1.x, -m_normal1.y)
                        m_upperLimit.set(m_normal1)
                    }
                } else {
                    m_front = offset1 >= 0f && offset2 >= 0f
                    if (m_front) {
                        m_normal.set(m_normal1)
                        m_lowerLimit.set(-m_normal1.x, -m_normal1.y)
                        m_upperLimit.set(m_normal1)
                    } else {
                        m_normal.set(-m_normal1.x, -m_normal1.y)
                        m_lowerLimit.set(-m_normal2.x, -m_normal2.y)
                        m_upperLimit.set(m_normal1)
                    }
                }
            } else {
                m_front = offset1 >= 0f
                if (m_front) {
                    m_normal.set(m_normal1)
                    m_lowerLimit.set(-m_normal1.x, -m_normal1.y)
                    m_upperLimit.set(-m_normal1.x, -m_normal1.y)
                } else {
                    m_normal.set(-m_normal1.x, -m_normal1.y)
                    m_lowerLimit.set(m_normal1)
                    m_upperLimit.set(m_normal1)
                }
            }

            m_polygonB.count = polygonB.m_count
            for (index in 0 until polygonB.m_count) {
                Transform.mulToOutUnsafe(m_xf, polygonB.m_vertices[index], m_polygonB.vertices[index])
                Rot.mulToOutUnsafe(m_xf.q, polygonB.m_normals[index], m_polygonB.normals[index])
            }

            m_radius = 2f * Settings.polygonRadius
            manifold.pointCount = 0
            computeEdgeSeparation(edgeAxis)
            if (edgeAxis.type == EPAxis.Type.UNKNOWN) {
                return
            }
            if (edgeAxis.separation > m_radius) {
                return
            }
            computePolygonSeparation(polygonAxis)
            if (polygonAxis.type != EPAxis.Type.UNKNOWN && polygonAxis.separation > m_radius) {
                return
            }

            val kRelativeTol = 0.98f
            val kAbsoluteTol = 0.001f
            val primaryAxis = if (polygonAxis.type == EPAxis.Type.UNKNOWN) {
                edgeAxis
            } else if (polygonAxis.separation > kRelativeTol * edgeAxis.separation + kAbsoluteTol) {
                polygonAxis
            } else {
                edgeAxis
            }

            val ie0 = ie[0]
            val ie1 = ie[1]
            if (primaryAxis.type == EPAxis.Type.EDGE_A) {
                manifold.type = ManifoldType.FACE_A
                var bestIndex = 0
                var bestValue = Vec2.dot(m_normal, m_polygonB.normals[0])
                for (index in 1 until m_polygonB.count) {
                    val value = Vec2.dot(m_normal, m_polygonB.normals[index])
                    if (value < bestValue) {
                        bestValue = value
                        bestIndex = index
                    }
                }
                val i1 = bestIndex
                val i2 = if (i1 + 1 < m_polygonB.count) i1 + 1 else 0
                ie0.v.set(m_polygonB.vertices[i1])
                ie0.id.indexA = 0
                ie0.id.indexB = i1.toByte()
                ie0.id.typeA = ContactID.Type.FACE.ordinal.toByte()
                ie0.id.typeB = ContactID.Type.VERTEX.ordinal.toByte()
                ie1.v.set(m_polygonB.vertices[i2])
                ie1.id.indexA = 0
                ie1.id.indexB = i2.toByte()
                ie1.id.typeA = ContactID.Type.FACE.ordinal.toByte()
                ie1.id.typeB = ContactID.Type.VERTEX.ordinal.toByte()
                if (m_front) {
                    rf.i1 = 0
                    rf.i2 = 1
                    rf.v1.set(m_v1)
                    rf.v2.set(m_v2)
                    rf.normal.set(m_normal1)
                } else {
                    rf.i1 = 1
                    rf.i2 = 0
                    rf.v1.set(m_v2)
                    rf.v2.set(m_v1)
                    rf.normal.set(m_normal1).negateLocal()
                }
            } else {
                manifold.type = ManifoldType.FACE_B
                ie0.v.set(m_v1)
                ie0.id.indexA = 0
                ie0.id.indexB = primaryAxis.index.toByte()
                ie0.id.typeA = ContactID.Type.VERTEX.ordinal.toByte()
                ie0.id.typeB = ContactID.Type.FACE.ordinal.toByte()
                ie1.v.set(m_v2)
                ie1.id.indexA = 0
                ie1.id.indexB = primaryAxis.index.toByte()
                ie1.id.typeA = ContactID.Type.VERTEX.ordinal.toByte()
                ie1.id.typeB = ContactID.Type.FACE.ordinal.toByte()
                rf.i1 = primaryAxis.index
                rf.i2 = if (rf.i1 + 1 < m_polygonB.count) rf.i1 + 1 else 0
                rf.v1.set(m_polygonB.vertices[rf.i1])
                rf.v2.set(m_polygonB.vertices[rf.i2])
                rf.normal.set(m_polygonB.normals[rf.i1])
            }

            rf.sideNormal1.set(rf.normal.y, -rf.normal.x)
            rf.sideNormal2.set(rf.sideNormal1).negateLocal()
            rf.sideOffset1 = Vec2.dot(rf.sideNormal1, rf.v1)
            rf.sideOffset2 = Vec2.dot(rf.sideNormal2, rf.v2)

            var np = clipSegmentToLine(clipPoints1, ie, rf.sideNormal1, rf.sideOffset1, rf.i1)
            if (np < Settings.maxManifoldPoints) {
                return
            }
            np = clipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2)
            if (np < Settings.maxManifoldPoints) {
                return
            }

            if (primaryAxis.type == EPAxis.Type.EDGE_A) {
                manifold.localNormal.set(rf.normal)
                manifold.localPoint.set(rf.v1)
            } else {
                manifold.localNormal.set(polygonB.m_normals[rf.i1])
                manifold.localPoint.set(polygonB.m_vertices[rf.i1])
            }

            var pointCount = 0
            for (index in 0 until Settings.maxManifoldPoints) {
                val separation = Vec2.dot(rf.normal, temp.set(clipPoints2[index].v).subLocal(rf.v1))
                if (separation <= m_radius) {
                    val cp = manifold.points[pointCount]
                    if (primaryAxis.type == EPAxis.Type.EDGE_A) {
                        Transform.mulTransToOutUnsafe(m_xf, clipPoints2[index].v, cp.localPoint)
                        cp.id.set(clipPoints2[index].id)
                    } else {
                        cp.localPoint.set(clipPoints2[index].v)
                        cp.id.typeA = clipPoints2[index].id.typeB
                        cp.id.typeB = clipPoints2[index].id.typeA
                        cp.id.indexA = clipPoints2[index].id.indexB
                        cp.id.indexB = clipPoints2[index].id.indexA
                    }
                    pointCount++
                }
            }
            manifold.pointCount = pointCount
        }

        fun computeEdgeSeparation(axis: EPAxis) {
            axis.type = EPAxis.Type.EDGE_A
            axis.index = if (m_front) 0 else 1
            axis.separation = Float.MAX_VALUE
            val nx = m_normal.x
            val ny = m_normal.y
            for (index in 0 until m_polygonB.count) {
                val localV = m_polygonB.vertices[index]
                val tempX = localV.x - m_v1.x
                val tempY = localV.y - m_v1.y
                val s = nx * tempX + ny * tempY
                if (s < axis.separation) {
                    axis.separation = s
                }
            }
        }

        fun computePolygonSeparation(axis: EPAxis) {
            axis.type = EPAxis.Type.UNKNOWN
            axis.index = -1
            axis.separation = -Float.MAX_VALUE
            perp.x = -m_normal.y
            perp.y = m_normal.x
            for (index in 0 until m_polygonB.count) {
                val normalB = m_polygonB.normals[index]
                val vB = m_polygonB.vertices[index]
                n.x = -normalB.x
                n.y = -normalB.y
                var tempX = vB.x - m_v1.x
                var tempY = vB.y - m_v1.y
                val s1 = n.x * tempX + n.y * tempY
                tempX = vB.x - m_v2.x
                tempY = vB.y - m_v2.y
                val s2 = n.x * tempX + n.y * tempY
                val s = MathUtils.min(s1, s2)
                if (s > m_radius) {
                    axis.type = EPAxis.Type.EDGE_B
                    axis.index = index
                    axis.separation = s
                    return
                }
                if (n.x * perp.x + n.y * perp.y >= 0f) {
                    if (Vec2.dot(temp.set(n).subLocal(m_upperLimit), m_normal) < -Settings.angularSlop) {
                        continue
                    }
                } else {
                    if (Vec2.dot(temp.set(n).subLocal(m_lowerLimit), m_normal) < -Settings.angularSlop) {
                        continue
                    }
                }
                if (s > axis.separation) {
                    axis.type = EPAxis.Type.EDGE_B
                    axis.index = index
                    axis.separation = s
                }
            }
        }
    }

    companion object {
        const val NULL_FEATURE: Int = Int.MAX_VALUE
        private val d = Vec2()

        fun getPointStates(state1: Array<PointState>, state2: Array<PointState>, manifold1: Manifold, manifold2: Manifold) {
            for (index in 0 until Settings.maxManifoldPoints) {
                state1[index] = PointState.NULL_STATE
                state2[index] = PointState.NULL_STATE
            }
            for (index in 0 until manifold1.pointCount) {
                val id = manifold1.points[index].id
                state1[index] = PointState.REMOVE_STATE
                for (j in 0 until manifold2.pointCount) {
                    if (manifold2.points[j].id.isEqual(id)) {
                        state1[index] = PointState.PERSIST_STATE
                        break
                    }
                }
            }
            for (index in 0 until manifold2.pointCount) {
                val id = manifold2.points[index].id
                state2[index] = PointState.ADD_STATE
                for (j in 0 until manifold1.pointCount) {
                    if (manifold1.points[j].id.isEqual(id)) {
                        state2[index] = PointState.PERSIST_STATE
                        break
                    }
                }
            }
        }

        fun clipSegmentToLine(vOut: Array<ClipVertex>, vIn: Array<ClipVertex>, normal: Vec2, offset: Float, vertexIndexA: Int): Int {
            var numOut = 0
            val vIn0 = vIn[0]
            val vIn1 = vIn[1]
            val vIn0v = vIn0.v
            val vIn1v = vIn1.v
            val distance0 = Vec2.dot(normal, vIn0v) - offset
            val distance1 = Vec2.dot(normal, vIn1v) - offset
            if (distance0 <= 0f) {
                vOut[numOut++].set(vIn0)
            }
            if (distance1 <= 0f) {
                vOut[numOut++].set(vIn1)
            }
            if (distance0 * distance1 < 0f) {
                val interp = distance0 / (distance0 - distance1)
                val out = vOut[numOut]
                out.v.x = vIn0v.x + interp * (vIn1v.x - vIn0v.x)
                out.v.y = vIn0v.y + interp * (vIn1v.y - vIn0v.y)
                out.id.indexA = vertexIndexA.toByte()
                out.id.indexB = vIn0.id.indexB
                out.id.typeA = ContactID.Type.VERTEX.ordinal.toByte()
                out.id.typeB = ContactID.Type.FACE.ordinal.toByte()
                numOut++
            }
            return numOut
        }
    }
}
