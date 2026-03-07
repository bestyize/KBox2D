package xyz.thewind.kbox2d

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.safeContentPadding
import androidx.compose.foundation.layout.size
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.runtime.withFrameNanos
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Fill
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.collision.shapes.PolygonShape
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.Body
import xyz.thewind.kbox2d.dynamics.BodyDef
import xyz.thewind.kbox2d.dynamics.BodyType
import xyz.thewind.kbox2d.dynamics.Fixture
import xyz.thewind.kbox2d.dynamics.FixtureDef
import xyz.thewind.kbox2d.dynamics.World
import xyz.thewind.kbox2d.particle.ParticleColor
import xyz.thewind.kbox2d.particle.ParticleDef
import xyz.thewind.kbox2d.particle.ParticleGroupDef
import xyz.thewind.kbox2d.particle.ParticleGroupType
import xyz.thewind.kbox2d.particle.ParticleType

private data class DemoScene(
    val world: World,
    val worldWidth: Float,
    val worldHeight: Float,
)

private object DemoSceneFactory {
    fun create(): DemoScene {
        val world = World(gravity = Vec2(0f, -9.8f))
        world.setParticleRadius(0.16f)
        buildBounds(world)
        buildPlatforms(world)
        buildStacks(world)
        buildBalls(world)
        buildParticleBlob(world)
        return DemoScene(world = world, worldWidth = 16f, worldHeight = 10f)
    }

    fun spawnBall(world: World, seed: Int) {
        val x = 3f + (seed % 9) * 1.1f
        createCircleBody(world, x = x, y = 9f, radius = 0.32f, density = 1.1f)
    }

    fun spawnParticleBurst(world: World, seed: Int) {
        val centerX = 4f + (seed % 7) * 1.3f
        world.createParticleGroup(ParticleGroupDef().apply {
            flags = ParticleType.b2_viscousParticle or ParticleType.b2_colorMixingParticle or ParticleType.b2_tensileParticle
            groupFlags = ParticleGroupType.b2_solidParticleGroup
            position.set(centerX, 8.2f)
            linearVelocity.set(0f, -1f)
            color = ParticleColor(64, 176.toByte(), 255.toByte(), 220.toByte())
            shape = CircleShape().apply {
                m_p.setZero()
                m_radius = 0.55f
            }
        })
    }

    private fun buildBounds(world: World) {
        createBoxBody(world, x = 8f, y = 0.35f, halfWidth = 8.2f, halfHeight = 0.35f, type = BodyType.STATIC)
        createBoxBody(world, x = 0.2f, y = 5f, halfWidth = 0.2f, halfHeight = 5.2f, type = BodyType.STATIC)
        createBoxBody(world, x = 15.8f, y = 5f, halfWidth = 0.2f, halfHeight = 5.2f, type = BodyType.STATIC)
    }

    private fun buildPlatforms(world: World) {
        createBoxBody(world, x = 4.1f, y = 2.9f, halfWidth = 2.2f, halfHeight = 0.16f, angle = 0.18f, type = BodyType.STATIC)
        createBoxBody(world, x = 11.8f, y = 5.1f, halfWidth = 2.5f, halfHeight = 0.16f, angle = -0.22f, type = BodyType.STATIC)
        createBoxBody(world, x = 7.9f, y = 7.0f, halfWidth = 1.4f, halfHeight = 0.14f, angle = 0.08f, type = BodyType.STATIC)
    }

    private fun buildStacks(world: World) {
        for (column in 0 until 4) {
            for (row in 0 until 4) {
                createBoxBody(
                    world = world,
                    x = 2.6f + column * 0.75f,
                    y = 1.0f + row * 0.72f,
                    halfWidth = 0.28f,
                    halfHeight = 0.28f,
                    type = BodyType.DYNAMIC,
                    density = 1f,
                )
            }
        }
    }

    private fun buildBalls(world: World) {
        repeat(5) { index ->
            createCircleBody(
                world = world,
                x = 9.5f + index * 0.8f,
                y = 8.6f + index * 0.18f,
                radius = 0.3f,
                density = 0.9f,
            )
        }
    }

    private fun buildParticleBlob(world: World) {
        world.createParticleGroup(ParticleGroupDef().apply {
            flags = ParticleType.b2_viscousParticle or ParticleType.b2_colorMixingParticle or ParticleType.b2_tensileParticle
            groupFlags = ParticleGroupType.b2_solidParticleGroup
            position.set(12.6f, 8.1f)
            color = ParticleColor(82, 160.toByte(), 255.toByte(), 220.toByte())
            shape = CircleShape().apply {
                m_p.setZero()
                m_radius = 1.05f
            }
        })

        repeat(8) { index ->
            world.createParticle(ParticleDef().apply {
                flags = ParticleType.b2_wallParticle
                position.set(1.4f + index * 0.38f, 9.25f)
                color = ParticleColor(255.toByte(), 220.toByte(), 120.toByte(), 255.toByte())
            })
        }
    }

    private fun createBoxBody(
        world: World,
        x: Float,
        y: Float,
        halfWidth: Float,
        halfHeight: Float,
        type: BodyType,
        density: Float = 0f,
        angle: Float = 0f,
    ): Body {
        val body = requireNotNull(world.createBody(BodyDef().apply {
            this.type = type
            position = Vec2(x, y)
            this.angle = angle
        }))
        body.createFixture(FixtureDef().apply {
            shape = PolygonShape().apply { setAsBox(halfWidth, halfHeight) }
            this.density = density
            friction = 0.4f
            restitution = 0.15f
        })
        return body
    }

    private fun createCircleBody(
        world: World,
        x: Float,
        y: Float,
        radius: Float,
        density: Float,
    ): Body {
        val body = requireNotNull(world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(x, y)
        }))
        body.createFixture(FixtureDef().apply {
            shape = CircleShape().apply {
                m_p.setZero()
                m_radius = radius
            }
            this.density = density
            friction = 0.25f
            restitution = 0.35f
        })
        return body
    }
}

@Composable
fun KBox2dDemoScreen() {
    var paused by remember { mutableStateOf(false) }
    var sceneVersion by remember { mutableIntStateOf(0) }
    var spawnSeed by remember { mutableIntStateOf(0) }
    var frameTick by remember { mutableIntStateOf(0) }
    val scene = remember(sceneVersion) { DemoSceneFactory.create() }

    LaunchedEffect(scene, paused) {
        while (true) {
            withFrameNanos {
                if (!paused) {
                    scene.world.step(1f / 60f, 8, 3)
                    frameTick++
                }
            }
        }
    }

    MaterialTheme {
        Surface(
            modifier = Modifier
                .fillMaxSize()
                .background(Color(0xFF0D1117)),
            color = Color(0xFF0D1117),
        ) {
            Column(
                modifier = Modifier
                    .safeContentPadding()
                    .fillMaxSize()
                    .padding(16.dp),
                verticalArrangement = Arrangement.spacedBy(12.dp),
            ) {
                Text(
                    text = "KBox2D Compose Demo",
                    style = MaterialTheme.typography.headlineSmall,
                    color = Color(0xFFF4F7FB),
                )
                Text(
                    text = "Bodies=${scene.world.getBodyCount()}  Particles=${scene.world.getParticleCount()}  Contacts=${scene.world.getParticleContactCount()}  Frame=$frameTick",
                    style = MaterialTheme.typography.bodyMedium,
                    color = Color(0xFFB8C1CC),
                )
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    Button(onClick = { paused = !paused }) {
                        Text(if (paused) "继续" else "暂停")
                    }
                    Button(onClick = {
                        sceneVersion++
                        frameTick = 0
                    }) {
                        Text("重置场景")
                    }
                    Button(onClick = {
                        spawnSeed++
                        DemoSceneFactory.spawnBall(scene.world, spawnSeed)
                        frameTick++
                    }) {
                        Text("投放小球")
                    }
                    Button(onClick = {
                        spawnSeed++
                        DemoSceneFactory.spawnParticleBurst(scene.world, spawnSeed)
                        frameTick++
                    }) {
                        Text("投放粒子")
                    }
                }
                Box(
                    modifier = Modifier
                        .fillMaxWidth()
                        .weight(1f)
                        .background(Color(0xFF111722)),
                ) {
                    Canvas(modifier = Modifier.fillMaxSize()) {
                        drawSceneBackground(scene)
                        drawParticles(scene)
                        drawBodies(scene)
                    }
                    Text(
                        text = "显示内容：静态边界 + 斜坡 + 盒子堆 + 小球 + 粒子团",
                        color = Color(0xFF8FA3B8),
                        modifier = Modifier
                            .align(Alignment.BottomStart)
                            .padding(12.dp),
                        style = MaterialTheme.typography.bodySmall,
                    )
                }
            }
        }
    }
}

private fun DrawScope.drawSceneBackground(scene: DemoScene) {
    drawRect(Color(0xFF111722))
    val transform = DemoViewport(scene = scene, canvasSize = size)
    for (x in 0..scene.worldWidth.toInt()) {
        val start = transform.worldToScreen(Vec2(x.toFloat(), 0f))
        val end = transform.worldToScreen(Vec2(x.toFloat(), scene.worldHeight))
        drawLine(
            color = Color(0x1FFFFFFF),
            start = start,
            end = end,
            strokeWidth = 1f,
        )
    }
    for (y in 0..scene.worldHeight.toInt()) {
        val start = transform.worldToScreen(Vec2(0f, y.toFloat()))
        val end = transform.worldToScreen(Vec2(scene.worldWidth, y.toFloat()))
        drawLine(
            color = Color(0x18FFFFFF),
            start = start,
            end = end,
            strokeWidth = 1f,
        )
    }
}

private fun DrawScope.drawBodies(scene: DemoScene) {
    val viewport = DemoViewport(scene = scene, canvasSize = size)
    var body = scene.world.getBodyList()
    while (body != null) {
        var fixture = body.getFixtureList()
        while (fixture != null) {
            when (val shape = fixture.getShape()) {
                is CircleShape -> drawDemoCircle(body, fixture, shape, viewport)
                is PolygonShape -> drawDemoPolygon(body, fixture, shape, viewport)
            }
            fixture = fixture.getNext()
        }
        body = body.getNext()
    }
}

private fun DrawScope.drawParticles(scene: DemoScene) {
    val viewport = DemoViewport(scene = scene, canvasSize = size)
    val positions = scene.world.getParticlePositionBuffer()
    val colors = scene.world.getParticleColorBuffer()
    val radiusPx = viewport.scale * scene.world.getParticleRadius()
    positions.forEachIndexed { index, position ->
        val color = colors.getOrNull(index)?.toComposeColor() ?: Color(0xFF6BCBFF)
        drawCircle(
            color = color,
            radius = radiusPx,
            center = viewport.worldToScreen(position),
        )
    }
}

private fun DrawScope.drawDemoCircle(
    body: Body,
    fixture: Fixture,
    shape: CircleShape,
    viewport: DemoViewport,
) {
    val center = bodyLocalToWorld(body, shape.m_p)
    val screenCenter = viewport.worldToScreen(center)
    val radiusPx = shape.m_radius * viewport.scale
    val fill = bodyColor(body.getType(), fixture)
    drawCircle(color = fill, radius = radiusPx, center = screenCenter)
    val axisWorld = bodyLocalToWorld(body, Vec2(shape.m_p.x + shape.m_radius, shape.m_p.y))
    drawLine(
        color = Color.White.copy(alpha = 0.85f),
        start = screenCenter,
        end = viewport.worldToScreen(axisWorld),
        strokeWidth = 2f,
    )
}

private fun DrawScope.drawDemoPolygon(
    body: Body,
    fixture: Fixture,
    shape: PolygonShape,
    viewport: DemoViewport,
) {
    if (shape.m_count <= 1) return
    val path = Path()
    for (index in 0 until shape.m_count) {
        val worldPoint = bodyLocalToWorld(body, shape.m_vertices[index])
        val screen = viewport.worldToScreen(worldPoint)
        if (index == 0) {
            path.moveTo(screen.x, screen.y)
        } else {
            path.lineTo(screen.x, screen.y)
        }
    }
    path.close()
    val fill = bodyColor(body.getType(), fixture)
    drawPath(path = path, color = fill, style = Fill)
    drawPath(path = path, color = Color.White.copy(alpha = 0.2f), style = Stroke(width = 1.5f))
}

private fun bodyColor(type: BodyType, fixture: Fixture): Color = when (type) {
    BodyType.STATIC -> Color(0xFF3A4658)
    BodyType.KINEMATIC -> Color(0xFF8ECAE6)
    BodyType.DYNAMIC -> when (fixture.getShape()) {
        is CircleShape -> Color(0xFFFFB703)
        else -> Color(0xFF8EEC82)
    }
}

private fun bodyLocalToWorld(body: Body, local: Vec2): Vec2 {
    val angle = body.getAngle()
    val cosAngle = cos(angle)
    val sinAngle = sin(angle)
    val position = body.getPosition()
    return Vec2(
        position.x + cosAngle * local.x - sinAngle * local.y,
        position.y + sinAngle * local.x + cosAngle * local.y,
    )
}

private data class DemoViewport(
    val scene: DemoScene,
    val canvasSize: Size,
) {
    val scale: Float = min(canvasSize.width / scene.worldWidth, canvasSize.height / scene.worldHeight) * 0.94f
    private val offsetX: Float = (canvasSize.width - scene.worldWidth * scale) / 2f
    private val offsetY: Float = (canvasSize.height - scene.worldHeight * scale) / 2f

    fun worldToScreen(point: Vec2): Offset = Offset(
        x = offsetX + point.x * scale,
        y = canvasSize.height - offsetY - point.y * scale,
    )
}

private fun ParticleColor.toComposeColor(): Color = Color(
    red = r.toInt() and 0xFF,
    green = g.toInt() and 0xFF,
    blue = b.toInt() and 0xFF,
    alpha = a.toInt() and 0xFF,
)

@Preview
@Composable
private fun KBox2dDemoPreview() {
    KBox2dDemoScreen()
}
