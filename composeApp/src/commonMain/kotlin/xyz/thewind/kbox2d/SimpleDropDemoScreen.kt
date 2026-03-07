package xyz.thewind.kbox2d

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.safeContentPadding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.ui.draw.clip
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateListOf
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
import androidx.compose.ui.graphics.drawscope.Fill
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin
import kotlin.random.Random
import xyz.thewind.kbox2d.collision.shapes.CircleShape
import xyz.thewind.kbox2d.collision.shapes.PolygonShape
import xyz.thewind.kbox2d.common.Vec2
import xyz.thewind.kbox2d.dynamics.Body
import xyz.thewind.kbox2d.dynamics.BodyDef
import xyz.thewind.kbox2d.dynamics.BodyType
import xyz.thewind.kbox2d.dynamics.Fixture
import xyz.thewind.kbox2d.dynamics.FixtureDef
import xyz.thewind.kbox2d.dynamics.World

private enum class SpawnMode { RANDOM, BALL, BOX }

private data class SimpleScene(
    val world: World,
    val worldWidth: Float,
    val worldHeight: Float,
)

private object SimpleSceneFactory {
    private const val TAG_BOUNDARY = "boundary"
    private const val TAG_OBSTACLE = "obstacle"
    private const val TAG_BALL = "ball"
    private const val TAG_BOX = "box"
    private const val WORLD_WIDTH = 12f
    private const val WORLD_HEIGHT = 16f
    private const val WALL_THICKNESS = 0.16f

    fun create(): SimpleScene {
        val world = World(gravity = Vec2(0f, -10f))
        addBounds(world)
        addWallObstacle(world)
        addCenterObstacles(world)
        addBalls(world)
        addBoxes(world)
        return SimpleScene(world = world, worldWidth = WORLD_WIDTH, worldHeight = WORLD_HEIGHT)
    }

    fun spawnRandomBody(world: World, worldWidth: Float, worldHeight: Float, x: Float, y: Float, seed: Int) {
        val random = Random(seed)
        val spawnX = x.coerceIn(0.9f, worldWidth - 0.9f)
        val spawnY = y.coerceIn(1.0f, worldHeight - 0.8f)
        if (random.nextBoolean()) {
            spawnBall(world, worldWidth, worldHeight, spawnX, spawnY, seed)
        } else {
            spawnBox(world, worldWidth, worldHeight, spawnX, spawnY, seed)
        }
    }


    fun spawnBall(world: World, worldWidth: Float, worldHeight: Float, x: Float, y: Float, seed: Int) {
        val random = Random(seed)
        val spawnX = x.coerceIn(0.9f, worldWidth - 0.9f)
        val spawnY = y.coerceIn(1.0f, worldHeight - 0.8f)
        addCircle(
            world = world,
            x = spawnX,
            y = spawnY,
            radius = random.nextFloat() * 0.16f + 0.28f,
            restitution = random.nextFloat() * 0.25f + 0.55f,
            tag = TAG_BALL,
        )
    }

    fun spawnBox(world: World, worldWidth: Float, worldHeight: Float, x: Float, y: Float, seed: Int) {
        val random = Random(seed)
        val spawnX = x.coerceIn(0.9f, worldWidth - 0.9f)
        val spawnY = y.coerceIn(1.0f, worldHeight - 0.8f)
        val half = random.nextFloat() * 0.12f + 0.28f
        addBox(
            world = world,
            x = spawnX,
            y = spawnY,
            hx = half,
            hy = half,
            type = BodyType.DYNAMIC,
            restitution = random.nextFloat() * 0.2f + 0.35f,
            angle = random.nextFloat() * 0.5f - 0.25f,
            tag = TAG_BOX,
        )
    }

    fun clearDynamicBodies(world: World) {
        val toDestroy = mutableListOf<Body>()
        var body = world.getBodyList()
        while (body != null) {
            if (body.getType() == BodyType.DYNAMIC) {
                toDestroy += body
            }
            body = body.getNext()
        }
        toDestroy.forEach(world::destroyBody)
    }

    private fun addBounds(world: World) {
        addBox(world, x = WORLD_WIDTH / 2f, y = WALL_THICKNESS, hx = WORLD_WIDTH / 2f - 0.35f, hy = WALL_THICKNESS, type = BodyType.STATIC, restitution = 0.62f, tag = TAG_BOUNDARY)
        addBox(world, x = WALL_THICKNESS, y = WORLD_HEIGHT / 2f, hx = WALL_THICKNESS, hy = WORLD_HEIGHT / 2f - 0.35f, type = BodyType.STATIC, restitution = 0.62f, tag = TAG_BOUNDARY)
        addBox(world, x = WORLD_WIDTH - WALL_THICKNESS, y = WORLD_HEIGHT / 2f, hx = WALL_THICKNESS, hy = WORLD_HEIGHT / 2f - 0.35f, type = BodyType.STATIC, restitution = 0.62f, tag = TAG_BOUNDARY)
    }

    private fun addWallObstacle(world: World) {
        addBox(
            world = world,
            x = WORLD_WIDTH - WALL_THICKNESS - 0.95f,
            y = 7.4f,
            hx = 0.95f,
            hy = 0.12f,
            type = BodyType.STATIC,
            restitution = 0.48f,
            angle = -0.18f,
            tag = TAG_OBSTACLE,
        )
    }


    private fun addCenterObstacles(world: World) {
        addBox(
            world = world,
            x = 4.0f,
            y = 5.6f,
            hx = 0.95f,
            hy = 0.12f,
            type = BodyType.STATIC,
            restitution = 0.46f,
            angle = 0.28f,
            tag = TAG_OBSTACLE,
        )
        addBox(
            world = world,
            x = 6.2f,
            y = 8.6f,
            hx = 0.18f,
            hy = 1.25f,
            type = BodyType.STATIC,
            restitution = 0.35f,
            angle = 0.06f,
            tag = TAG_OBSTACLE,
        )
        addBox(
            world = world,
            x = 8.6f,
            y = 4.4f,
            hx = 1.1f,
            hy = 0.12f,
            type = BodyType.STATIC,
            restitution = 0.5f,
            angle = -0.34f,
            tag = TAG_OBSTACLE,
        )
    }

    private fun addBalls(world: World) {
        repeat(5) { index ->
            addCircle(
                world = world,
                x = 2.4f + index * 1.05f,
                y = 13.4f + index * 0.82f,
                radius = 0.36f,
                restitution = 0.72f,
                tag = TAG_BALL,
            )
        }
    }

    private fun addBoxes(world: World) {
        repeat(5) { index ->
            addBox(
                world = world,
                x = 7.5f + (index % 2) * 1.0f,
                y = 12.6f + index * 0.95f,
                hx = 0.4f,
                hy = 0.4f,
                type = BodyType.DYNAMIC,
                restitution = 0.42f,
                angle = if (index % 2 == 0) 0.12f else -0.08f,
                tag = TAG_BOX,
            )
        }
    }

    private fun addCircle(
        world: World,
        x: Float,
        y: Float,
        radius: Float,
        restitution: Float,
        tag: String,
    ) {
        val body = requireNotNull(world.createBody(BodyDef().apply {
            type = BodyType.DYNAMIC
            position = Vec2(x, y)
        }))
        body.m_userData = tag
        body.createFixture(FixtureDef().apply {
            shape = CircleShape().apply {
                m_p.setZero()
                m_radius = radius
            }
            density = 1f
            friction = 0.2f
            this.restitution = restitution
        })
    }

    private fun addBox(
        world: World,
        x: Float,
        y: Float,
        hx: Float,
        hy: Float,
        type: BodyType,
        restitution: Float,
        angle: Float = 0f,
        tag: String,
    ) {
        val body = requireNotNull(world.createBody(BodyDef().apply {
            this.type = type
            position = Vec2(x, y)
            this.angle = angle
        }))
        body.m_userData = tag
        body.createFixture(FixtureDef().apply {
            shape = PolygonShape().apply { setAsBox(hx, hy) }
            density = if (type == BodyType.DYNAMIC) 1f else 0f
            friction = 0.35f
            this.restitution = restitution
        })
    }
}

@Composable
fun SimpleDropDemoScreen() {
    var paused by remember { mutableStateOf(false) }
    var sceneVersion by remember { mutableIntStateOf(0) }
    var frame by remember { mutableIntStateOf(0) }
    var spawnSeed by remember { mutableIntStateOf(0) }
    var spawnMode by remember { mutableStateOf(SpawnMode.RANDOM) }
    val scene = remember(sceneVersion) { SimpleSceneFactory.create() }

    LaunchedEffect(scene, paused) {
        while (true) {
            withFrameNanos {
                if (!paused) {
                    scene.world.step(1f / 60f, 8, 3)
                    frame++
                }
            }
        }
    }

    MaterialTheme {
        Surface(
            modifier = Modifier.fillMaxSize(),
            color = Color(0xFFFFF8F8),
        ) {
            Column(
                modifier = Modifier
                    .safeContentPadding()
                    .fillMaxSize()
                    .padding(16.dp),
                verticalArrangement = Arrangement.spacedBy(12.dp),
            ) {
                Surface(
                    modifier = Modifier.fillMaxWidth(),
                    color = Color(0xFFFFEEF3),
                    shape = RoundedCornerShape(20.dp),
                ) {
                    Column(
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(horizontal = 16.dp, vertical = 14.dp),
                        verticalArrangement = Arrangement.spacedBy(6.dp),
                    ) {
                        Text(
                            text = "KBox2D 简单掉落示例",
                            style = MaterialTheme.typography.headlineSmall,
                            color = Color(0xFF2E1F26),
                        )
                        Text(
                            text = "矩形框内：球体 + 矩形掉落；点击区域内可随机生成新形状。",
                            style = MaterialTheme.typography.bodyMedium,
                            color = Color(0xFF7A5C66),
                        )
                        Text(
                            text = "Bodies=${scene.world.getBodyCount()}  Frame=$frame",
                            style = MaterialTheme.typography.bodySmall,
                            color = Color(0xFF9B7B86),
                        )
                    }
                }
                Surface(
                    modifier = Modifier.fillMaxWidth(),
                    color = Color(0xFFFFF6F8),
                    shape = RoundedCornerShape(18.dp),
                ) {
                    Column(
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(horizontal = 12.dp, vertical = 10.dp),
                        verticalArrangement = Arrangement.spacedBy(8.dp),
                    ) {
                        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                            Button(onClick = { paused = !paused }) {
                                Text(if (paused) "继续" else "暂停")
                            }
                            Button(onClick = {
                                sceneVersion++
                                frame = 0
                            }) {
                                Text("重置")
                            }
                            Button(onClick = {
                                SimpleSceneFactory.clearDynamicBodies(scene.world)
                                frame = 0
                            }) {
                                Text("清空掉落物")
                            }
                        }
                        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                            Button(onClick = { spawnMode = SpawnMode.RANDOM }) { Text("随机模式") }
                            Button(onClick = { spawnMode = SpawnMode.BALL }) { Text("只生成球") }
                            Button(onClick = { spawnMode = SpawnMode.BOX }) { Text("只生成方块") }
                        }
                    }
                }
                Box(
                    modifier = Modifier
                        .fillMaxWidth()
                        .weight(1f)
                        .clip(RoundedCornerShape(24.dp))
                        .background(Color(0xFFFFF1F4))
                        .pointerInput(scene, spawnSeed) {
                            detectTapGestures { offset ->
                                val viewport = SimpleViewport(scene.worldWidth, scene.worldHeight, Size(size.width.toFloat(), size.height.toFloat()))
                                val worldPoint = viewport.toWorld(offset)
                                spawnSeed++
                                when (spawnMode) {
                                    SpawnMode.RANDOM -> SimpleSceneFactory.spawnRandomBody(
                                        world = scene.world,
                                        worldWidth = scene.worldWidth,
                                        worldHeight = scene.worldHeight,
                                        x = worldPoint.x,
                                        y = worldPoint.y,
                                        seed = spawnSeed,
                                    )
                                    SpawnMode.BALL -> SimpleSceneFactory.spawnBall(
                                        world = scene.world,
                                        worldWidth = scene.worldWidth,
                                        worldHeight = scene.worldHeight,
                                        x = worldPoint.x,
                                        y = worldPoint.y,
                                        seed = spawnSeed,
                                    )
                                    SpawnMode.BOX -> SimpleSceneFactory.spawnBox(
                                        world = scene.world,
                                        worldWidth = scene.worldWidth,
                                        worldHeight = scene.worldHeight,
                                        x = worldPoint.x,
                                        y = worldPoint.y,
                                        seed = spawnSeed,
                                    )
                                }
                                frame++
                            }
                        },
                ) {
                    val renderFrame = frame
                    Canvas(modifier = Modifier.fillMaxSize()) {
                        if (renderFrame < 0) return@Canvas
                        val viewport = SimpleViewport(scene.worldWidth, scene.worldHeight, size)
                        drawRect(Color(0xFFFFF1F4))
                        drawBorder(viewport, scene.worldWidth, scene.worldHeight)
                        drawWorldBodies(scene.world, viewport)
                    }
                    Text(
                        text = "点击框内投放形状；障碍物已单独着色，支持清空动态物体。",
                        color = Color(0xFF9B7B86),
                        style = MaterialTheme.typography.bodySmall,
                        modifier = Modifier
                            .align(Alignment.BottomStart)
                            .padding(12.dp),
                    )
                }
            }
        }
    }
}

private data class SimpleViewport(
    val worldWidth: Float,
    val worldHeight: Float,
    val canvasSize: Size,
) {
    val scale: Float = min(canvasSize.width / worldWidth, canvasSize.height / worldHeight) * 0.92f
    private val offsetX: Float = (canvasSize.width - worldWidth * scale) / 2f
    private val offsetY: Float = (canvasSize.height - worldHeight * scale) / 2f

    fun toScreen(point: Vec2): Offset = Offset(
        x = offsetX + point.x * scale,
        y = canvasSize.height - offsetY - point.y * scale,
    )

    fun toWorld(offset: Offset): Vec2 = Vec2(
        ((offset.x - offsetX) / scale).coerceIn(0f, worldWidth),
        ((canvasSize.height - offsetY - offset.y) / scale).coerceIn(0f, worldHeight),
    )
}

private fun androidx.compose.ui.graphics.drawscope.DrawScope.drawBorder(
    viewport: SimpleViewport,
    worldWidth: Float,
    worldHeight: Float,
) {
    val topLeft = viewport.toScreen(Vec2(0.18f, worldHeight - 0.18f))
    val bottomRight = viewport.toScreen(Vec2(worldWidth - 0.18f, 0.18f))
    drawRect(
        color = Color(0xFFFF6B81),
        topLeft = Offset(topLeft.x, topLeft.y),
        size = Size(bottomRight.x - topLeft.x, bottomRight.y - topLeft.y),
        style = Stroke(width = 1.4f),
    )
}

private fun androidx.compose.ui.graphics.drawscope.DrawScope.drawWorldBodies(
    world: World,
    viewport: SimpleViewport,
) {
    var body = world.getBodyList()
    while (body != null) {
        var fixture = body.getFixtureList()
        while (fixture != null) {
            if ((body.m_userData as? String) != "boundary") {
                when (val shape = fixture.getShape()) {
                    is CircleShape -> drawCircleBody(body, shape, viewport)
                    is PolygonShape -> drawPolygonBody(body, shape, viewport)
                }
            }
            fixture = fixture.getNext()
        }
        body = body.getNext()
    }
}

private fun androidx.compose.ui.graphics.drawscope.DrawScope.drawCircleBody(
    body: Body,
    shape: CircleShape,
    viewport: SimpleViewport,
) {
    val center = bodyLocalToWorld(body, shape.m_p)
    val screen = viewport.toScreen(center)
    val color = when (body.m_userData as? String) {
        "obstacle" -> Color(0xFFFFA1B5)
        "ball" -> Color(0xFFFF5C7A)
        else -> if (body.getType() == BodyType.STATIC) Color(0xFFD8A1AF) else Color(0xFFFF7A95)
    }
    drawCircle(color = color, radius = shape.m_radius * viewport.scale, center = screen)
    val axis = bodyLocalToWorld(body, Vec2(shape.m_p.x + shape.m_radius, shape.m_p.y))
    drawLine(
        color = Color(0xFFFFF8F8).copy(alpha = 0.88f),
        start = screen,
        end = viewport.toScreen(axis),
        strokeWidth = 1.6f,
    )
}

private fun androidx.compose.ui.graphics.drawscope.DrawScope.drawPolygonBody(
    body: Body,
    shape: PolygonShape,
    viewport: SimpleViewport,
) {
    val path = Path()
    for (index in 0 until shape.m_count) {
        val point = viewport.toScreen(bodyLocalToWorld(body, shape.m_vertices[index]))
        if (index == 0) path.moveTo(point.x, point.y) else path.lineTo(point.x, point.y)
    }
    path.close()
    val color = when (body.m_userData as? String) {
        "obstacle" -> Color(0xFFFFBCCD)
        "box" -> Color(0xFFFF8FA3)
        else -> if (body.getType() == BodyType.STATIC) Color(0xFFE2B7C1) else Color(0xFFFF9FB1)
    }
    drawPath(path = path, color = color, style = Fill)
    drawPath(path = path, color = Color(0xFFFFFFFF).copy(alpha = 0.22f), style = Stroke(width = 1.2f))
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

@Preview
@Composable
private fun SimpleDropDemoPreview() {
    SimpleDropDemoScreen()
}
