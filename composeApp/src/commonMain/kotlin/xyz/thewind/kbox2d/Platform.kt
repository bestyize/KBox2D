package xyz.thewind.kbox2d

interface Platform {
    val name: String
}

expect fun getPlatform(): Platform