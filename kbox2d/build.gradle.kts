import org.jetbrains.kotlin.gradle.dsl.JvmTarget

plugins {
    alias(libs.plugins.kotlinMultiplatform)
    alias(libs.plugins.androidLibrary)
    alias(libs.plugins.mavenPublish)
}

// Tags pass -PreleaseVersion=X.Y.Z. Local builds use an unpublished snapshot.
group = "io.github.bestyize"
version = providers.gradleProperty("releaseVersion").getOrElse("0.1.0-SNAPSHOT")

kotlin {
    androidTarget {
        compilerOptions {
            jvmTarget.set(JvmTarget.JVM_11)
        }
    }

    jvm()
    iosArm64()
    iosSimulatorArm64()

    sourceSets {
        commonTest.dependencies {
            implementation(libs.kotlin.test)
        }
    }
}

android {
    namespace = "xyz.thewind.kbox2d.core"
    compileSdk = libs.versions.android.compileSdk.get().toInt()

    defaultConfig {
        minSdk = libs.versions.android.minSdk.get().toInt()
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
}


// Only the reusable KMP engine is published; composeApp is a demonstration app.
mavenPublishing {
    publishToMavenCentral()

    // Local publishToMavenLocal / PR checks do not need access to signing secrets.
    // Release CI injects signingInMemoryKey and always signs Maven Central artifacts.
    if (providers.gradleProperty("signingInMemoryKey").isPresent) {
        signAllPublications()
    }

    coordinates(group.toString(), "kbox2d", version.toString())

    pom {
        name.set("KBox2D")
        description.set("Experimental cross-platform 2D physics engine for Kotlin Multiplatform, adapted from JBox2D / Box2D.")
        inceptionYear.set("2026")
        url.set("https://github.com/bestyize/KBox2D")

        licenses {
            license {
                name.set("MIT License (original KBox2D contributions)")
                url.set("https://opensource.org/licenses/MIT")
                distribution.set("repo")
            }
            license {
                name.set("BSD 3-Clause License (adapted JBox2D portions)")
                url.set("https://opensource.org/licenses/BSD-3-Clause")
                distribution.set("repo")
            }
        }

        developers {
            developer {
                id.set("bestyize")
                name.set("bestyize")
                url.set("https://github.com/bestyize")
            }
        }

        scm {
            url.set("https://github.com/bestyize/KBox2D")
            connection.set("scm:git:https://github.com/bestyize/KBox2D.git")
            developerConnection.set("scm:git:ssh://git@github.com/bestyize/KBox2D.git")
        }
    }
}
