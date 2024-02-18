plugins {
    id("com.android.library") version "8.2.2"
    id("com.github.ben-manes.versions") version "0.51.0"
}

android {
    namespace = "io.calimero.serial.provider.jni"
    compileSdk = 34
    defaultConfig {
        minSdk = 31
        testOptions.targetSdk = 34
        lint.targetSdk = 34
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"

        externalNativeBuild {
            cmake {
                cppFlags += "-DDEBUG"
            }
        }
    }

    buildTypes {
        release {
            isMinifyEnabled = true
        }
        debug {
            isJniDebuggable = true
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    externalNativeBuild {
        cmake {
            path("src/unix/CMakeLists.txt")
            version = "3.22.1"
        }
    }

    flavorDimensions += listOf("package")
    productFlavors {
        create("demo") {
            dimension = "package"
            namespace = "io.calimero.serial.demo"
        }
    }

    buildFeatures {
        viewBinding = true
    }
    buildToolsVersion = "35.0.0 rc1"
    ndkVersion = "26.2.11394342"
}

allprojects {
    gradle.projectsEvaluated {
        tasks.withType<JavaCompile> {
            options.compilerArgs.addAll(listOf("--add-reads", "io.calimero.serial.provider.jni=ALL-UNNAMED"))
        }
    }
}

dependencies {
    api("io.calimero:calimero-core:3.0-SNAPSHOT")

    implementation("androidx.appcompat:appcompat:1.6.1")
    implementation("com.google.android.material:material:1.11.0")
    implementation("androidx.constraintlayout:constraintlayout:2.1.4")

    androidTestImplementation("androidx.test.ext:junit:1.1.5")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.5.1")
}
