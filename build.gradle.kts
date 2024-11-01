import org.gradle.internal.os.OperatingSystem

plugins {
	`cpp-library`
	`java-library`
	`maven-publish`
	signing
	id("com.github.ben-manes.versions") version "0.51.0"
	`visual-studio`
}

repositories {
	mavenCentral()
	maven("https://oss.sonatype.org/content/repositories/snapshots")
}

group = "io.calimero"
version = "2.6-rc1"

// set this variable manually to Java home directory if default location does not work
val javaHome = providers.systemProperty("java.home")
val cppStd = "-std=c++17"
val os = OperatingSystem.current()

java {
	toolchain {
		languageVersion.set(JavaLanguageVersion.of(17))
	}

	withSourcesJar()
	withJavadocJar()
}

tasks.withType<JavaCompile>().configureEach {
	options.encoding = "UTF-8"
	options.compilerArgs.addAll(listOf(
		"-Xlint:all,-serial",
		"--limit-modules", "java.base"
	))
}

tasks.named<JavaCompile>("compileTestJava") {
	options.compilerArgs.add("-Xlint:-try")
}

tasks.withType<Javadoc>().configureEach {
	options.encoding = "UTF-8"
	(options as CoreJavadocOptions).addStringOption("Xdoclint:-missing", "-quiet")
}

tasks.named<Jar>("jar") {
	from("$projectDir") {
		include("LICENSE")
		include("LICENSE.LESSER")
		into("META-INF")
	}
}

tasks.named<Jar>("sourcesJar") {
	from("$projectDir") {
		include("README.md")
	}
	from("${projectDir}/src") {
		include("include/**", "unix/**", "win/**")
	}
}

tasks.named<Jar>("jar") {
	from("${projectDir}/bin") {
		include("linux*/**", "win*/**")
	}
}

tasks.named("assemble") {
	val jdk = javaHome.get()
	val os = os
	doFirst {
		logger.info("Platform: $os")
		logger.info("Java home directory: $jdk")
	}
}

library {
	source {
		if (os.isWindows) {
			from(fileTree("src/win") { include("**/*.cpp") })
		} else { // linux, macos
			from(fileTree("src/unix") { include("**/*.cpp") })
		}
	}

	privateHeaders {
		val jdk = javaHome.get()
		from("src/include", "$jdk/include", "$jdk/../include")
		val osSpecificInclude = when {
			os.isLinux   -> "linux"
			os.isMacOsX  -> "darwin"
			os.isWindows -> "win32"
			else         -> ""
		}
		from("$jdk/include/$osSpecificInclude", "$jdk/../include/$osSpecificInclude")
	}

	baseName = "serialcom"
	linkage = listOf(Linkage.SHARED)

	binaries.configureEach(CppSharedLibrary::class.java) {
		val compileTask = compileTask.get()
		compileTask.isPositionIndependentCode = true
		compileTask.isDebuggable = !compileTask.isOptimized

		when (toolChain) {
			is Clang, is Gcc -> {
				if (compileTask.isOptimized) {
					// "-Wl,-s" not necessary
					compileTask.compilerArgs =
						if (toolChain is Clang) listOf(cppStd, "-Wall", "-Oz", "-fno-cxx-exceptions")
						else listOf(cppStd, "-Wall", "-Os", "-fno-exceptions")
				} else {
					// maybe -O0 works better, -Og still does several optimizations
					// option "-g" is included by default
					compileTask.compilerArgs = listOf(cppStd, "-DDEBUG", "-Wall", "-Og")
				}
			}
			is VisualCpp -> {
				if (compileTask.isOptimized) {
					compileTask.compilerArgs = listOf("/Os", "/Wall")
				} else {
					// '/Zi' already included
					compileTask.compilerArgs = listOf("/DDEBUG", "/Wall")
					val linkTask = linkTask.get()
					linkTask.linkerArgs = listOf("/DEBUG")
				}
			}
		}
	}
}

tasks.withType<CppCompile>().configureEach {
	doFirst {
		val compile = if (isOptimized) "release" else "debug"
		logger.info("${toolChain.get()} additional compiler args for $compile: ${compilerArgs.get()}")
	}
}

if (!os.isWindows) {
	// always build debug/release(+stripped) versions
	tasks.named("build") {
		dependsOn(tasks.named("stripSymbolsRelease"))
	}
}

dependencies {
	// both cpp-library and java-library plugin use api & implementation configurations,
	// hence cpp-library tries to resolve java dependencies, workaround with configurations not known by cpp
//    api "com.github:calimero-core:${version}"
	compileOnly("com.github.calimero:calimero-core:$version")
	runtimeOnly("com.github.calimero:calimero-core:$version")
}

// Don't publish the cpp artifacts
tasks.named("generateMetadataFileForMainPublication") {
	enabled = false
}
tasks.named("publishMainPublicationToMavenLocal") {
	enabled = false
}

publishing {
	publications {
		create<MavenPublication>("mavenJava") {
			artifactId = rootProject.name
			from(components["java"])
			pom {
				name.set("Calimero serial-native service provider")
				description.set("Serial communication provider using JNI")
				url.set("https://github.com/calimero-project/serial-native")
				inceptionYear.set("2006")
				licenses {
					license {
						name.set("GNU General Public License, version 2, with the Classpath Exception")
						url.set("LICENSE")
					}
					license {
						name.set("GNU Lesser General Public License, version 2.1")
						url.set("LICENSE.LESSER")
					}
				}
				developers {
					developer {
						name.set("Boris Malinowsky")
						email.set("b.malinowsky@gmail.com")
					}
				}
				scm {
					connection.set("scm:git:git://github.com/calimero-project/serial-native.git")
					url.set("https://github.com/calimero-project/serial-native.git")
				}
			}
		}
	}
	repositories {
		maven {
			name = "maven"
			val releasesRepoUrl = uri("https://s01.oss.sonatype.org/service/local/staging/deploy/maven2")
			val snapshotsRepoUrl = uri("https://s01.oss.sonatype.org/content/repositories/snapshots")
			url = if (version.toString().endsWith("SNAPSHOT")) snapshotsRepoUrl else releasesRepoUrl
			credentials(PasswordCredentials::class)
		}
	}
}

signing {
	if (project.hasProperty("signing.keyId")) {
		sign(publishing.publications["mavenJava"])
	}
}
