import org.gradle.internal.os.OperatingSystem
import org.gradle.nativeplatform.toolchain.internal.NativeToolChainRegistryInternal

plugins {
	`cpp-library`
	`visual-studio`
	id("com.github.ben-manes.versions") version "0.54.0"
}

group = "io.calimero"
version = "3.0-SNAPSHOT"

// set this variable manually to Java home directory if default location does not work
val javaHome = providers.systemProperty("java.home")
val os = OperatingSystem.current()!!
val cppStd = if (os.isWindows) "/std:c++17" else "-std=c++17"

// Supported UNIX locking modes in addition to TIOCEXCL:
// USE_UUCP_LOCKING: use UUCP-style lock  -> /var/lock/
// USE_FCNTL_LOCKING: use UNIX record lock -> fcntl(F_SETLK)
val unixLockingModes: List<String> = (project.findProperty("unixLockingModes") as String?) ?.split(" ") ?.filter { it.isNotBlank() }
	?: listOf("-DUSE_UUCP_LOCKING", "-DUSE_FCNTL_LOCKING")

tasks.named("assemble") {
	val jdk = javaHome.get()
	val os = os
	doFirst {
		logger.info("Platform: $os")
		logger.info("Java home directory: $jdk")
	}
}

toolChains {
	if (os.isWindows) {
		// Windows: support compilation using the VS Build Tools which aren't found by Gradle
		(this as NativeToolChainRegistryInternal).registerDefaultToolChain("visualCppBuildTools", VisualCpp::class.java)
		this.withType<VisualCpp>().configureEach {
			if (name == "visualCppBuildTools") {
				val vsToolsInstallDir = "C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools"
				logger.info("Using VS Build Tools directory $vsToolsInstallDir")
				setInstallDir(vsToolsInstallDir)
			}
		}
	}
	if (os.isLinux) {
		// ARMv7 support on Linux (e.g., Raspberry)
		(this as NativeToolChainRegistryInternal).registerDefaultToolChain("gccArmv7", Gcc::class.java)
		this.withType<Gcc>().configureEach {
			if (name == "gccArmv7") target("host:arm-v7")
		}
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
		compileTask.get().apply {
			isPositionIndependentCode = true
			isDebuggable = !isOptimized
			compilerArgs = if (isOptimized) {
				when (toolChain.get()) {
					is Gcc -> listOf(cppStd, "-Wall", "-Os", "-fno-exceptions", "-fno-unwind-tables", "-fno-asynchronous-unwind-tables")
					is Clang -> listOf(cppStd, "-Wall", "-Oz", "-fno-cxx-exceptions", "-fno-unwind-tables")
					is VisualCpp -> listOf(cppStd, "/Os", "/Wall", "/GS-")
					else -> listOf()
				}
			} else { // debug
				when (toolChain.get()) {
					// maybe -O0 works better, -Og still does several optimizations
					// option "-g" is included by default
					is Gcc, is Clang -> listOf(cppStd, "-DDEBUG", "-Wall", "-Og")
					is VisualCpp -> listOf(cppStd, "/DDEBUG", "/Wall") // '/Zi' already included
					else -> listOf()
				}
			}
			if (toolChain.get() !is VisualCpp)
				compilerArgs.addAll(unixLockingModes)
		}
		linkTask.get().apply {
			debuggable = !isOptimized
			linkerArgs = toolChain.map { tc ->
				if (tc is Gcc && isOptimized) listOf("-Wl,-z,max-page-size=0x1000", "-Wl,-s", "-static-libgcc")
				else if (tc is VisualCpp && isOptimized) listOf("/NODEFAULTLIB", "/NOENTRY", "kernel32.lib")
				else if (tc is Clang && isOptimized) listOf("-install_name", "@rpath/lib${baseName.get()}.dylib")
				else listOf()
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

// always build debug/release(+stripped) versions
tasks.named("build") {
	dependsOn(tasks.matching { it.name.startsWith("assemble") })
	if (!os.isWindows)
		dependsOn(tasks.named("stripSymbolsRelease"))
}
