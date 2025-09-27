import java.time.Instant
import java.time.ZoneId
import java.time.format.DateTimeFormatter

plugins {
	`java-library`
	`maven-publish`
	signing
	id("com.github.ben-manes.versions") version "0.53.0"
}

repositories {
	mavenCentral()
	maven("https://central.sonatype.com/repository/maven-snapshots/")
}

group = "io.calimero"
version = "3.0-SNAPSHOT"

java {
	toolchain {
		languageVersion.set(JavaLanguageVersion.of(21))
	}

	withSourcesJar()
	withJavadocJar()
}

tasks.withType<JavaCompile>().configureEach {
	options.encoding = "UTF-8"
	options.compilerArgs.addAll(listOf(
		"-Xlint:all,-serial",
		"--limit-modules", "java.base,io.calimero.core"
	))
}

tasks.named<JavaCompile>("compileJava") {
	options.javaModuleVersion = version.toString()
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
	from("${projectDir}/bin") {
		include("linux*/**", "win*/**", "mac*/**")
	}

	manifest {
		val gitHash = providers.exec {
			commandLine("git", "-C", "$projectDir", "rev-parse", "--verify", "--short", "HEAD")
		}.standardOutput.asText.map { it.trim() }
		val buildDate = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss z")
			.withZone(ZoneId.of("UTC"))
			.format(Instant.now())

		attributes(
			"Implementation-Version" to project.version,
			"Revision" to gitHash.get(),
			"Build-Date" to buildDate
		)
	}
}

tasks.named<Jar>("sourcesJar") {
	from("$projectDir") {
		include("README.md")
		include("LICENSE")
		include("LICENSE.LESSER")
	}
	from("${projectDir}/native-lib/src") {
		include("include/**", "unix/**", "win/**")
	}
}

dependencies {
	// both cpp-library and java-library plugin use api & implementation configurations,
	// hence cpp-library tries to resolve java dependencies, workaround with configurations not known by cpp
    api("io.calimero:calimero-core:${version}")
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
			val releasesRepoUrl = uri("https://ossrh-staging-api.central.sonatype.com/service/local/staging/deploy/maven2/")
			val snapshotsRepoUrl = uri("https://central.sonatype.com/repository/maven-snapshots/")
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
