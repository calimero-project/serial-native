Calimero serial communication provider [![CI with Gradle](https://github.com/calimero-project/serial-native/actions/workflows/gradle.yml/badge.svg)](https://github.com/calimero-project/serial-native/actions/workflows/gradle.yml)
=====

Provider for serial access to KNX networks using JNI. It contains Linux, macOS, and Windows libraries written in C.

Compile the C sources for your target platform, or use one of the precompiled libraries in the 
`jar` archive available for Windows and Linux. Compiling the C sources requires an installed C/C++ toolchain.

This provider uses `System.Logger` for logging.

### Setup

* Put the `.jar` file on the module or class path used by Calimero.

* Copy the (pre-)compiled C library to a folder of the Java library path, to be found by the JRE.
In Java, this prints the Java library path:

        System.out.println("Java library path = " + System.getProperty("java.library.path"));


### Building from source 
~~~ sh
git clone https://github.com/calimero-project/serial-native.git
~~~

#### With Gradle 
For Linux, macOS, Windows; ARM(v7) is not working. The compiled library can be found in the _build/lib_ directory.

    ./gradlew build

#### With Maven
For Linux, macOS, Windows, ARMv7, Aarch64. The compiled library can be found in the _target/nar_ directory.

    mvn nar:nar-compile

The `nar` maven plugin supports compiling native code on a number of different architectures. Currently, the POM contains settings for Linux and macOS, which need to be adjusted depending on Java version and directory. To skip running any nar plugins, use `-Dnar.skip=true`.

