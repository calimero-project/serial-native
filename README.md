Calimero serial communication provider [![CI with Gradle](https://github.com/calimero-project/serial-native/actions/workflows/gradle.yml/badge.svg)](https://github.com/calimero-project/serial-native/actions/workflows/gradle.yml) [![](https://jitpack.io/v/calimero-project/serial-native.svg)](https://jitpack.io/#calimero-project/serial-native) [![](https://img.shields.io/badge/jitpack-master-brightgreen?label=JitPack)](https://jitpack.io/#calimero-project/serial-native/master)
=====

Provider for serial access to KNX networks using JNI. It contains Linux, macOS, and Windows libraries written in C.

Compile the C sources for your target platform, or use one of the precompiled libraries in the 
`jar` archive available for Windows, Linux, and macOS. Compiling the C sources requires an installed C/C++ toolchain.

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
For Linux, macOS, Windows. The compiled library can be found in the _build/lib_ directory.

    ./gradlew build

#### With Maven
The Java library is compiled via 

    mvn clean compile

The native library is compiled using the `NAR` maven plugin (the plugin configuration assumes that the _JAVA_HOME_ environment variable is set).
The POM contains settings for Linux and macOS:

    mvn nar:nar-compile

The compiled library can be found in the _target/nar_ directory.
The `NAR` maven plugin supports compiling native code on a number of different architectures.
