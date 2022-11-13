JNI C libraries for serial communication
=====

Linux, macOS, and Windows libraries for serial port access using JNI (written in C).

Compile the sources for your target platform, or use one of the precompiled libraries in the 
`zip` or `tar.gz` archives available for Windows and Linux. Compiling the sources requires an installed C/C++ toolchain.

### Build with Gradle 
For Linux, macOS, Windows; ARM(v7) is not working. The compiled library can be found in the _build/lib_ directory.

    ./gradlew build

### Build with Maven
For Linux, macOS, Windows, ARMv7. The compiled library can be found in the _target/nar_ directory.

    mvn nar:nar-compile

The `nar` maven plugin supports compiling native code on a number of different architectures. Currently, the POM contains settings for Linux and macOS, which need to be adjusted depending on Java version and directory. To skip running any nar plugins, use `-Dnar.skip=true`.

### Copy library to Java library path

Copy the compiled library to a folder of the Java library path, to be found by the JRE.
In Java, this prints the Java library path:

    System.out.println("Java library path = " + System.getProperty("java.library.path"));

