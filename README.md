serial-native
=============

Libraries for serial port access using JNI (written in C).

Usage
=====
Compile the sources, or use one of the precompiled libraries in the 
`zip` or `tar.gz` archives available for Windows and Linux.
Copy the compiled library to a folder listed by `java.library.path` to be found by the JRE.

Maven
---
The `nar` maven plugin supports compiling native code on a number of different architectures. Currently, the POM contains settings for Linux and OS X.
To skip running any nar plugins, use `-Dnar.skip=true`.