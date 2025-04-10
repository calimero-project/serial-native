/*
    Calimero 3 - A library for KNX network access
    Copyright (c) 2011, 2024 B. Malinowsky

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

//  This file is part of Calimero 3, a library for KNX network access.
//  It provides Java Native Interface (JNI) access to tty serial port I/O
//  communication devices.
//  The JNI is defined in serialcom.h.
//
// Implementation notes, for what it's worth:
// I do not take any precautions of any cua devices, they are not supported.
//
// Strings are treated as utf-8, where necessary.
// Data types used are targeted for 32/64 bit machines.

#if defined _DEBUG
#if !defined DEBUG
#define DEBUG
#endif
#endif

#define _POSIX_C_SOURCE 1

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>
#include <dirent.h>
#include <limits.h>

// next include is for select()
#include <sys/select.h>
#include <signal.h> // kill

#include <time.h>

#if defined __APPLE__

#if !defined TIOCMIWAIT
#define TIOCMIWAIT  0x545C
#endif

#else
#include <linux/serial.h>
#endif

// for debug trace calls
#if defined DEBUG

#include <stdio.h>
#include <stdarg.h>

#else
// non-debug stuff provides dummies to keep library small

extern "C" {
    static void perror(const char* /*str*/) {}
}

#if defined __APPLE__
static int puts(const char* /*str*/) { return 1; }
#else
extern "C" {
    static int puts(const char* /*str*/) { return 1; }
}
#endif // __APPLE__

#endif // DEBUG

#include "serialcom.h"

/*
 Typical serial port device names

 Linux: /dev/ttyS0 or /dev/ttyUSB0 or /dev/ttyACM0
 HP-UX: /dev/tty1p0
 Solaris: /dev/ttya
 MacOS: /dev/ttys0
*/

// define our own file descriptor type to distinguish from the default used type int
typedef int fd_t;

static const fd_t INVALID_FD = ((fd_t) -1);

#if defined DEBUG

static uint64_t timestamp_us() {
#ifdef __MACH__
	return 0;
#else
	struct timespec now;
	clock_gettime(CLOCK_REALTIME, &now);

	uint64_t us = now.tv_sec * 1000000 + now.tv_nsec / 1000;
	return us;
#endif
}

static uint64_t timestamp_diff(uint64_t start) {
	uint64_t now = timestamp_us();
	return now - start;
}

static const char* timestamp() {
	static char buf[50];
	buf[0] = 0;
#ifdef __MACH__
	return "0.000";
#else
	struct timespec now;
	clock_gettime(CLOCK_REALTIME, &now);
	sprintf(buf, "%ld.%03ld", now.tv_sec, now.tv_nsec / 1000000);
	return buf;
#endif
}

static void trace(const char* msg, const char* msg2 = 0)
{
    const char* opt = msg2 != 0 ? msg2 : "";
    const char* space = msg2 != 0 ? " " : "";

    printf("[%s] %s%s%s\n", timestamp(), msg, space, opt);
    fflush(stdout);
}

#if !defined __APPLE__
static void dbg_out(const char* format, ...)
{
    va_list argptr;
    va_start(argptr,format);

    vprintf(format, argptr);
    fflush(stdout);
}
#endif

static void trace_error(const char* msg, const char* msg2 = 0)
{
    int error = errno;
    if (error != 0) {
    	const uint32_t bufSize = 1024;
        char errmsg[bufSize];
        const char* opt = msg2 != 0 ? msg2 : "";
        const char* space = msg2 != 0 ? " " : "";
        const char* str = strerror(error);
        snprintf(errmsg, 1024, "%s%s%s: %s (%i)", msg, space, opt, str, error);
        trace(errmsg);
    }
}

#else
static void trace(const char* /*msg*/, const char* /*msg2*/= 0) {}
#if !defined __APPLE__
	static void dbg_out(const char* /*format*/, ...) {}
#endif
static void trace_error(const char* /*msg*/, const char* /*msg2*/= 0) {}
#endif

static void throwException(JNIEnv* env, int error)
{
    const size_t size = 256;
    char msg[size];
    msg[0] = 0;
    // use POSIX strerror (i.e., retrieve error string pointer and copy it over)
    // instead of XSI/GNU strerror_r functions
    // while strerror_r is thread-safe, it is not available on all systems
    //const char* str = strerror_r(error, msg, 100);
    const char* str = strerror(error);
    strncpy(msg, str, size - 1);
    jclass c = env->FindClass("java/io/IOException");
    env->ThrowNew(c, msg);
}

static fd_t getFD(JNIEnv* env, jobject obj)
{
    return static_cast<fd_t>(env->GetLongField(obj,
            env->GetFieldID(env->GetObjectClass(obj), "fd", "J")));
}

static void setFD(JNIEnv* env, jobject obj, fd_t fd)
{
    jclass cls = env->GetObjectClass(obj);
    jfieldID fid = env->GetFieldID(cls, "fd", "J");
    env->SetLongField(obj, fid, static_cast<jlong>(fd));
}

static const char* lockDir = "/var/lock/";
static const char* devPrefix = "/dev/";
static const char* lckPrefix = "LCK..";
static const char* pidPrefix = "PID..";

static const uint32_t MaxFileNameLength = 100;

static char lockedPort[MaxFileNameLength];

//
// some string helper
//

static size_t createLockName(char* out, const char* dir, const char* namePrefix, const char* name)
{
    strcpy(out, dir);
    strcat(out, namePrefix);
    strcat(out, name);
    return strlen(out);
}

// keeps us from pulling in s(n)printf for number conversions in release builds,
// which adds several KB for nothing
// returns str pointer
static char* itoa(int32_t i, char* str)
{
    if (i < 0) {
        *str++ = '-';
        i = -i;
    }

    char* t = str;
    do {
        *t++ = static_cast<char>(i % 10 + '0');
    } while ((i /= 10) > 0);
    *t = 0;
    for (--t; t > str; ++str, --t) {
        char c = *str;
        *str = *t;
        *t = c;
    }
    return str;
}

static bool tryLink(const char* forName, const char* linkName)
{
    if (link(forName, linkName) == -1) {
        trace_error("create link", linkName);
        // we are nevertheless successful if lock count states 2 locks
        struct stat buf;
        if (stat(forName, &buf) == -1 || buf.st_nlink != 2)
            return false;
    }
    unlink(forName);
    return true;
}

static pid_t readPid(const char* filename)
{
    pid_t pid = -1;
    fd_t fd = open(filename, O_RDONLY);
    if (fd != INVALID_FD) {
        char in[20] = { 0 };
        /*ssize_t rd = */read(fd, in, sizeof(in));
        close(fd);

        pid = static_cast<pid_t>(strtoul(in, 0, 10));
    }
    else
        trace_error("read pid from", filename);
    return pid;
}

// try to create PID..pid and write our pid into it
// create LCK..name as link to pid file
// all files are located in /var/lock/
static bool ensureLock(const char* port)
{
    trace("ensure lock in /var/lock/");
    const size_t size = 50;
    char name[size];
    const char* start = port;
    if (!strncmp(port, devPrefix, 5) && port[5])
        start = port + 5;
    strncpy(name, start, size - 1);

    char pidFile[MaxFileNameLength];
    char lckFile[MaxFileNameLength];
    char strPid[20];
    pid_t myPid = getpid();
    itoa(myPid, strPid);
    createLockName(pidFile, lockDir, pidPrefix, strPid);
    createLockName(lckFile, lockDir, lckPrefix, name);

    fd_t fd = open(pidFile, O_RDWR | O_EXCL | O_CREAT, 0644);
    if (fd == INVALID_FD) {
        trace_error("open", pidFile);
        return false;
    }

    /*ssize_t wr = */write(fd, strPid, strlen(strPid));
    close(fd);

    // additional locking check, because O_EXCL in open is only
    // supported when using NFSv3 or later on kernel 2.6 or later
    // (see man open(2) )
    if (tryLink(pidFile, lckFile))
        return true;
    bool locked = false;
    // lock file exists, read its pid
    // if we cannot read in the pid, we cannot lock
    pid_t pid = readPid(lckFile);
    if (pid > 0) {
        // we might already have locked the port
        if (pid == myPid)
            locked = true;
        // check the read pid, if stale, try to remove lock file
        else if (kill(pid, 0) == -1 && errno != EPERM) {
            unlink(lckFile);
            if (tryLink(pidFile, lckFile))
                return true;
        }
    }
    unlink(pidFile);
    return locked;
}

static bool releaseLock(const char* port)
{
    const char* name = strrchr(port, '/');
    name = name == 0 ? port : name + 1;

    char lockFile[MaxFileNameLength];
    createLockName(lockFile, lockDir, lckPrefix, name);
    errno = 0;
    trace("release lock", lockFile);

    pid_t pid = readPid(lockFile);
    if (pid != -1 && pid == getpid())
        unlink(lockFile);
    return true;
}

static bool closePort(fd_t fd)
{
    if (fd == INVALID_FD)
        return true;
    trace("close handle");
    bool closed = false;
    do {
        closed = close(fd) == 0;
        if (closed)
            break;
    } while (errno == EINTR);

    releaseLock(lockedPort);
    lockedPort[0] = 0;

    return closed;
}

static bool closePort(JNIEnv* env, jobject obj)
{
    fd_t fd = getFD(env, obj);
    setFD(env, obj, INVALID_FD);
    return closePort(fd);
}

static bool setPortDefaults(fd_t fd)
{
    struct termios options;

    if (tcgetattr(fd, &options) == -1) {
        trace_error("config port: tcgetattr");
        return false;
    }

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag |= CREAD | CLOCAL;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_iflag = INPCK;
    options.c_lflag = 0;
    options.c_oflag = 0;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &options) == -1) {
        trace_error("config port: tcsetattr");
        return false;
    }
    fcntl(fd, F_SETOWN, getpid());

    return true;
}

static fd_t openPort(JNIEnv* env, jstring portId, bool configurePort, int* lastError)
{
    jboolean iscopy;
    const char* port = env->GetStringUTFChars(portId, &iscopy);

    fd_t fd = INVALID_FD;
    errno = 0;
    int error = 0;

    trace("open", port);
    if (ensureLock(port)) {
        strncpy(lockedPort, port, MaxFileNameLength);
        lockedPort[MaxFileNameLength - 1] = 0;

        do {
            // we set the port exclusive below, not here
            fd = (fd_t) open(port, /*O_EXCL |*/O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd != INVALID_FD)
                break;
        } while (errno == EINTR);
    }
    else
        perror("obtaining port lock");

    // check if someone else has opened the port
    if (fd == INVALID_FD || errno == EBUSY) {
        error = errno;
        perror(port);
        releaseLock(port);
    }
    else {
        // we continue if we are not able to set exclusive mode
        if (ioctl(fd, TIOCEXCL) == -1) {
            error = errno;
            perror("set exclusive");
        }

        if (configurePort) {
            // we continue if we are not able to save old port settings
            struct termios saved;
            if (tcgetattr(fd, &saved) == -1) {
                error = errno;
                perror("save old port settings");
            }

            setPortDefaults(fd);
        }
    }

    if (lastError)
        *lastError = error;
    env->ReleaseStringUTFChars(portId, port);
    return fd;
}

#if !defined __APPLE__
static bool hasDevSerialLink(JNIEnv* env, jstring portId)
{
    const char* dir = "/dev/serial/by-id";
    trace("open dir", dir);
    struct dirent *dp;
    DIR *dfd;
    if ((dfd = opendir(dir)) == NULL) {
        trace_error("can't open", dir);
        return false;
    }

    jboolean iscopy;
    const char* port = env->GetStringUTFChars(portId, &iscopy);
    bool hasLink = false;
    char filename[PATH_MAX];
    while ((dp = readdir(dfd)) != NULL) {
        struct stat stbuf;
        sprintf(filename, "%s/%s", dir, dp->d_name);
        if (stat(filename, &stbuf) == -1) {
            trace_error("error in stat for", filename);
            continue;
        }
        // ignore entries '.' and '..'
        if (dp->d_name[0] == '.' && strlen(dp->d_name) <= 2)
            continue;

        trace("found entry", dp->d_name);
        char abs[PATH_MAX];
        if (realpath(filename, abs) == NULL)
            trace_error("error in realpath for", filename);
        else if (strcmp(port, abs) == 0) {
            hasLink = true;
            dbg_out("%s -> %s\n", dp->d_name, abs);
            break;
        }
    }

    env->ReleaseStringUTFChars(portId, port);
    return hasLink;
}
#endif // !defined __APPLE__

#if !defined PORT_UNKNOWN
#define PORT_UNKNOWN 0
#endif

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    portExists
 * Signature: (Ljava/lang/String;)Z
 */
JNIEXPORT jboolean JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_portExists(
        JNIEnv* env, jclass /*c*/, jstring portID)
{
    int error;
    // this will open the port, alternatives might be to use /dev/serial or /proc/tty
    fd_t fd = openPort(env, portID, false, &error);
    if (fd == INVALID_FD) {
        if (error == EBUSY || error == EPERM || error == EACCES)
            return JNI_TRUE;
        return JNI_FALSE;
    }
#if defined TIOCGSERIAL
    jboolean valid = JNI_FALSE;
    struct serial_struct info;
    errno = 0;
    if (ioctl(fd, TIOCGSERIAL, &info) == 0) {
        if (info.type != PORT_UNKNOWN)
            valid = JNI_TRUE;
        else if (hasDevSerialLink(env, portID))
            valid = JNI_TRUE;
    }
    else
        trace_error("check port type");
#else
    jboolean valid = JNI_FALSE;
#endif
    closePort(fd);
    return valid;
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    open
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_open(JNIEnv* env,
        jobject obj, jstring portID)
{
    int error;
    fd_t fd = openPort(env, portID, true, &error);
    if (fd == INVALID_FD)
        throwException(env, error);
    else {
        setFD(env, obj, fd);
#if defined DEBUG

#if !defined TXSETIHOG
#define     TXSETIHOG (('X'<<8) + 9)
#define     TXSETOHOG (('X'<<8) + 10)
#endif

        // set rx and tx queues
        const uint32_t size = 200;
        // XXX how to set buffers
        if (ioctl(fd, TXSETIHOG, &size) == -1)
            trace_error("TXSETIHOG");
        if (ioctl(fd, TXSETOHOG, &size) == -1)
            trace_error("TXSETOHOG");

        char out[50];
        snprintf(out, 50, "set queue tx %d rx %d", 0, 0);
        trace(out);
#endif // DEBUG
        int status;
        int ret = ioctl(fd, TIOCMGET, &status);
        if (ret != -1) {
            status |= TIOCM_DTR;
            ret = ioctl(fd, TIOCMSET, &status);
        }
        if (ret == -1) {
            int err = errno;
            closePort(env, obj);
            throwException(env, err);
        }
    }
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    close0
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_close0(JNIEnv* env,
        jobject obj)
{
    if (!closePort(env, obj))
        throwException(env, errno);
}

static const uint32_t BAUDRATE = io_calimero_serial_provider_jni_TtySerialCom_BAUDRATE;
static const uint8_t PARITY = io_calimero_serial_provider_jni_TtySerialCom_PARITY;
static const uint8_t DATABITS = io_calimero_serial_provider_jni_TtySerialCom_DATABITS;
static const uint8_t STOPBITS = io_calimero_serial_provider_jni_TtySerialCom_STOPBITS;
static const uint8_t FLOWCTRL = io_calimero_serial_provider_jni_TtySerialCom_FLOWCTRL;

#undef STOPBITS_10
#undef STOPBITS_15
#undef STOPBITS_20

static const uint8_t STOPBITS_10 = io_calimero_serial_provider_jni_TtySerialCom_ONE_STOPBIT;
static const uint8_t STOPBITS_15 = 3;
static const uint8_t STOPBITS_20 = io_calimero_serial_provider_jni_TtySerialCom_TWO_STOPBITS;

static const uint8_t FLOWCTRL_NONE = io_calimero_serial_provider_jni_TtySerialCom_FLOWCTRL_NONE;
static const uint8_t FLOWCTRL_CTSRTS = io_calimero_serial_provider_jni_TtySerialCom_FLOWCTRL_CTSRTS;

// CSTOPB set corresponds to 2 stop bits, default is 1 stop bit
static void setTermiosStopBits(tcflag_t* cflags, uint8_t stopbits)
{
    *cflags &= ~CSTOPB;
    switch (stopbits) {
    case STOPBITS_20:
        *cflags |= CSTOPB;
        break;
    case STOPBITS_15: /* does not exist */
        break;
    case STOPBITS_10:
        break;
    default:
        break;
    }
}

static uint8_t getGenericStopBits(tcflag_t cflags)
{
    int stopbits = cflags & CSTOPB;
    switch (stopbits) {
    // CSTOPB means 2 stop bits
    case CSTOPB:
        return STOPBITS_20;
    default:
        return STOPBITS_10;
    }
}

static const int PARITY_NONE = io_calimero_serial_provider_jni_TtySerialCom_PARITY_NONE;
static const int PARITY_ODD = io_calimero_serial_provider_jni_TtySerialCom_PARITY_ODD;
static const int PARITY_EVEN = io_calimero_serial_provider_jni_TtySerialCom_PARITY_EVEN;
static const int PARITY_MARK = io_calimero_serial_provider_jni_TtySerialCom_PARITY_MARK;
static const int PARITY_SPACE = -1; //io_calimero_serial_provider_jni_TtySerialCom_PARITY_SPACE;


static void setTermiosParity(tcflag_t* cflags, int32_t parity)
{
#ifdef  CMSPAR
    *cflags &= ~(PARENB | PARODD | CMSPAR);
#else
    *cflags &= ~(PARENB | PARODD);
#endif // CMSPAR

    switch (parity) {
    case PARITY_NONE:
        break;
    case PARITY_EVEN:
        *cflags |= PARENB;
        break;
    case PARITY_ODD:
        *cflags |= PARENB | PARODD;
        break;
    // mark and space parity have to be simulated
    case PARITY_SPACE:
        // for 7 data bits with space parity use 8 data bits with no parity
        if ((*cflags & CS7) == CS7) {
            trace("setting space parity for 7S1");
            *cflags &= ~(CSIZE | CSTOPB);
            *cflags |= CS8;
        }
#if defined CMSPAR
        else {
            trace("setting space parity for 8S1");
            // 8S1: 8 data bits with space parity
            *cflags |= CMSPAR | PARENB;
        }
#endif // CMSPAR
        break;
    case PARITY_MARK:
        if ((*cflags & CS7) == CS7) {
            trace("setting mark parity for 7M1");
            // for mark parity use 7 data bits, 2 stop bits
            *cflags &= ~CSIZE;
            *cflags |= CSTOPB | CS7;
        }
#if defined CMSPAR
        else {
            trace("setting mark parity for 8M1");
            // 8M1
            *cflags |= CMSPAR | PARENB | PARODD;
        }
#endif // CMSPAR
        break;
    default:
        break;
    }
}

static uint32_t getGenericParity(tcflag_t cflags)
{
#ifdef  CMSPAR
    if ((cflags & PARENB) && (cflags & PARODD) && (cflags & CMSPAR))
        return PARITY_MARK;
    //if ((cflags & PARENB) && (cflags & CMSPAR))
    //  return PARITY_SPACE;
#endif // CMSPAR

    if ((cflags & PARENB) && (cflags & PARODD))
        return PARITY_ODD;
    if (cflags & PARENB)
        return PARITY_EVEN;

    return PARITY_NONE;
}

static int32_t getTermiosBaudrate(uint32_t baudrate)
{
    switch (baudrate) {
    case 0:
        return B0;
    case 50:
        return B50;
    case 75:
        return B75;
    case 110:
        return B110;
    case 134:
        return B134;
    case 150:
        return B150;
    case 200:
        return B200;
    case 300:
        return B300;
    case 600:
        return B600;
    case 1200:
        return B1200;
    case 1800:
        return B1800;
    case 2400:
        return B2400;
    case 4800:
        return B4800;
    case 9600:
        return B9600;
#ifdef B14400
        case 14400:
        return B14400;
#endif /* B14400 */
    case 19200:
        return B19200;
#ifdef B28800
        case 28800:
        return B28800;
#endif /* B28800 */
    case 38400:
        return B38400;
#ifdef B57600 // MacOS X does not define this Baud rate
    case 57600:
        return B57600;
#endif // B57600
#ifdef B115200
    case 115200:
        return B115200;
#endif /*  B115200 */
#ifdef B230400
    case 230400:
        return B230400;
#endif /* B230400 */
#ifdef B460800
    case 460800:
        return B460800;
#endif /* B460800 */
#ifdef B500000
    case 500000:
        return B500000;
#endif /* B500000 */
#ifdef B576000
    case 576000:
        return B576000;
#endif /* B576000 */
#ifdef B921600
    case 921600:
        return B921600;
#endif /* B921600 */
#ifdef B1000000
    case 1000000:
        return B1000000;
#endif /* B1000000 */
#ifdef B1152000
    case 1152000:
        return B1152000;
#endif /* B1152000 */
#ifdef B1500000
    case 1500000:
        return B1500000;
#endif /* B1500000 */
#ifdef B2000000
    case 2000000:
        return B2000000;
#endif /* B2000000 */
#ifdef B2500000
    case 2500000:
        return B2500000;
#endif /* B2500000 */
#ifdef B3000000
    case 3000000:
        return B3000000;
#endif /* B3000000 */
#ifdef B3500000
    case 3500000:
        return B3500000;
#endif /* B3500000 */
#ifdef B4000000
    case 4000000:
        return B4000000;
#endif /* B4000000 */
    }
    return -1;
}

static int32_t getGenericBaudrate(speed_t baudrate)
{
    switch (baudrate) {
    case B0:
        return 0;
    case B50:
        return 50;
    case B75:
        return 75;
    case B110:
        return 110;
    case B134:
        return 134;
    case B150:
        return 150;
    case B200:
        return 200;
    case B300:
        return 300;
    case B600:
        return 600;
    case B1200:
        return 1200;
    case B1800:
        return 1800;
    case B2400:
        return 2400;
    case B4800:
        return 4800;
    case B9600:
        return 9600;
#ifdef B14400
        case B14400:
        return 14400;
#endif /* B14400 */
    case B19200:
        return 19200;
#ifdef B28800
        case B28800:
        return 28800;
#endif /* B28800 */
    case B38400:
        return 38400;
#ifdef B57600 // MacOS X does not define this Baud rate
    case B57600:
        return 57600;
#endif // B57600
#ifdef B115200
    case B115200:
        return 115200;
#endif /* B115200 */
#ifdef B230400
    case B230400:
        return 230400;
#endif /* B230400 */
#ifdef B460800
    case B460800:
        return 460800;
#endif /* B460800 */
#ifdef B500000
    case B500000:
        return 500000;
#endif /* B500000 */
#ifdef B576000
    case B576000:
        return 576000;
#endif /* B576000 */
#ifdef B921600
    case B921600:
        return 921600;
#endif /* B921600 */
#ifdef B1000000
    case B1000000:
        return 1000000;
#endif /* B1000000 */
#ifdef B1152000
    case B1152000:
        return 1152000;
#endif /* B1152000 */
#ifdef B1500000
    case B1500000:
        return 1500000;
#endif /* B1500000 */
#ifdef B2000000
    case B2000000:
        return 2000000;
#endif /* B2000000 */
#ifdef B2500000
    case B2500000:
        return 2500000;
#endif /* B2500000 */
#ifdef B3000000
    case B3000000:
        return 3000000;
#endif /* B3000000 */
#ifdef B3500000
    case B3500000:
        return 3500000;
#endif /* B3500000 */
#ifdef B4000000
    case B4000000:
        return 4000000;
#endif /* B4000000 */
    default:
        return -1;
    }
}

static void setTermiosDataBits(tcflag_t* cflags, uint8_t databits)
{
    *cflags &= ~CSIZE;
    switch (databits) {
    case 5:
        *cflags |= CS5;
        break;
    case 6:
        *cflags |= CS6;
        break;
    case 7:
        *cflags |= CS7;
        break;
    case 8:
        *cflags |= CS8;
        break;
    default:
        *cflags |= CS8;
        break;
    }
}

static uint8_t getGenericDataBits(tcflag_t cflags)
{
    switch (cflags & CSIZE) {
    case CS5:
        return 5;
    case CS6:
        return 6;
    case CS7:
        return 7;
    case CS8:
        return 8;
    default:
        return -1;
    }
}

// check for the existence of the two HW flow control identifiers
#if defined CNEW_RTSCTS
#define HW_FLOWCTL CNEW_RTSCTS
#elif defined CRTSCTS
#define HW_FLOWCTL CRTSCTS
#else
#define HW_FLOWCTL 0
#endif

// XXX SW flow control missing
static uint32_t getGenericFlowControl(tcflag_t cflags)
{
    if (cflags & HW_FLOWCTL)
        return FLOWCTRL_CTSRTS;
    return FLOWCTRL_NONE;
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    setControl
 * Signature: (II)I
 */
JNIEXPORT jint JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_setControl(JNIEnv* env,
        jobject obj, jint control, jint newValue)
{
    fd_t fd = getFD(env, obj);
    uint8_t ctrl = static_cast<uint8_t>(control);
    struct termios options;    // = { 0 };
    jint old = 0;

    char strCtrl[10];
    trace("set control", itoa(ctrl, strCtrl));

    if (tcgetattr(fd, &options) == -1)
        throwException(env, errno);
    else {
        if (ctrl == BAUDRATE) {
            old = getGenericBaudrate(cfgetispeed(&options));
            int baudrate = getTermiosBaudrate(newValue);
            cfsetispeed(&options, baudrate);
            cfsetospeed(&options, baudrate);
        }
        else if (ctrl == PARITY) {
            old = getGenericParity(options.c_cflag);
            setTermiosParity(&options.c_cflag, newValue);
            // XXX (c_iflag & INPCK) && !(c_iflag & IGNPAR))
        }
        else if (ctrl == DATABITS) {
            old = getGenericDataBits(options.c_cflag);
            setTermiosDataBits(&options.c_cflag, static_cast<uint8_t>(newValue));
        }
        else if (ctrl == STOPBITS) {
            old = getGenericStopBits(options.c_cflag);
            setTermiosStopBits(&options.c_cflag, static_cast<uint8_t>(newValue));
        }
        else if (ctrl == FLOWCTRL) {
            old = getGenericFlowControl(options.c_iflag);
            // disable SW flow ctrl
            options.c_iflag &= ~(IXON | IXOFF | IXANY);
#ifdef HW_FLOWCTL
            if (newValue == FLOWCTRL_CTSRTS) {
                options.c_cflag |= HW_FLOWCTL;
            }
            else if (newValue == FLOWCTRL_NONE) {
                options.c_cflag &= ~HW_FLOWCTL;
            }
            else
                trace("flow control mode not supported");
#else
            trace("flow control mode not supported");
#endif // HW_FLOWCTL
        }

        if (tcsetattr(fd, TCSANOW, &options) == -1) {
            throwException(env, errno);
            trace_error("config port: tcsetattr");
        }
    }
    return old;
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    getControl
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_getControl(JNIEnv* env,
        jobject obj, jint control)
{
    fd_t fd = getFD(env, obj);
    uint8_t ctrl = static_cast<uint8_t>(control);

    char strCtrl[10];
    trace("get control", itoa(ctrl, strCtrl));

    struct termios options;
    if (tcgetattr(fd, &options) == -1)
        throwException(env, errno);
    else if (ctrl == BAUDRATE)
        return getGenericBaudrate(cfgetispeed(&options));
    else if (ctrl == PARITY)
        return getGenericParity(options.c_cflag);
    else if (ctrl == DATABITS)
        return getGenericDataBits(options.c_cflag);
    else if (ctrl == STOPBITS)
        return getGenericStopBits(options.c_cflag);
    return 0;
}

static bool drain(fd_t fd)
{
    int ret = 0;
    do {
        ret = tcdrain(fd);
    } while (ret != 0 && errno == EINTR);
    return ret == 0;
}

static ssize_t write(JNIEnv* env, fd_t fd, const uint8_t* pBuf, size_t off, size_t len)
{
    trace("start write");

    ssize_t written = write(fd, pBuf + off, len);
    if (written < 0)
        throwException(env, errno);
    else if (!drain(fd))
        throwException(env, errno);
    trace("end write");
    return written;
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    writeBytes
 * Signature: ([BII)I
 */
JNIEXPORT jint JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_writeBytes(JNIEnv* env,
        jobject obj, jbyteArray buf, jint off, jint len)
{
    jboolean iscopy;
    return static_cast<jint>(write(env, getFD(env, obj),
            reinterpret_cast<uint8_t*>(env->GetByteArrayElements(buf, &iscopy)), off, len));
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    write
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_write(JNIEnv* env,
        jobject obj, jint byte)
{
    uint8_t out = static_cast<uint8_t>(byte);
    return static_cast<jint>(write(env, getFD(env, obj), &out, 0, 1));
}

static uint32_t isInputWaiting(JNIEnv* env, fd_t fd)
{
    fcntl(fd, F_SETFL, O_NONBLOCK);
    uint32_t bytes = 0;
    if (ioctl(fd, FIONREAD, &bytes) == -1) {
#if defined FIORDCHK
        int rdchk = ioctl(fd, FIORDCHK, 0);
        if (rdchk > -1)
            bytes = rdchk;
        else
#endif // FIORDCHK
            throwException(env, errno);
    }
    return bytes;
}

static int32_t receiveTimeout = 500; // ms

#if defined DEBUG
static ssize_t read(fd_t fd, uint8_t* buffer, uint32_t off, uint32_t len)
#else
static ssize_t read(fd_t fd, uint8_t* buffer, uint32_t /*off*/, uint32_t len)
#endif
{
#if defined DEBUG
	uint64_t start = timestamp_us();
#endif

    fd_set input;
    const int maxFd = fd + 1;
    size_t offset = 0;
    size_t remaining = len;
    while (remaining > 0) {
        FD_ZERO(&input);
        FD_SET(fd, &input);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = receiveTimeout * 1000;
        int n = select(maxFd, &input, NULL, NULL, &timeout);
        if (n == -1) {
            perror("select failed");
            // EINTR: simply continue with the byte counting loop, since we have to calculate
            // a new timeout
            if (errno == EINTR)
                ;
            else
                // e.g., EBADF
                break;
        }
        else if (n == 0) {
//            trace("read timeout");
            break;
        }
        else {
            // read received bytes from stream into buffer
            if (FD_ISSET(fd, &input)) {

            	// get number of bytes that are immediately available for reading:
            	// if 0, this indicates other errors, e.g., disconnected usb adapter
            	size_t len = 0;
                ioctl(fd, FIONREAD, &len);
                if (len == 0)
                    return -1;

                ssize_t ret = read(fd, buffer + offset, remaining);
                if (ret == -1) {
                    perror("read");
                    // retry if error is EAGAIN or EINTR, otherwise bail out
#ifdef EWOULDBLOCK
                    if (errno == EWOULDBLOCK) // alias for EAGAIN
                        ;
                    else
#endif // EWOULDBLOCK
                    if (errno == EAGAIN || errno == EINTR)
                        ;
                    else
                        break;
                }
                else {
                    offset += ret;
                    remaining -= ret;
                }
            }
        }
    }

#if defined DEBUG
    char buf[1024];
    buf[0] = 0;
    uint8_t* pb = buffer + off;
    if (offset > 0) {
        int used = sprintf(buf, "read data [%llu us] (length %zu):", (unsigned long long) timestamp_diff(start), offset);
        for (unsigned i = 0; i < offset; ++i)
            snprintf(buf + used + 3 * i, 4, " %02X", *pb++);
        trace(buf);
    }
#endif // DEBUG
    return offset;
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    readBytes
 * Signature: ([BII)I
 */
JNIEXPORT jint JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_readBytes(JNIEnv* env,
        jobject obj, jbyteArray b, jint off, jint len)
{
    jboolean iscopy;
    jbyte* pBuf = env->GetByteArrayElements(b, &iscopy);
    ssize_t r = read(getFD(env, obj), reinterpret_cast<uint8_t*>(pBuf), off, len);
    env->ReleaseByteArrayElements(b, pBuf, 0);
    if (r == -1)
        throwException(env, errno);
    return static_cast<jint>(r);
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    read
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_read(JNIEnv* env,
        jobject obj)
{
    uint8_t in = 0;
    ssize_t ret = read(getFD(env, obj), &in, 0, 1);
    if (ret == -1)
        throwException(env, errno);
    else if (ret == 0)
        return -1;
    return in;
}

static const uint32_t EVENT_RXCHAR = io_calimero_serial_provider_jni_TtySerialCom_EVENT_RXCHAR;
static const uint32_t EVENT_RXFLAG = io_calimero_serial_provider_jni_TtySerialCom_EVENT_RXFLAG;
static const uint32_t EVENT_TXEMPTY = io_calimero_serial_provider_jni_TtySerialCom_EVENT_TXEMPTY;
static const uint32_t EVENT_CTS = io_calimero_serial_provider_jni_TtySerialCom_EVENT_CTS;
static const uint32_t EVENT_DSR = io_calimero_serial_provider_jni_TtySerialCom_EVENT_DSR;
static const uint32_t EVENT_RLSD = io_calimero_serial_provider_jni_TtySerialCom_EVENT_RLSD;
static const uint32_t EVENT_BREAK = io_calimero_serial_provider_jni_TtySerialCom_EVENT_BREAK;
static const uint32_t EVENT_ERR = io_calimero_serial_provider_jni_TtySerialCom_EVENT_ERR;
static const uint32_t EVENT_RING = io_calimero_serial_provider_jni_TtySerialCom_EVENT_RING;

// XXX
static int currentEventMask = 0;

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    setEvents
 * Signature: (IZ)V
 */
JNIEXPORT void JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_setEvents(JNIEnv* /*env*/,
        jobject /*obj*/, jint eventMask, jboolean enable)
{
    //fd_t h = getFD(env, obj);
    int events = 0;
    // XXX assign those
    if (eventMask & EVENT_RXCHAR)
        events |= 0;
    if (eventMask & EVENT_RXFLAG)
        events |= 0;
    if (eventMask & EVENT_TXEMPTY)
        events |= 0;

    if (eventMask & EVENT_CTS)
        events |= TIOCM_CTS;
    if (eventMask & EVENT_DSR)
        events |= TIOCM_DSR;
    if (eventMask & EVENT_RLSD)
        events |= TIOCM_CAR;
    // XXX assign those
    if (eventMask & EVENT_BREAK)
        events |= 0;
    if (eventMask & EVENT_ERR)
        events |= 0;
    if (eventMask & EVENT_RING)
        events |= TIOCM_RNG;

    if (enable)
        currentEventMask |= events;
    else
        currentEventMask &= ~events;

    //if (!GetCommMask(h, &mask))
    //    throwException(env, errno);
    //else {
    //   mask = enable ? mask | eventMask : mask & ~eventMask;
    //   if (!SetCommMask(h, mask))
    //       throwException(env, errno);
    //}
    trace("set events");
}

// counter struct is not defined on, e.g., MacOS X
#if defined TIOCGICOUNT

static uint32_t getGenericLineEvents(struct serial_icounter_struct sics)
{
    uint32_t events = 0;
    // XXX quick and dirty
    if (sics.rx)
        events |= EVENT_RXCHAR;
    if (sics.rx)
        events |= EVENT_RXFLAG;
    if (sics.tx == 0)
        events |= EVENT_TXEMPTY;
    if (sics.cts)
        events |= EVENT_CTS;
    if (sics.dsr)
        events |= EVENT_DSR;
    if (sics.dcd)
        events |= EVENT_RLSD;
    if (sics.brk)
        events |= EVENT_BREAK;
    if (sics.frame || sics.buf_overrun || sics.parity)
        events |= EVENT_ERR;
    if (sics.rng)
        events |= EVENT_RING;
    return events;
}

static struct serial_icounter_struct queryInterruptCounters(JNIEnv* env, fd_t fd)
{
    //struct serial_icounter_struct {
    //        int cts, dsr, rng, dcd;
    //        int rx, tx;
    //        int frame, overrun, parity, brk;
    //        int buf_overrun;
    //        int reserved[9];
    //};
    struct serial_icounter_struct sics;
    int ret;
    do {
        ret = ioctl(fd, TIOCGICOUNT, &sics);
    } while (ret == -1 && errno == EINTR);
    if (ret == -1)
        throwException(env, errno);
    return sics;
}

#else

struct serial_icounter_struct { };

static uint32_t getGenericLineEvents(struct serial_icounter_struct /*sics*/)
{
    return 0;
}

static struct serial_icounter_struct queryInterruptCounters(JNIEnv* /*env*/, fd_t /*fd*/)
{
    return serial_icounter_struct();
}

#endif // TIOCGICOUNT

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    waitEvent
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_waitEvent(JNIEnv* env,
        jobject obj)
{
    // maybe use TIOCGICOUNT

    trace("in wait event");
    fd_t fd = getFD(env, obj);
    int mask = currentEventMask;
    int ret;
    do {
        ret = ioctl(fd, TIOCMIWAIT, mask);
        if (ret == -1 && errno == EINTR)
            puts("waitEvent interrupted");
    } while (ret == -1 && errno == EINTR);
    if (ret == -1)
        throwException(env, errno);

    // XXX Win behavior: if event mask was changed while waiting for event we return 0
    return getGenericLineEvents(queryInterruptCounters(env, fd));
}

static const uint8_t LINE_STATUS = io_calimero_serial_provider_jni_TtySerialCom_LINE_STATUS;
static const uint8_t ERROR_STATUS = io_calimero_serial_provider_jni_TtySerialCom_ERROR_STATUS;
static const uint8_t INPUT_AVAIL =
        io_calimero_serial_provider_jni_TtySerialCom_AVAILABLE_INPUT_STATUS;

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    getStatus
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_getStatus(JNIEnv* env,
        jobject obj, jint type)
{
    trace("get status");

    fd_t fd = getFD(env, obj);
    uint32_t value = 0;
    int ret = 0;
    if (type == LINE_STATUS) {
        // line status bit field
        // 0x0010 : CTS (clear-to-send) signal is on
        // 0x0020 : DSR (data-set-ready) signal is on
        // 0x0040 : ring indicator signal is on
        // 0x0080 : RLSD (receive-line-signal-detect) signal is on
        const uint32_t LINE_CTS = 0x0010;
        const uint32_t LINE_DSR = 0x0020;
        const uint32_t LINE_RING = 0x0040;
        const uint32_t LINE_RLSD = 0x0080; // aka DCD "Data Carrier Detect" (TIOCM_CAR)
        uint32_t status;
        ret = ioctl(fd, TIOCMGET, &status);
        if (ret != -1) {
            if (status & TIOCM_CTS)
                value |= LINE_CTS;
            if (status & TIOCM_DSR)
                value |= LINE_DSR;
            if (status & TIOCM_RNG)
                value |= LINE_RING;
            if (status & TIOCM_CAR)
                value |= LINE_RLSD;
        }
    }
    else if (type == INPUT_AVAIL) {
        value = isInputWaiting(env, fd);
    }
    else if (type == ERROR_STATUS) {
        // 0x0010 : detected a break condition
        // 0x0008 : detected a framing error
        // 0x0002 : character-buffer overrun
        // 0x0001 : input buffer overflow (no room in the input buffer, or character received after EOF)
        // 0x0004 : detected a parity error

        // XXX
    }

    trace("exit status");

    if (ret == -1)
        throwException(env, errno);
    return value;
}

static int getInt(JNIEnv* env, jobject obj, const char* name)
{
    return env->GetIntField(obj, env->GetFieldID(env->GetObjectClass(obj), name, "I"));
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    setTimeouts
 * Signature: (Lio/calimero/serial/provider/jni/TtySerialCom/Timeouts;)V
 */
JNIEXPORT void JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_setTimeouts(JNIEnv* env,
        jobject obj, jobject timeouts)
{
    uint32_t readIntervalTimeout = getInt(env, timeouts, "readInterval");
    uint32_t readTotalTimeoutMultiplier = getInt(env, timeouts, "readTotalMultiplier");
    uint32_t readTotalTimeoutConstant = getInt(env, timeouts, "readTotalConstant");
    //uint32_t writeTotalTimeoutMultiplier = getInt(env, timeouts, "writeTotalMultiplier");
    //uint32_t writeTotalTimeoutConstant = getInt(env, timeouts, "writeTotalConstant");

    // VMIN specifies the minimum number of characters to read. If set to 0, then the VTIME
    // value specifies the time to wait for every character read. If VMIN is non-zero, VTIME
    // specifies the time to wait for the first character read. If a character is read within
    // the time given, read will block until all VMIN characters are read. That is, once the first
    // character is read, the serial interface driver expects to receive an entire packet of
    // characters (VMIN bytes total). If no character is read within the time allowed, then the
    // call to read returns 0. VTIME specifies the amount of time to wait for incoming characters
    // in tenths of seconds. If VTIME is 0, read waits indefinitely unless the NDELAY option is set.

    fd_t fd = getFD(env, obj);
    struct termios options;
    if (tcgetattr(fd, &options) == -1)
        throwException(env, errno);
    else {
        const uint32_t MAXDWORD = 0; // XXX define value
        cc_t vmin = 0;
        cc_t vtime = 0;
        if (readTotalTimeoutMultiplier == 0 && readTotalTimeoutConstant == 0
                && readIntervalTimeout == MAXDWORD) {
            // setting: return immediately from read, even if no characters read
            vmin = 0;
            vtime = 0;
            trace("set timeouts: return immediately from read, even if no characters read");
        }
        else if (readTotalTimeoutMultiplier == 0 && readTotalTimeoutConstant == 0) {
            // setting: no total timeouts used for read operations, block for next char
            vmin = 1;
            vtime = 0;
            trace("set timeouts: no total timeouts used for read operations, block for next char");
        }
        else if (readIntervalTimeout > 0) {
            // setting: enforce an inter-character timeout after reading the first char
            vmin = 255;
            vtime = static_cast<cc_t>(readIntervalTimeout / 100);
            trace("set timeouts: enforce an inter-character timeout after reading the first char");
        }
        else if (readTotalTimeoutConstant > 0 && readTotalTimeoutMultiplier == 0
                && readIntervalTimeout == 0) {
            // setting: set a maximum timeout to wait for next character
            vmin = 0;
            vtime = static_cast<cc_t>(readTotalTimeoutConstant / 100);
            if (vtime == 0)
                vtime = 1;
            receiveTimeout = readTotalTimeoutConstant;
            trace("set timeouts: set a maximum timeout to wait for next character");
        }
        else {
            // XXX hmm
            // if we're here we have: multiplier > 0 or totalConstant > 0 or interval = 0
        }

        options.c_cc[VMIN] = vmin;
        options.c_cc[VTIME] = vtime;
    }

    if (tcsetattr(fd, TCSANOW, &options) == -1) {
        trace_error("set timeouts: tcsetattr");
        throwException(env, errno);
    }
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    getTimeouts
 * Signature: ()Lio/calimero/serial/provider/jni/TtySerialCom/Timeouts;
 */
JNIEXPORT jobject JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_getTimeouts(
        JNIEnv* env, jobject obj)
{
    trace("get timeouts");
    fd_t fd = getFD(env, obj);
    jobject timeouts = 0;
    struct termios options;
    if (tcgetattr(fd, &options) == -1) {
        throwException(env, errno);
        trace_error("get timeouts: tcgetattr");
    }
    else {
        jclass cls = env->FindClass("io/calimero/serial/provider/jni/TtySerialCom$Timeouts");
        jmethodID ctor = env->GetMethodID(cls, "<init>", "(IIIII)V");
        cc_t vmin = options.c_cc[VMIN];
        cc_t vtime = options.c_cc[VTIME];

        uint32_t readIntervalTimeout = 0;
        uint32_t readTotalTimeoutMultiplier = 0;
        uint32_t readTotalTimeoutConstant = 0;
        uint32_t writeTotalTimeoutMultiplier = 0;
        uint32_t writeTotalTimeoutConstant = 0;

        const uint32_t MAXDWORD = 0; // XXX define
        if ((vmin > 0) && (vtime == 0)) { // XXX where did we remember the initial time settings
            readIntervalTimeout = readTotalTimeoutMultiplier = readTotalTimeoutConstant = 0;
        }
        else if ((vmin == 0) && (vtime > 0)) {
            readTotalTimeoutConstant = vtime * 100;
            readIntervalTimeout = readTotalTimeoutMultiplier = 0;
        }
        else if ((vmin > 0) && (vtime > 0)) {
            readIntervalTimeout = vtime * 100;
        }
        else if ((vmin == 0) && (vtime == 0)) {
            readIntervalTimeout = MAXDWORD;
            readTotalTimeoutConstant = readTotalTimeoutMultiplier = 0;
        }

        timeouts = env->NewObject(cls, ctor, readIntervalTimeout, readTotalTimeoutMultiplier,
                readTotalTimeoutConstant, writeTotalTimeoutMultiplier, writeTotalTimeoutConstant);
        if (timeouts == 0)
            throwException(env, 0);
    }
    return timeouts;
}
