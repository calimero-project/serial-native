/*
    Calimero 2 - A library for KNX network access
    Copyright (c) 2006, 2022 B. Malinowsky

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

    Linking this library statically or dynamically with other modules is
    making a combined work based on this library. Thus, the terms and
    conditions of the GNU General Public License cover the whole
    combination.

    As a special exception, the copyright holders of this library give you
    permission to link this library with independent modules to produce an
    executable, regardless of the license terms of these independent
    modules, and to copy and distribute the resulting executable under terms
    of your choice, provided that you also meet, for each linked independent
    module, the terms and conditions of the license of that module. An
    independent module is a module which is not derived from or based on
    this library. If you modify this library, you may extend this exception
    to your version of the library, but you are not obligated to do so. If
    you do not wish to do so, delete this exception statement from your
    version.
*/

package io.calimero.serial.provider.ffm;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.StringWriter;
import java.lang.foreign.Addressable;
import java.lang.foreign.GroupLayout;
import java.lang.foreign.MemoryAddress;
import java.lang.foreign.MemoryLayout;
import java.lang.foreign.MemoryLayout.PathElement;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.MemorySession;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.MethodType;
import java.lang.invoke.VarHandle;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Duration;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicInteger;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.unix.Linux;
import org.unix.fd_set;
import org.unix.stat;
import org.unix.termios;
import org.unix.timeval;

import tuwien.auto.calimero.DataUnitBuilder;
import tuwien.auto.calimero.serial.spi.SerialCom;

/**
 * Provider for serial communication using Panama FFM to invoke Linux/macOS C functions. The implementation of this API
 * contains platform dependent code.
 *
 * @author B. Malinowsky
 */
final class TtySerialCom implements SerialCom {
	private static final boolean debug = true;
	private static boolean debug() {
		return debug;
	}

	// flow control modes
	private static final int FLOWCTRL_NONE = 0;
	private static final int FLOWCTRL_CTSRTS = 1;

	private static final int ERROR_STATUS = 1;
	static final int AVAILABLE_INPUT_STATUS = 2;
	private static final int LINE_STATUS = 3;

	// according EV #defines in winbase.h
	// CTS, DSR, RLSD and RING are voltage level change events

	// Any Character received
	private static final int EVENT_RXCHAR = 0x0001;
	// Received certain character
	private static final int EVENT_RXFLAG = 0x0002;
	// Transmit Queue Empty
	private static final int EVENT_TXEMPTY = 0x0004;
	// CTS changed state
	private static final int EVENT_CTS = 0x0008;
	// DSR changed state
	private static final int EVENT_DSR = 0x0010;
	// RLSD changed state
	private static final int EVENT_RLSD = 0x0020;
	// BREAK received
	private static final int EVENT_BREAK = 0x0040;
	// Line status error occurred
	private static final int EVENT_ERR = 0x0080;
	// Ring signal detected
	private static final int EVENT_RING = 0x0100;

	// error flags
	// #define CE_RXOVER 0x0001 // Receive Queue overflow
	// #define CE_OVERRUN 0x0002 // Receive Overrun Error
	// #define CE_RXPARITY 0x0004 // Receive Parity Error
	// #define CE_FRAME 0x0008 // Receive Framing error
	// #define CE_BREAK 0x0010 // Break Detected
	// #define CE_TXFULL 0x0100 // TX Queue is full
	// #define CE_MODE 0x8000 // Requested mode unsupported

	private static final String lockDir = "/var/lock/";
	private static final String lckPrefix = "LCK..";
	private static final String pidPrefix = "PID..";

	private static final boolean macos = System.getProperty("os.name").toLowerCase(Locale.ENGLISH).contains("mac os x");
	private static boolean macOs() {
		return macos;
	}

	private record fd_t(int value) {
		static final fd_t INVALID_FD = new fd_t(-1);

		static fd_t of(final int fd) {
			if (fd == -1) return INVALID_FD;
			return new fd_t(fd);
		}
	}

	record Timeouts(int readInterval, int readTotalMultiplier, int readTotalConstant, int writeTotalMultiplier,
			int writeTotalConstant) {}


	private final Logger logger;

	private fd_t fd = fd_t.INVALID_FD;
	private String lockedPort = "";
	private int currentEventMask = 0;

	private final InputStream is;
	private final OutputStream os;


	TtySerialCom() {
		logger = LoggerFactory.getLogger(MethodHandles.lookup().lookupClass());
		is = new PortInputStream(this);
		os = new PortOutputStream(this);
	}

	TtySerialCom(final String portId, final int baudrate, final int databits, final StopBits stopbits,
			final Parity parity, final FlowControl flowControl, final Duration readIntervalTimeout,
			final Duration receiveTimeout) throws IOException {
		logger = LoggerFactory.getLogger("io.calimero.serial.provider.ffm:" + portId);
		open(portId);
		setBaudrate(baudrate);
		setDatabits(databits);
		setStopbits(stopbits);
		setParity(parity);
		setFlowControl(flowControl);
		setTimeouts(new Timeouts((int) readIntervalTimeout.toMillis(), 0, (int) receiveTimeout.toMillis(), 0, 0));
		is = new PortInputStream(this);
		os = new PortOutputStream(this);
	}

	@Override
	public int baudRate() throws IOException {
		trace("get baudrate");
		try (var session = MemorySession.openConfined()) {
			final var options = tcgetattr(session);
			return getGenericBaudrate(Linux.cfgetispeed(options));
		}
	}

	@Override
	public InputStream inputStream() {
		return is;
	}

	@Override
	public OutputStream outputStream() {
		return os;
	}

	// any open input/output stream accessing this port becomes unusable
	@Override
	public final void close() {
		try {
			if (!closePort())
				logger.error("closing serial port: {}", errnoMsg());
		}
		catch (final IOException e) {
			logger.error("closing serial port", e);
		}
	}

	@Override
	public String toString() {
		if (fd == fd_t.INVALID_FD)
			return "closed";
		try {
			return "baudrate " + baudRate() + ", " + getParity() + " parity, " + getDatabits() + " databits, "
					+ getStopbits() + " stopbits, " + getTimeouts();
		}
		catch (final IOException e) {
			return "invalid port setup";
		}
	}

	// this will open the port, alternatives might be to use /dev/serial or /proc/tty
	boolean portExists(final String portId) {
		try {
			final var error = new AtomicInteger();
			final fd_t fd = openPort(portId, false, error);
			if (fd == fd_t.INVALID_FD)
				return false;

			boolean valid = false;
// #if defined TIOCGSERIAL
			if (definedTIOCGSERIAL()) {
//    	final long TIOCGSERIAL = 0;
//    final boolean valid = false;
//    final var info = serial_struct.allocate(session);
//    errno = 0;
//    if (Linux.ioctl(fd, TIOCGSERIAL, &info) == 0) {
//        if (info.type != PORT_UNKNOWN)
//            valid = true;
//        else if (hasDevSerialLink(portID))
//            valid = true;
//    }
//    else
//        trace_error("check port type");
			}
			else
				valid = false;

			closePort(fd.value);
			return valid;
		}
		catch (final IOException e) {
			return false;
		}
	}

	private static String createLockName(final String dir, final String namePrefix, final String name) {
		return dir + namePrefix + name;
	}

	boolean tryLink(final Addressable forName, final MemorySegment linkName) {
		if (Linux.link(forName, linkName) == -1) {
			trace("create link", linkName.getUtf8String(0), "failed:", errnoMsg());
			try (var session = MemorySession.openConfined()) {
				// we are nevertheless successful if lock count states 2 locks
				final var seg = stat.allocate(session);
				if (Linux.stat(forName, seg) == -1)
					return false;
				if (stat.st_nlink$get(seg) != 2)
					return false;
			}
		}
		Linux.unlink(forName);
		return true;
	}

	private static int readPid(final Path filename) throws IOException {
		try {
			final String s = Files.readString(filename);
			return Integer.parseInt(s);
		}
		catch (final NumberFormatException e) {
			throw new IOException(e);
		}
	}

	// try to create PID..pid and write our pid into it
	// create LCK..name as link to pid file
	// all files are located in /var/lock/
	boolean ensureLock(final String port) {
		String p = port;
		if (p.startsWith("/dev/") && p.length() > 5)
			p = p.substring(5);
		final int myPid = Linux.getpid();
		try (var session = MemorySession.openConfined()) {
			final String pidFile = createLockName(lockDir, pidPrefix, "" + myPid);
			final var mpidFile = session.allocateUtf8String(pidFile);
			final fd_t fd = fd_t.of(Linux.open(mpidFile, Linux.O_RDWR() | Linux.O_EXCL() | Linux.O_CREAT(), 0644));
			if (fd.equals(fd_t.INVALID_FD)) {
				trace_error("open", pidFile);
				return false;
			}

			final var mstrPid = session.allocateUtf8String(Integer.toString(myPid));
			/*ssize_t wr = */Linux.write(fd.value(), mstrPid, Linux.strlen(mstrPid));
			Linux.close(fd.value());
			// additional locking check, because O_EXCL in open is only
			// supported when using NFSv3 or later on kernel 2.6 or later
			// (see man open(2) )
			final String lckFile = createLockName(lockDir, lckPrefix, p);
			final var mlckFile = session.allocateUtf8String(lckFile);
			if (tryLink(mpidFile, mlckFile))
				return true;
			boolean locked = false;
			try {
				// lock file exists, read its pid
				// if we cannot read in the pid, we cannot lock
				final int pid = readPid(Path.of(lckFile));
				if (pid == myPid) {
					lockedPort = port;
					locked = true;
				}
				// check the read pid, if stale, try to remove lock file
				else if (Linux.kill(pid, 0) == -1 && errno() != Linux.EPERM()) {
					Linux.unlink(mlckFile);
					if (tryLink(mpidFile, mlckFile)) {
						lockedPort = port;
						return true;
					}
				}
			}
			catch (final IOException e) {
				trace_error("read pid from", lckFile, e.toString());
			}
			// we might already have locked the port
			Linux.unlink(mpidFile);
			return locked;
		}
	}

	void releaseLock() throws IOException {
		if (lockedPort.isEmpty())
			return;
		final int idx = lockedPort.lastIndexOf('/');
		final String name = idx == -1 ? lockedPort : lockedPort.substring(idx + 1);
		final var lockFile = createLockName(lockDir, lckPrefix, name);
		trace("release lock", lockFile);
		lockedPort = "";
		final int pid = readPid(Path.of(lockFile));
		if (pid != -1 && pid == Linux.getpid())
			Files.deleteIfExists(Path.of(lockFile));
	}

	private boolean setPortDefaults(final int fd) {
		try (var session = MemorySession.openConfined()) {
			final var options = termios.allocate(session);
			if (Linux.tcgetattr(fd, options) == -1) {
				trace_error("config port: tcgetattr");
				return false;
			}
			Linux.cfsetispeed(options, Linux.B9600());
			Linux.cfsetospeed(options, Linux.B9600());
			long cflag = termios.c_cflag$get(options);
			cflag |= Linux.CREAD() | Linux.CLOCAL();
			cflag &= ~Linux.CSIZE();
			cflag |= Linux.CS8();
			termios.c_cflag$set(options, (int) cflag);
			termios.c_iflag$set(options, Linux.INPCK());
			termios.c_lflag$set(options, 0);
			termios.c_oflag$set(options, 0);
			termios.c_cc$slice(options).set(Linux.C_CHAR, Linux.VMIN(), (byte) 0);
			termios.c_cc$slice(options).set(Linux.C_CHAR, Linux.VTIME(), (byte) 0);
			if (Linux.tcsetattr(fd, Linux.TCSANOW(), options) == -1) {
				trace_error("config port: tcsetattr");
				return false;
			}
			Linux.fcntl(fd, Linux.F_SETOWN(), Linux.getpid());
		}
		return true;
	}

	private fd_t openPort(final String portId, final boolean configurePort, final AtomicInteger lastError)
			throws IOException {
		trace("open", portId);
		fd_t fd = fd_t.INVALID_FD;
		errno(0);
		int error = 0;

		try (var session = MemorySession.openConfined()) {
			if (ensureLock(portId)) {
				final var port = session.allocateUtf8String(portId);

				do {
					// we set the port exclusive below, not here
					fd = fd_t.of(Linux.open(port, /*O_EXCL |*/Linux.O_RDWR() | Linux.O_NOCTTY() | Linux.O_NONBLOCK()));
					if (!fd.equals(fd_t.INVALID_FD))
						break;
				}
				while (errno() == Linux.EINTR());
			}
			else
				throw new IOException("obtaining port lock for " + portId + ": " + errnoMsg());
			// check if someone else has opened the port
			if (fd.equals(fd_t.INVALID_FD) || errno() == Linux.EBUSY()) {
				releaseLock();
				throw new IOException("opening " + portId + ": " + errnoMsg());
			}

			// we continue if we are not able to set exclusive mode
			if (Linux.ioctl(fd.value(), Linux.TIOCEXCL()) == -1) {
				error = errno();
				perror("set exclusive");
			}

			if (configurePort) {
				// we continue if we are not able to save old port settings
				final var saved = termios.allocate(session);
				if (Linux.tcgetattr(fd.value, saved) == -1) {
					error = errno();
					perror("save old port settings");
				}

				setPortDefaults(fd.value);
			}
		}

		if (lastError != null)
			lastError.set(error);
		return fd;
	}


//#if !defined __APPLE__
//static boolean hasDevSerialLink(final String portId)
//{
//    final String dir = "/dev/serial/by-id";
//    trace("open dir", dir);
//    final dirent *dp;
//    DIR *dfd;
//    if ((dfd = opendir(dir)) == NULL) {
//        trace_error("can't open", dir);
//        return false;
//    }
//
//    final boolean iscopy;
//    final char* port = env->GetStringUTFChars(portId, &iscopy);
//    boolean hasLink = false;
//    char filename[Linux.PATH_MAX];
//    while ((dp = readdir(dfd)) != NULL) {
//        stat stbuf;
//        sprintf(filename, "%s/%s", dir, dp->d_name);
//        if (stat(filename, &stbuf) == -1) {
//            trace_error("error in stat for", filename);
//            continue;
//        }
//        // ignore entries '.' and '..'
//        if (dp->d_name[0] == '.' && strlen(dp->d_name) <= 2)
//            continue;
//
//        trace("found entry", dp->d_name);
//        char abs[PATH_MAX];
//        realpath(filename, abs);
//        if (strcmp(port, abs) == 0) {
//            hasLink = true;
//            dbg_out("%s -> %s\n", dp->d_name, abs);
//            break;
//        }
//    }
//    return hasLink;
//}
//#endif // !defined __APPLE__

	private static boolean definedTIOCGSERIAL() {
		return false;
	}

	public void open(final String portId) throws IOException {
		final AtomicInteger error = new AtomicInteger();
		fd = openPort(portId, true, error);
		if (fd == fd_t.INVALID_FD)
			throwException(error.get());
		else {
			try (var session = MemorySession.openConfined()) {
				if (debug()) {
// #if !defined TXSETIHOG
					final int TXSETIHOG = (('X' << 8) + 9);
					final int TXSETOHOG = (('X' << 8) + 10);

					// set rx and tx queues
//        final /*uint*/ long size = 200;
					final var size = session.allocate(Linux.C_INT, 200);
					// XXX how to set buffers
					if (Linux.ioctl(fd.value, TXSETIHOG, size.address()) == -1)
						perror("TXSETIHOG");
					if (Linux.ioctl(fd.value, TXSETOHOG, size.address()) == -1)
						perror("TXSETOHOG");

					final String out = "set queue tx %d rx %d".formatted(0, 0);
					trace(out);
				} // DEBUG

				final var status = session.allocate(Linux.C_INT);
				int ret = Linux.ioctl(fd.value, Linux.TIOCMGET(), status.address());
				if (ret != -1) {
					status.set(Linux.C_INT, 0, status.get(Linux.C_INT, 0) | Linux.TIOCM_DTR());
					ret = Linux.ioctl(fd.value, Linux.TIOCMSET(), status.address());
				}
				if (ret == -1) {
					final int err = errno();
					closePort();
					throwException(err);
				}
			}
		}
	}

	private boolean closePort() throws IOException {
		final boolean ret = closePort(fd.value);
		fd = fd_t.INVALID_FD;
		return ret;
	}

	private boolean closePort(final int fd) throws IOException {
		if (fd == fd_t.INVALID_FD.value)
			return true;
		trace("close handle");
		boolean closed = false;
		do {
			closed = Linux.close(fd) == 0;
			if (closed)
				break;
		}
		while (errno() == Linux.EINTR());

		releaseLock();

		return closed;
	}

	private int setBaudrate(final int newValue) throws IOException {
		int old = 0;
		trace("set baudrate", Integer.toString(newValue));

		try (var session = MemorySession.openConfined()) {
			final var options = tcgetattr(session);
			old = getGenericBaudrate(Linux.cfgetispeed(options));
			final int baudrate = getTermiosBaudrate(newValue);
			Linux.cfsetispeed(options, baudrate);
			Linux.cfsetospeed(options, baudrate);
			tcsetattr(options);
		}
		return old;
	}

	private void setParity(final Parity parity) throws IOException {
		trace("set parity", parity.toString());
		try (var session = MemorySession.openConfined()) {
			final var options = tcgetattr(session);
			setTermiosParity(options, parity);
			// XXX (c_iflag & INPCK) && !(c_iflag & IGNPAR))
			tcsetattr(options);
		}
	}

	private int setDatabits(final int databits) throws IOException {
		int old = 0;
		trace("set databits", Integer.toString(databits));

		try (var session = MemorySession.openConfined()) {
			final var options = tcgetattr(session);
			old = getGenericDataBits(termios.c_cflag$get(options));
			setTermiosDataBits(options, databits);
			tcsetattr(options);
		}
		return old;
	}

	private void setStopbits(final StopBits stopbits) throws IOException {
		trace("set stopbits", stopbits.toString());

		try (var session = MemorySession.openConfined()) {
			final var options = tcgetattr(session);
			setTermiosStopBits(options, stopbits);
			tcsetattr(options);
		}
	}

	private void setFlowControl(final FlowControl flowControl) throws IOException {
		trace("set flow control", flowControl.toString());

		try (var session = MemorySession.openConfined()) {
			final var options = tcgetattr(session);
			long iflag = termios.c_iflag$get(options);
			// disable SW flow ctrl
			iflag &= ~(Linux.IXON() | Linux.IXOFF() | Linux.IXANY());
			termios.c_iflag$set(options, (int) iflag);
			//# ifdef HW_FLOWCTL
			long cflag = termios.c_cflag$get(options);
//			if (flowControl == FLOWCTRL_CTSRTS) {
//				cflag |= HW_FLOWCTL;
//			}
//			else
			if (flowControl == FlowControl.None) {
				cflag &= ~HW_FLOWCTL;
			}
			else
				trace("flow control mode not supported");
			termios.c_cflag$set(options, (int) cflag);
			//#else
			//            trace("flow control mode not supported");
			//#endif // HW_FLOWCTL

			tcsetattr(options);
		}
	}

	private Parity getParity() throws IOException {
		trace("get parity");
		try (var session = MemorySession.openConfined()) {
			final var options = tcgetattr(session);
			final long cflag = termios.c_cflag$get(options);
			return getGenericParity(cflag);
		}
	}

	private int getDatabits() throws IOException {
		trace("get databits");
		try (var session = MemorySession.openConfined()) {
			final var options = tcgetattr(session);
			final long cflag = termios.c_cflag$get(options);
			return getGenericDataBits(cflag);
		}
	}

	private StopBits getStopbits() throws IOException {
		trace("get stopbits");
		try (var session = MemorySession.openConfined()) {
			final var options = tcgetattr(session);
			return getGenericStopBits(options);
		}
	}

	private void tcsetattr(final MemorySegment options) throws IOException {
		if (Linux.tcsetattr(fd.value, Linux.TCSANOW(), options) == -1) {
			throwException(errno());
			trace_error("config port: tcsetattr");
		}
	}

	private MemorySegment tcgetattr(final MemorySession session) throws IOException {
		final var options = termios.allocate(session);
		if (Linux.tcgetattr(fd.value, options) == -1)
			throwException(errno());
		return options;
	}

	// CSTOPB set corresponds to 2 stop bits, default is 1 stop bit
	private static void setTermiosStopBits(final MemorySegment cflags, /*uint8_t*/ final StopBits stopbits) {
		long flags = termios.c_cflag$get(cflags);

		flags &= ~Linux.CSTOPB();
		switch (stopbits) {
		case Two:
			flags |= Linux.CSTOPB();
			break;
//		case STOPBITS_15: /* does not exist */
//			break;
		case One:
			break;
		default:
			break;
		}
		termios.c_cflag$set(cflags, (int) flags);
	}

	private static final int cstopb = 1024;

	private static StopBits getGenericStopBits(final MemorySegment cflags) {
		final long flags = termios.c_cflag$get(cflags);

		final int stopbits = (int) (flags & Linux.CSTOPB());
		switch (stopbits) {
		// CSTOPB means 2 stop bits
		case cstopb:
			return StopBits.Two;
		default:
			return StopBits.One;
		}
	}

	private void setTermiosParity(final MemorySegment flags, final Parity parity) {
		long cflags = termios.c_cflag$get(flags);

//#ifdef  CMSPAR
//    *cflags &= ~(PARENB | PARODD | CMSPAR);
//#else
		cflags &= ~(Linux.PARENB() | Linux.PARODD());
//#endif // CMSPAR

		switch (parity) {
		case None:
			break;
		case Even:
			cflags |= Linux.PARENB();
			break;
		case Odd:
			cflags |= Linux.PARENB() | Linux.PARODD();
			break;
		case Mark:
			if ((cflags & Linux.CS7()) == Linux.CS7()) {
				trace("setting mark parity for 7M1");
				// for mark parity use 7 data bits, 2 stop bits
				cflags &= ~Linux.CSIZE();
				cflags |= Linux.CSTOPB() | Linux.CS7();
			}
//#if defined CMSPAR
//        else {
//            trace("setting mark parity for 8M1");
//            // 8M1
//            *cflags |= CMSPAR | PARENB | PARODD;
//        }
//#endif // CMSPAR
			break;
		default:
			break;
		}
		termios.c_cflag$set(flags, (int) cflags);
	}

	private static Parity getGenericParity(final long cflags) {
//#ifdef  CMSPAR
//    if ((cflags & PARENB) && (cflags & PARODD) && (cflags & CMSPAR))
//        return PARITY_MARK;
//    //if ((cflags & PARENB) && (cflags & CMSPAR))
//    //  return PARITY_SPACE;
//#endif // CMSPAR

		if ((cflags & Linux.PARENB()) != 0 && (cflags & Linux.PARODD()) != 0)
			return Parity.Odd;
		if ((cflags & Linux.PARENB()) != 0)
			return Parity.Even;

		return Parity.None;
	}

	private static int getTermiosBaudrate(final /*uint*/ int baudrate) {
		switch (baudrate) {
		case 0:
			return Linux.B0();
		case 50:
			return Linux.B50();
		case 75:
			return Linux.B75();
		case 110:
			return Linux.B110();
		case 134:
			return Linux.B134();
		case 150:
			return Linux.B150();
		case 200:
			return Linux.B200();
		case 300:
			return Linux.B300();
		case 600:
			return Linux.B600();
		case 1200:
			return Linux.B1200();
		case 1800:
			return Linux.B1800();
		case 2400:
			return Linux.B2400();
		case 4800:
			return Linux.B4800();
		case 9600:
			return Linux.B9600();
//#ifdef B14400
// undefined on Linux
//		case 14400:
//			return Linux.B14400();
//#endif /* B14400 */
		case 19200:
			return Linux.B19200();
//#ifdef B28800
// undefined on Linux
//		case 28800:
//			return Linux.B28800();
//#endif /* B28800 */
		case 38400:
			return Linux.B38400();
//#final ifdef B57600 // MacOS X does not define this Baud rate
		case 57600:
			return Linux.B57600();
//#endif // B57600
//#final ifdef B115200
		case 115200:
			return Linux.B115200();
//#endif /*  B115200 */
//#final ifdef B230400
		case 230400:
			return Linux.B230400();
//#endif /* B230400 */
//#ifdef B460800
//    case 460800:
//        return Linux.B460800();
//#endif /* B460800 */
//#final ifdef B500000
//    case 500000:
//        return Linux.B500000();
//#endif /* B500000 */
//#ifdef B576000
//    case 576000:
//        return Linux.B576000();
//#endif /* B57600 */
//#ifdef B921600
//    case 921600:
//        return Linux.B921600();
//#endif /* B921600 */
//#ifdef B1000000
//    case 1000000:
//        return Linux.B1000000();
//#endif /* B1000000 */
//#ifdef B1152000
//    case 1152000:
//        return Linux.B1152000();
//#endif /* B1152000 */
//#ifdef B1500000
//    case 1500000:
//        return Linux.B1500000();
//#endif /* B1500000 */
//#ifdef B2000000
//    case 2000000:
//        return Linux.B2000000();
//#endif /* B2000000 */
//#ifdef B2500000
//    case 2500000:
//        return Linux.B2500000();
//#endif /* B2500000 */
//#ifdef B3000000
//    case 3000000:
//        return Linux.B3000000();
//#endif /* B3000000 */
//#ifdef B3500000
//    case 3500000:
//        return Linux.B3500000();
//#endif /* B3500000 */
//#ifdef B4000000
//    case 4000000:
//        return Linux.B4000000();
//#endif /* B4000000 */
		}
		return -1;
	}

	private static int getGenericBaudrate(final long baudrate) {
		if (baudrate == Linux.B0())
			return 0;
		else if (baudrate == Linux.B50())
			return 50;
		else if (baudrate == Linux.B75())
			return 75;
		else if (baudrate == Linux.B110())
			return 110;
		else if (baudrate == Linux.B134())
			return 134;
		else if (baudrate == Linux.B150())
			return 150;
		else if (baudrate == Linux.B200())
			return 200;
		else if (baudrate == Linux.B300())
			return 300;
		else if (baudrate == Linux.B600())
			return 600;
		else if (baudrate == Linux.B1200())
			return 1200;
		else if (baudrate == Linux.B1800())
			return 1800;
		else if (baudrate == Linux.B2400())
			return 2400;
		else if (baudrate == Linux.B4800())
			return 4800;
		else if (baudrate == Linux.B9600())
			return 9600;
//		else if (baudrate == Linux.B14400())
//			return 14400;
		else if (baudrate == Linux.B19200())
			return 19200;
//		else if (baudrate == Linux.B28800())
//			return 28800;
		else if (baudrate == Linux.B38400())
			return 38400;
		else if (baudrate == Linux.B57600())
			return 57600;
		else if (baudrate == Linux.B115200())
			return 115200;
		else if (baudrate == Linux.B230400())
			return 230400;
		else
			return -1;

//#endif /* B230400 */
//#ifdef B460800
//    case Linux.B460800():
//        return 460800;
//#endif /* B460800 */
//#ifdef B500000
//    case Linux.B500000():
//        return 500000;
//#endif /* B500000 */
//#final ifdef B576000
//    case Linux.B576000():
//        return 576000;
//#endif /* B576000 */
//#ifdef B921600
//    case Linux.B921600():
//        return 921600;
//#endif /* B921600 */
//#ifdef B1000000
//    case Linux.B1000000():
//        return 1000000;
//#endif /* B1000000 */
//#ifdef B1152000
//    case Linux.B1152000():
//        return 1152000;
//#endif /* B1152000 */
//#ifdef B1500000
//    case Linux.B1500000():
//        return 1500000;
//#endif /* B1500000 */
//#ifdef B2000000
//    case Linux.B2000000():
//        return 2000000;
//#endif /* B2000000 */
//#ifdef B2500000
//    case Linux.B2500000():
//        return 2500000;
//#endif /* B2500000 */
//#ifdef B3000000
//    case Linux.B3000000():
//        return 3000000;
//#endif /* B3000000 */
//#ifdef B3500000
//    case Linux.B3500000():
//        return 3500000;
//#endif /* B3500000 */
//#ifdef B4000000
//    case Linux.B4000000():
//        return 4000000;
//#endif /* B4000000 */
//    default:
//        return -1;
//    }
	}

	private static void setTermiosDataBits(final MemorySegment flags, /*uint8_t*/ final int databits) {
		long cflags = termios.c_cflag$get(flags);
		cflags &= ~Linux.CSIZE();
		switch (databits) {
		case 5:
			cflags |= Linux.CS5();
			break;
		case 6:
			cflags |= Linux.CS6();
			break;
		case 7:
			cflags |= Linux.CS7();
			break;
		case 8:
			cflags |= Linux.CS8();
			break;
		default:
			cflags |= Linux.CS8();
			break;
		}
		termios.c_cflag$set(flags, (int) cflags);
	}

	private static /*uint8_t*/ int getGenericDataBits(final long cflags) {
		final long i = cflags & Linux.CSIZE();
		if (i == Linux.CS5())
			return 5;
		else if (i == Linux.CS6())
			return 6;
		else if (i == Linux.CS7())
			return 7;
		else if (i == Linux.CS8())
			return 8;
		else
			return -1;
	}

	private static final int HW_FLOWCTL;
	static {
		// check for the existence of the two HW flow control identifiers
//	#if defined CNEW_RTSCTS
//		#define HW_FLOWCTL CNEW_RTSCTS
//	#elif defined CRTSCTS
//		#define HW_FLOWCTL CRTSCTS
//	#else
//		#define HW_FLOWCTL 0
//	#endif

		HW_FLOWCTL = Linux.CRTSCTS();
	}

// XXX SW flow control missing
	private static /*uint*/ long getGenericFlowControl(final long cflags) {
		if ((cflags & HW_FLOWCTL) != 0)
			return FLOWCTRL_CTSRTS;
		return FLOWCTRL_NONE;
	}

	private static boolean drain(final fd_t fd) {
		int ret = 0;
		do {
			ret = Linux.tcdrain(fd.value());
		}
		while (ret != 0 && errno() == Linux.EINTR());
		return ret == 0;
	}

	int readBytes(final byte[] b, final int off, final int len) throws IOException {
		try (var session = MemorySession.openConfined()) {
			final var pBuf = session.allocateArray(ValueLayout.JAVA_BYTE, b);
			final long r = read(fd, pBuf, off, len);
			final var tmp = pBuf.toArray(ValueLayout.JAVA_BYTE);
			System.arraycopy(tmp, 0, b, 0, tmp.length);
			if (r == -1)
				throwException(errno());
			return (int) (r);
		}
	}

	int read() throws IOException {
		try (var session = MemorySession.openConfined()) {
			/*uint8_t*/ final MemorySegment in = session.allocate(1);
			final long ret = read(fd, in, 0, 1);
			if (ret == -1)
				throwException(errno());
			else if (ret == 0)
				return -1;
			return in.get(ValueLayout.JAVA_BYTE, 0) & 0xff;
		}
	}

	private int receiveTimeout = 500; // ms

	private long read(final fd_t fd, /*uint8_t*/ final MemorySegment buffer, /*uint*/ final long off,
			/*uint*/ final long len) {
		final long start = System.nanoTime();

		final int maxFd = fd.value + 1;
		long offset = 0;
		long remaining = len;

		try (var session = MemorySession.openConfined()) {
			final var input = fd_set.allocate(session);
			while (remaining > 0) {
				FD_ZERO(input);
				FD_SET(fd, input);

				final var timeout = timeval.allocate(session);
				timeval.tv_sec$set(timeout, 0);
				timeval.tv_usec$set(timeout, receiveTimeout * 1000);
				final int n = Linux.select(maxFd, input, MemoryAddress.NULL, MemoryAddress.NULL, timeout);
				if (n == -1) {
					perror("select failed");
					// Linux.EINTR(): simply continue with the byte counting loop, since we have to calculate
					// a new timeout
					if (errno() == Linux.EINTR())
						;
					else
						// e.g., EBADF
						break;
				}
				else if (n == 0) {
					//              trace("read timeout");
					break;
				}
				else {
					// read received bytes from stream into buffer
					if (FD_ISSET(fd, input)) {

						// get number of bytes that are immediately available for reading:
						// if 0, this indicates other errors, e.g., disconnected usb adapter
						final /*size_t*/ var nread = session.allocate(ValueLayout.JAVA_LONG, 0);
						Linux.ioctl(fd.value, Linux.FIONREAD(), nread.address());
						if (nread.get(ValueLayout.JAVA_LONG, 0) == 0)
							return -1;

						final long ret = Linux.read(fd.value, buffer.address().addOffset(offset), remaining);
						if (ret == -1) {
							perror("read");
							// retry if error is EAGAIN or Linux.EINTR(), otherwise bail out
							//#ifdef EWOULDBLOCK
							if (errno() == Linux.EWOULDBLOCK()) // alias for EAGAIN
								;
							else
							//#endif // EWOULDBLOCK
							if (errno() == Linux.EAGAIN() || errno() == Linux.EINTR())
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
			if (debug()) {
				if (offset > 0) {
					final long end = System.nanoTime();
					final long diff = end - start;
					final String s = "read data [%d us] (length %d):".formatted(diff / 1000, offset);
					final byte[] data = buffer.asSlice(off, offset).toArray(ValueLayout.JAVA_BYTE);
					final String hex = DataUnitBuilder.toHex(data, " ");
					trace(s, hex);
				}
			}
		}
		return offset;
	}

	int writeBytes(final byte[] buf, final int off, final int len) throws IOException {
		try (var session = MemorySession.openConfined()) {
			return (int) write(fd, session.allocateArray(ValueLayout.JAVA_BYTE, buf), off, len);
		}
	}

	int write(final int bite) throws IOException {
		try (var session = MemorySession.openConfined()) {
			final var out = session.allocate(ValueLayout.JAVA_BYTE, (byte) bite);
			return (int) write(fd, out, 0, 1);
		}
	}

	private long write(final fd_t fd, /*uint8_t*/ final MemorySegment pBuf, final long off, final long len)
			throws IOException {
		final var hex = DataUnitBuilder.toHex(pBuf.toArray(ValueLayout.JAVA_BYTE), " ");
		trace("start write data (length " + len + "):", hex);

		final long written = Linux.write(fd.value, pBuf.address().addOffset(off), len);
		if (written < 0)
			throwException(errno());
		else if (!drain(fd))
			throwException(errno());
		trace("end write");
		return written;
	}

	private static /*uint*/ long isInputWaiting(final fd_t fd) throws IOException {
		Linux.fcntl(fd.value, Linux.F_SETFL(), Linux.O_NONBLOCK());

		try (var session = MemorySession.openConfined()) {
			/*uint*/ final var bytes = session.allocate(ValueLayout.JAVA_INT);
			if (Linux.ioctl(fd.value, Linux.FIONREAD(), bytes.address()) == -1) {

	//#if defined FIORDCHK
				if (definedFIORDCHK()) {
	//        final int rdchk = Linux.ioctl(fd, Linux.FIORDCHK, 0);
	//        if (rdchk > -1)
	//            bytes = rdchk;
	//        else
	//        	throwException(errno());
				} // FIORDCHK
				else
					throwException(errno());
			}
			return bytes.get(ValueLayout.JAVA_INT, 0);
		}
	}

	private static boolean definedFIORDCHK() {
		// TODO Auto-generated method stub
		return false;
	}

	private void setEvents(final int eventMask, final boolean enable) {
		//fd_t h = getFD();
		int events = 0;
		// XXX assign those
		if ((eventMask & EVENT_RXCHAR) != 0)
			events |= 0;
		if ((eventMask & EVENT_RXFLAG) != 0)
			events |= 0;
		if ((eventMask & EVENT_TXEMPTY) != 0)
			events |= 0;

		if ((eventMask & EVENT_CTS) != 0)
			events |= Linux.TIOCM_CTS();
		if ((eventMask & EVENT_DSR) != 0)
			events |= Linux.TIOCM_DSR();
		if ((eventMask & EVENT_RLSD) != 0)
			events |= Linux.TIOCM_CAR();
		// XXX assign those
		if ((eventMask & EVENT_BREAK) != 0)
			events |= 0;
		if ((eventMask & EVENT_ERR) != 0)
			events |= 0;
		if ((eventMask & EVENT_RING) != 0)
			events |= Linux.TIOCM_RNG();

		if (enable)
			currentEventMask |= events;
		else
			currentEventMask &= ~events;

		//if (!GetCommMask(h, &mask))
		//    throwException(errno());
		//else {
		//   mask = enable ? mask | eventMask : mask & ~eventMask;
		//   if (!SetCommMask(h, mask))
		//       throwException(errno());
		//}
		trace("set events");
	}

// counter is not defined on, e.g., MacOS X
//#if defined TIOCGICOUNT
//
//static /*uint*/ long getGenericLineEvents(final serial_icounter_sics)
//{
//    /*uint*/ long events = 0;
//    // XXX quick and dirty
//    if (sics.rx)
//        events |= EVENT_RXCHAR;
//    if (sics.rx)
//        events |= EVENT_RXFLAG;
//    if (sics.tx == 0)
//        events |= EVENT_TXEMPTY;
//    if (sics.cts)
//        events |= EVENT_CTS;
//    if (sics.dsr)
//        events |= EVENT_DSR;
//    if (sics.dcd)
//        events |= EVENT_RLSD;
//    if (sics.brk)
//        events |= EVENT_BREAK;
//    if (sics.frame || sics.buf_overrun || sics.parity)
//        events |= EVENT_ERR;
//    if (sics.rng)
//        events |= EVENT_RING;
//    return events;
//}
//
//static serial_icounter_queryInterruptCounters(JNIEnv* env, fd_t fd)
//{
//    //serial_icounter_{
//    //        int cts, dsr, rng, dcd;
//    //        int rx, tx;
//    //        int frame, overrun, parity, brk;
//    //        int buf_overrun;
//    //        int reserved[9];
//    //};
//    serial_icounter_sics;
//    int ret;
//    do {
//        ret = Linux.ioctl(fd, Linux.TIOCGICOUNT, &sics);
//    } while (ret == -1 && errno() == Linux.EINTR());
//    if (ret == -1)
//        throwException(errno());
//    return sics;
//}
//
//#else

	record serial_icounter_struct() {}

	private static /*uint*/ long getGenericLineEvents(final serial_icounter_struct sics) {
		return 0;
	}

	private static serial_icounter_struct queryInterruptCounters(final fd_t fd) {
		return new serial_icounter_struct();
	}

//#endif // TIOCGICOUNT

	private static int TIOCMIWAIT = 0x545C;

	private int waitEvent() throws IOException {
		// maybe use TIOCGICOUNT

		trace("in wait event");
		final int mask = currentEventMask;
		int ret;
		do {
			ret = Linux.ioctl(fd.value, TIOCMIWAIT, mask);
			if (ret == -1 && errno() == Linux.EINTR())
				trace("waitEvent interrupted");
		}
		while (ret == -1 && errno() == Linux.EINTR());
		if (ret == -1)
			throwException(errno());

		// XXX Win behavior: if event mask was changed while waiting for event we return 0
		return (int) getGenericLineEvents(queryInterruptCounters(fd));
	}

	// clear communication error and get device status
	int getStatus(final int type) throws IOException {
		trace("get status");

		/*uint*/ long value = 0;
		int ret = 0;
		if (type == LINE_STATUS) {
			// line status bit field
			// 0x0010 : CTS (clear-to-send) signal is on
			// 0x0020 : DSR (data-set-ready) signal is on
			// 0x0040 : ring indicator signal is on
			// 0x0080 : RLSD (receive-line-signal-detect) signal is on
			/*uint*/ final long LINE_CTS = 0x0010;
			/*uint*/ final long LINE_DSR = 0x0020;
			/*uint*/ final long LINE_RING = 0x0040;
			/*uint*/ final long LINE_RLSD = 0x0080;
			try (var session = MemorySession.openConfined()) {
				// aka DCD "Data Carrier Detect" (TIOCM_CAR)
				/*uint*/ final var mstatus = session.allocate(ValueLayout.JAVA_INT);
				ret = Linux.ioctl(fd.value, Linux.TIOCMGET(), mstatus.address());
				if (ret != -1) {
					final long status = mstatus.get(ValueLayout.JAVA_INT, 0);
					if ((status & Linux.TIOCM_CTS()) != 0)
						value |= LINE_CTS;
					if ((status & Linux.TIOCM_DSR()) != 0)
						value |= LINE_DSR;
					if ((status & Linux.TIOCM_RNG()) != 0)
						value |= LINE_RING;
					if ((status & Linux.TIOCM_CAR()) != 0)
						value |= LINE_RLSD;
				}
			}
		}
		else if (type == AVAILABLE_INPUT_STATUS) {
			value = isInputWaiting(fd);
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
			throwException(errno());
		return (int) value;
	}

	private void setTimeouts(final Timeouts timeouts) throws IOException {
		final int readIntervalTimeout = timeouts.readInterval();
		final int readTotalTimeoutMultiplier = timeouts.readTotalMultiplier();
		final int readTotalTimeoutConstant = timeouts.readTotalConstant();
//		final int writeTotalTimeoutMultiplier = timeouts.writeTotalMultiplier();
//		final int writeTotalTimeoutConstant = timeouts.writeTotalConstant();

		// VMIN specifies the minimum number of characters to read. If set to 0, then the VTIME
		// value specifies the time to wait for every character read. If VMIN is non-zero, VTIME
		// specifies the time to wait for the first character read. If a character is read within
		// the time given, read will block until all VMIN characters are read. That is, once the first
		// character is read, the serial interface driver expects to receive an entire packet of
		// characters (VMIN bytes total). If no character is read within the time allowed, then the
		// call to read returns 0. VTIME specifies the amount of time to wait for incoming characters
		// in tenths of seconds. If VTIME is 0, read waits indefinitely unless the NDELAY option is set.

		try (var session = MemorySession.openConfined()) {
			final var options = termios.allocate(session);
			if (Linux.tcgetattr(fd.value, options) == -1)
				throwException(errno());
			else {
				/*uint*/ final long MAXDWORD = 0; // XXX define value
				int vmin = 0;
				int vtime = 0;
				if (readTotalTimeoutMultiplier == 0 && readTotalTimeoutConstant == 0 && readIntervalTimeout == MAXDWORD) {
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
					vtime = (readIntervalTimeout / 100);
					trace("set timeouts: enforce an inter-character timeout after reading the first char");
				}
				else if (readTotalTimeoutConstant > 0 && readTotalTimeoutMultiplier == 0 && readIntervalTimeout == 0) {
					// setting: set a maximum timeout to wait for next character
					vmin = 0;
					vtime = (readTotalTimeoutConstant / 100);
					if (vtime == 0)
						vtime = 1;
					receiveTimeout = readTotalTimeoutConstant;
					trace("set timeouts: set a maximum timeout to wait for next character:", "" + receiveTimeout, "ms");
				}
				else {
					// XXX hmm
					// if we're here we have: multiplier > 0 or totalConstant > 0 or interval = 0
				}

				termios.c_cc$slice(options).set(ValueLayout.JAVA_BYTE, Linux.VMIN(), (byte) vmin);
				termios.c_cc$slice(options).set(ValueLayout.JAVA_BYTE, Linux.VTIME(), (byte) vtime);
			}

			if (Linux.tcsetattr(fd.value, Linux.TCSANOW(), options) == -1) {
				trace_error("set timeouts: tcsetattr");
				throwException(errno());
			}
		}
	}

	private Timeouts getTimeouts() throws IOException {
		trace("get timeouts");
		try (var session = MemorySession.openConfined()) {
			final var options = termios.allocate(session);
			if (Linux.tcgetattr(fd.value, options) == -1) {
				trace_error("get timeouts: tcgetattr");
				throwException(errno());
				return null; // not reached
			}

			final byte vmin = termios.c_cc$slice(options).get(ValueLayout.JAVA_BYTE, Linux.VMIN());
			final byte vtime = termios.c_cc$slice(options).get(ValueLayout.JAVA_BYTE, Linux.VTIME());

			/*uint*/ int readIntervalTimeout = 0;
			/*uint*/ int readTotalTimeoutMultiplier = 0;
			/*uint*/ int readTotalTimeoutConstant = 0;
			/*uint*/ final int writeTotalTimeoutMultiplier = 0;
			/*uint*/ final int writeTotalTimeoutConstant = 0;

			/*uint*/ final long MAXDWORD = 0; // XXX define
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
				readIntervalTimeout = (int) MAXDWORD;
				readTotalTimeoutConstant = readTotalTimeoutMultiplier = 0;
			}
			return new Timeouts(readIntervalTimeout, readTotalTimeoutMultiplier, readTotalTimeoutConstant,
					writeTotalTimeoutMultiplier, writeTotalTimeoutConstant);
		}
	}

	private static void throwException(final int error) throws IOException {
		throw new IOException(errnoMsg(error));
	}


	//#if !defined __APPLE__
	//static boolean hasDevSerialLink(final String portId)
	//{
	//    final String dir = "/dev/serial/by-id";
	//    trace("open dir", dir);
	//    final dirent *dp;
	//    DIR *dfd;
	//    if ((dfd = opendir(dir)) == NULL) {
	//        trace_error("can't open", dir);
	//        return false;
	//    }
	//
	//    final boolean iscopy;
	//    final char* port = env->GetStringUTFChars(portId, &iscopy);
	//    boolean hasLink = false;
	//    char filename[Linux.PATH_MAX];
	//    while ((dp = readdir(dfd)) != NULL) {
	//        stat stbuf;
	//        sprintf(filename, "%s/%s", dir, dp->d_name);
	//        if (stat(filename, &stbuf) == -1) {
	//            trace_error("error in stat for", filename);
	//            continue;
	//        }
	//        // ignore entries '.' and '..'
	//        if (dp->d_name[0] == '.' && strlen(dp->d_name) <= 2)
	//            continue;
	//
	//        trace("found entry", dp->d_name);
	//        char abs[PATH_MAX];
	//        realpath(filename, abs);
	//        if (strcmp(port, abs) == 0) {
	//            hasLink = true;
	//            dbg_out("%s -> %s\n", dp->d_name, abs);
	//            break;
	//        }
	//    }
	//    return hasLink;
	//}
	//#endif // !defined __APPLE__


	private final static int __DARWIN_NBBY = 8;                               /* bits in a byte */
	private final static int __DARWIN_NFDBITS = /*sizeof(__int32_t)*/ 4 * __DARWIN_NBBY; /* bits per mask */

	private static final VarHandle fds_bits;
	static {
		final String name = macOs() ? "fds_bits" : "__fds_bits";
		final var valueLayout = (ValueLayout) fd_set.$LAYOUT()
				.select(PathElement.groupElement(name)).select(PathElement.sequenceElement());
		fds_bits = valueLayout.arrayElementVarHandle();
	}

	private static boolean FD_ISSET(final fd_t fd, final MemorySegment fdset) {
		// it's a inline header function, therefore panama doesn't find it at runtime
//		return Linux.__darwin_fd_isset(fd.value, fdset) != 0;

//		__darwin_fd_isset(int _fd, const struct fd_set *_p)
//		{
//			return _p->fds_bits[(unsigned long)_fd / __DARWIN_NFDBITS] & ((__int32_t)(((unsigned long)1) << ((unsigned long)_fd % __DARWIN_NFDBITS)));
//		}

		final long rawfd = fd.value & 0xffff_ffffL;
		final long element = (long) fds_bits.get(fdset, rawfd / __DARWIN_NFDBITS);
		final long bit = element & (1 << (rawfd % __DARWIN_NFDBITS));
		return bit != 0;
	}

	private static void FD_SET(final fd_t fd, final MemorySegment fdset) {
		// it's a inline header function, therefore panama doesn't find it at runtime
//		Linux.__darwin_fd_set(fd.value, fdset);

//		__darwin_fd_set(int _fd, struct fd_set *const _p)
//		{
//			(_p->fds_bits[(unsigned long)_fd / __DARWIN_NFDBITS] |= ((__int32_t)(((unsigned long)1) << ((unsigned long)_fd % __DARWIN_NFDBITS))));
//		}

		final long rawfd = fd.value & 0xffff_ffffL;
		fds_bits.getAndBitwiseOr(fdset, rawfd / __DARWIN_NFDBITS, 1 << (rawfd % __DARWIN_NFDBITS));
	}

	private static void FD_ZERO(final MemorySegment fdset) {
		fdset.fill((byte) 0);
	}

	private void perror(final String msg) {
		final String err = errnoMsg();
		logger.warn("{}: {}", msg, err);
	}

	private static final MethodHandle errno;
	static {
		final String name = macOs() ? "__error" : "__errno_location";
		try {
			errno = MethodHandles.lookup().findStatic(Linux.class, name, MethodType.methodType(MemoryAddress.class));
		}
		catch (NoSuchMethodException | IllegalAccessException e) {
			throw new AssertionError("should not reach here", e);
		}
	}

	private static int errno() {
		try {
			final var addr = (MemoryAddress) errno.invokeExact();
			return addr.get(ValueLayout.JAVA_INT, 0);
		}
		catch (final Throwable e) {
			throw new AssertionError("should not reach here", e);
		}
	}

	private static void errno(final int error) {
		try {
			final var addr = (MemoryAddress) errno.invokeExact();
			addr.set(ValueLayout.JAVA_INT, 0, error);
		}
		catch (final Throwable e) {
			throw new AssertionError("should not reach here", e);
		}
	}

	private static String errnoMsg() {
		return errnoMsg(errno());
	}

	private static String errnoMsg(final int error) {
		// use POSIX strerror (i.e., retrieve error string pointer and copy it over)
		// instead of XSI/GNU strerror_r functions
		// while strerror_r is thread-safe, it is not available on all systems
		//String str = strerror_r(error, msg, 100);
		final MemoryAddress str = Linux.strerror(error);
		return str.getUtf8String(0);
	}

	private void trace_error(final String msg, final String... opt) {
		final var join = String.join(" ", opt);
		logger.warn("{} {}", msg, join);
	}

	private void trace(final String msg, final String... opt) {
		final var join = String.join(" ", opt);
		logger.trace("{} {}", msg, join);
	}

	private static String dump(final MemorySegment seg, final MemoryLayout layout) {
		final var writer = new StringWriter();
		try {
			dump(seg, layout, writer, "");
		}
		catch (final Exception e) {
			e.printStackTrace();
		}
		return writer.toString();
	}

	private static void dump(final MemorySegment seg, final MemoryLayout layout, final StringWriter writer,
			final String indent) {
		if (layout instanceof final GroupLayout gl) {
			writer.write(indent + layout.name().get() + " {\n");
			for (final var member : gl.memberLayouts()) {
				if (member.name().isPresent()) {
					final String name = member.name().get();
					final long offset = gl.byteOffset(PathElement.groupElement(name));
					dump(seg.asSlice(offset, member.byteSize()), member, writer, indent + "\t");
				}
			}
			writer.write(indent + "}\n");
		}
		else if (layout instanceof final ValueLayout vl) {
			final var name = vl.name().get();
			final VarHandle varHandle = vl.varHandle();
			final Object o = varHandle.get(seg);
			writer.write(indent + name + "\t= " + o + "\n");
		}
	}
}
