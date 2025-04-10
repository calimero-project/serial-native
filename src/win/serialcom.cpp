/*
    Calimero 3 - A library for KNX network access
    Copyright (c) 2007, 2024 B. Malinowsky

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
//  It provides Java Native Interface (JNI) access to the Microsoft Windows
//  serial port communication I/O API.
//
//  To generate the .dll, compile and link this file using your favorite C++
//  compiler or IDE, e.g., MS VS or gcc/ld with cygwin.
//

#define WIN32_LEAN_AND_MEAN

#include <windows.h>

#if defined _DEBUG
#if !defined DEBUG
#define DEBUG
#endif
#endif

#include "serialcom.h"

#ifdef __cplusplus
extern "C" {
#endif


#if defined DEBUG
static void FormatError(DWORD error, wchar_t* buf, int size)
{
	buf[0] = 0;
	int len = FormatMessageW(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS
		| FORMAT_MESSAGE_MAX_WIDTH_MASK, 0, error, 0, buf, size, 0);
	while (len > 3 && (buf[len - 1] == ' ' || buf[len - 1] == '.'))
		--len;
	buf[len] = 0;
}

static void Trace(const wchar_t* msg, const wchar_t* msg2 = 0)
{
	const wchar_t* opt = msg2 != 0 ? msg2 : L"";
	wchar_t out[512];
    DWORD error = GetLastError();
	if (error != NO_ERROR && error != ERROR_IO_PENDING) {
		wchar_t errmsg[256];
		FormatError(error, errmsg, 256);
		_snwprintf(out, 512, L"%s %s (0x%X, %s)", msg, opt, error, errmsg);
	}
	else
		_snwprintf(out, 512, L"%s %s", msg, opt);
	OutputDebugStringW(out);
}
#else
static void Trace(const wchar_t* /*msg*/, const wchar_t* /*msg2*/ = 0) {}
#endif

#define QUERY_GETLASTERROR ((DWORD) -1)

static void ThrowException(JNIEnv* env, DWORD dwError)
{
    char msg[256];
    msg[0] = 0;
	if (dwError == QUERY_GETLASTERROR)
		dwError = GetLastError();
	FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM  | FORMAT_MESSAGE_MAX_WIDTH_MASK,
		0, dwError, 0, msg, 256, 0);
    jclass c = env->FindClass("java/io/IOException");
	env->ThrowNew(c, msg);
}

static HANDLE GetFD(JNIEnv* env, jobject obj)
{
	return reinterpret_cast<HANDLE>(env->GetLongField(obj,
		env->GetFieldID(env->GetObjectClass(obj), "fd", "J")));
}

static HANDLE Open(JNIEnv* env, jstring portID, bool overlapped, DWORD* lastError)
{
	jboolean iscopy;
	const jchar* port = env->GetStringChars(portID, &iscopy);
	HANDLE h = CreateFileW(reinterpret_cast<const wchar_t*>(port), GENERIC_READ | GENERIC_WRITE,
		0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | (overlapped ? FILE_FLAG_OVERLAPPED : 0), 0);
	if (lastError)
		*lastError = GetLastError();
	Trace(L"open", reinterpret_cast<const wchar_t*>(port));
	env->ReleaseStringChars(portID, port);
	return h;
}

static bool Close(JNIEnv* env, jobject obj)
{
	HANDLE h = GetFD(env, obj);
	if (h != INVALID_HANDLE_VALUE) {
		Trace(L"close handle");
		return CloseHandle(h) == TRUE;
	}
	return true;
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    portExists
 * Signature: (Ljava/lang/String;)Z
 */
JNIEXPORT jboolean JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_portExists
  (JNIEnv* env, jclass /*c*/, jstring portID)
{
	DWORD error;
	HANDLE h = Open(env, portID, false, &error);
	// if handle is valid, port exists, otherwise it depends on last error: on existing, but
	// used port, we would get ERROR_ACCESS_DENIED or ERROR_SHARING_VIOLATION (or similar)
	if (h != INVALID_HANDLE_VALUE)
		CloseHandle(h);
	else if (error == ERROR_FILE_NOT_FOUND)
		return JNI_FALSE;
	return JNI_TRUE;
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    open
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_open
  (JNIEnv* env, jobject obj, jstring portID)
{
	HANDLE h = Open(env, portID, true, 0);
	if (h == INVALID_HANDLE_VALUE)
		ThrowException(env, QUERY_GETLASTERROR);
	else {
		jclass cls = env->GetObjectClass(obj);
		jfieldID fid = env->GetFieldID(cls, "fd", "J");
		env->SetLongField(obj, fid, reinterpret_cast<DWORD_PTR>(h));
#if defined DEBUG
		// set rx and tx queues (rx will usually keep some default buffer, 4K or similar...)
		SetupComm(h, 200, 0);
		COMMPROP cp = { 0 };
		if (GetCommProperties(h, &cp)) {
			wchar_t out[50];
			_snwprintf(out, 50, L"set queue tx %d rx %d", cp.dwCurrentTxQueue, cp.dwCurrentRxQueue);
			Trace(out);
		}
#endif
		if (!EscapeCommFunction(h, SETDTR)) {
			DWORD err = GetLastError();
			Close(env, obj);
			ThrowException(env, err);
		}
	}
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    close0
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_close0
  (JNIEnv* env, jobject obj)
{
	if (!Close(env, obj))
		ThrowException(env, QUERY_GETLASTERROR);
}

static const int BAUDRATE = io_calimero_serial_provider_jni_TtySerialCom_BAUDRATE;
static const int PARITY = io_calimero_serial_provider_jni_TtySerialCom_PARITY;
static const int DATABITS = io_calimero_serial_provider_jni_TtySerialCom_DATABITS;
static const int STOPBITS = io_calimero_serial_provider_jni_TtySerialCom_STOPBITS;
static const int FLOWCTRL = io_calimero_serial_provider_jni_TtySerialCom_FLOWCTRL;

#undef STOPBITS_10
#undef STOPBITS_15
#undef STOPBITS_20

static const int STOPBITS_10 = io_calimero_serial_provider_jni_TtySerialCom_ONE_STOPBIT;
static const int STOPBITS_15 = 3;
static const int STOPBITS_20 = io_calimero_serial_provider_jni_TtySerialCom_TWO_STOPBITS;

static const int FLOWCTRL_NONE = io_calimero_serial_provider_jni_TtySerialCom_FLOWCTRL_NONE;
static const int FLOWCTRL_CTSRTS = io_calimero_serial_provider_jni_TtySerialCom_FLOWCTRL_CTSRTS;

static BYTE GetPlatformStopBits(int stopbits)
{
	switch (stopbits) {
		case STOPBITS_10: return ONESTOPBIT;
		case STOPBITS_15: return ONE5STOPBITS;
		case STOPBITS_20: return TWOSTOPBITS;
	}
	return (BYTE) -1;
}

static int GetGenericStopBits(int stopbits)
{
	switch (stopbits) {
		case ONESTOPBIT: return STOPBITS_10;
		case ONE5STOPBITS: return STOPBITS_15;
		case TWOSTOPBITS: return STOPBITS_20;
	}
	return -1;
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    setControl
 * Signature: (II)I
 */
JNIEXPORT jint JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_setControl
  (JNIEnv* env, jobject obj, jint control, jint newValue)
{
	HANDLE h = GetFD(env, obj);
	DCB dcb = { 0 };
	dcb.DCBlength = sizeof(dcb);
	jint old = 0;
	if (!GetCommState(h, &dcb))
		ThrowException(env, QUERY_GETLASTERROR);
	else {
		if (control == BAUDRATE) {
			old = dcb.BaudRate;
			dcb.BaudRate = newValue;
		}
		else if (control == PARITY) {
			old = dcb.Parity;
			dcb.Parity = static_cast<BYTE>(newValue);
			dcb.fParity = newValue != NOPARITY;
			// dcb.fParity = TRUE;
		}
		else if (control == DATABITS) {
			old = dcb.ByteSize;
			dcb.ByteSize = static_cast<BYTE>(newValue);
		}
		else if (control == STOPBITS) {
			old = GetGenericStopBits(dcb.StopBits);
			dcb.StopBits = GetPlatformStopBits(newValue);
		}
		else if (control == FLOWCTRL) {
			// set defaults, disable SW flow control
			dcb.fOutxDsrFlow = FALSE;
			dcb.fDtrControl = DTR_CONTROL_DISABLE;
			dcb.fDsrSensitivity = FALSE;
			dcb.fTXContinueOnXoff = FALSE;
			dcb.fOutX = FALSE;
			dcb.fInX = FALSE;
			dcb.XonChar = 0;
			dcb.XoffChar = 0;
			if (newValue == FLOWCTRL_CTSRTS) {
				dcb.fOutxCtsFlow = TRUE;
				dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
			}
			else if (newValue == FLOWCTRL_NONE) {
				dcb.fOutxCtsFlow = FALSE;
				dcb.fRtsControl = RTS_CONTROL_DISABLE;
			}
			else
				Trace(L"flow control mode not supported");
		}
		dcb.fAbortOnError = FALSE;
		if (!SetCommState(h, &dcb))
			ThrowException(env, QUERY_GETLASTERROR);
	}
	Trace(L"set control");
	return old;
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    getControl
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_getControl
  (JNIEnv* env, jobject obj, jint control)
{
	HANDLE h = GetFD(env, obj);
	DCB dcb = { 0 };
	dcb.DCBlength = sizeof(dcb);
	BOOL ret = GetCommState(h, &dcb);
	Trace(L"get control");
	if (!ret)
		ThrowException(env, QUERY_GETLASTERROR);
	else if (control == BAUDRATE)
		return dcb.BaudRate;
	else if (control == PARITY)
		return dcb.Parity;
	else if (control == DATABITS)
		return dcb.ByteSize;
	else if (control == STOPBITS)
		return GetGenericStopBits(dcb.StopBits);
	return 0;
}

// call *immediately* after read/write
static void WaitPendingIO(JNIEnv* env, HANDLE h, OVERLAPPED* o, DWORD* transferred)
{
	Trace(L"wait pending I/O");
	// the only status tolerated is I/O pending
	if (GetLastError() != ERROR_IO_PENDING) {
		// some I/O problem, throw error
		ThrowException(env, QUERY_GETLASTERROR);
	}
	else {
		// wait for operation completion, and check result
		DWORD res = WaitForSingleObject(o->hEvent, INFINITE);
		if (res == WAIT_OBJECT_0 && GetOverlappedResult(h, o, transferred, FALSE)) {
				; // completed successfully
		}
		else {
			// communication or wait error
			ThrowException(env, QUERY_GETLASTERROR);
		}
	}
}

static int Write(JNIEnv* env, HANDLE h, PBYTE pBuf, long off, long len)
{
	Trace(L"start write");
	DWORD written = 0;
	OVERLAPPED o = { 0 };
	o.hEvent = CreateEvent(0, TRUE, FALSE, 0);
	if (o.hEvent == 0)
		ThrowException(env, QUERY_GETLASTERROR);
	else if (!WriteFile(h, pBuf + off, len, &written, &o))
		WaitPendingIO(env, h, &o, &written);
	FlushFileBuffers(h);
	Trace(L"end write");
	CloseHandle(o.hEvent);
	return written;
}


/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    writeBytes
 * Signature: ([BII)I
 */
JNIEXPORT jint JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_writeBytes
  (JNIEnv* env, jobject obj, jbyteArray buf, jint off, jint len)
{
	jboolean iscopy;
	return Write(env, GetFD(env, obj),
		reinterpret_cast<PBYTE>(env->GetByteArrayElements(buf, &iscopy)), off, len);
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    write
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_write
  (JNIEnv* env, jobject obj, jint byte)
{
	BYTE out = static_cast<BYTE>(byte);
	return Write(env, GetFD(env, obj), &out, 0, 1);
}

static int Read(JNIEnv* env, HANDLE h, PBYTE pBuf, long off, long len)
{
	DWORD read = 0;
	OVERLAPPED o = { 0 };
	o.hEvent = CreateEvent(0, TRUE, FALSE, 0);
	Trace(L"start read");
	if (o.hEvent == 0)
		ThrowException(env, QUERY_GETLASTERROR);
	else if (!ReadFile(h, pBuf + off, len, &read, &o))
		WaitPendingIO(env, h, &o, &read);

#if defined DEBUG
	wchar_t buf[256];
	buf[0] = 0;
	PBYTE pb = pBuf + off;
	wcscpy(buf, L"data:");
	for (unsigned i = 0; i < read; ++i)
		_snwprintf(buf + 5 + 3 * i, 4, L" %02X", *pb++);
	Trace(L"end read", buf);
#endif
	return read;
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    readBytes
 * Signature: ([BII)I
 */
JNIEXPORT jint JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_readBytes
  (JNIEnv* env, jobject obj, jbyteArray b, jint off, jint len)
{
	jboolean iscopy;
	jbyte* pBuf = env->GetByteArrayElements(b, &iscopy);
	int r = Read(env, GetFD(env, obj), reinterpret_cast<PBYTE>(pBuf), off, len);
	env->ReleaseByteArrayElements(b, pBuf, 0);
	return r;
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    read
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_read
  (JNIEnv* env, jobject obj)
{
	BYTE in = 0;
	if (Read(env, GetFD(env, obj), &in, 0, 1) == 0)
		return -1;
	return in;
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    setEvents
 * Signature: (IZ)V
 */
JNIEXPORT void JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_setEvents
  (JNIEnv* env, jobject obj, jint eventMask, jboolean enable)
{
	HANDLE h = GetFD(env, obj);
	DWORD mask;
	if (!GetCommMask(h, &mask))
		ThrowException(env, QUERY_GETLASTERROR);
	else {
		mask = enable ? mask | eventMask : mask & ~eventMask;
		if (!SetCommMask(h, mask))
			ThrowException(env, QUERY_GETLASTERROR);
	}
	Trace(L"set events");
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    waitEvent
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_waitEvent
  (JNIEnv* env, jobject obj)
{
	HANDLE h = GetFD(env, obj);
	DWORD event = 0;
	DWORD undefined;
	OVERLAPPED o = { 0 };
    o.hEvent = CreateEvent(0, TRUE, FALSE, 0);
    if (o.hEvent == 0)
        ThrowException(env, QUERY_GETLASTERROR);
	else if (!WaitCommEvent(h, &event, &o))
		WaitPendingIO(env, h, &o, &undefined);
	CloseHandle(o.hEvent);
	Trace(L"wait event");
	// note if event mask was changed while waiting for event we return 0
	return event;
}

static const int LINE_STATUS = io_calimero_serial_provider_jni_TtySerialCom_LINE_STATUS;
static const int ERROR_STATUS = io_calimero_serial_provider_jni_TtySerialCom_ERROR_STATUS;
static const int INPUT_AVAIL = io_calimero_serial_provider_jni_TtySerialCom_AVAILABLE_INPUT_STATUS;

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    getStatus
 * Signature: (I)I
 */
JNIEXPORT jint JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_getStatus
  (JNIEnv* env, jobject obj, jint type)
{
	HANDLE h = GetFD(env, obj);
	DWORD value = 0;
	BOOL ret = 0;
	if (type == LINE_STATUS) {
		ret = GetCommModemStatus(h, &value);
	}
	else if (type == INPUT_AVAIL) {
		COMSTAT stat = { 0 };
		ret = ClearCommError(h, &value, &stat);
		value = stat.cbInQue;
	}
	else if (type == ERROR_STATUS) {
		ret = ClearCommError(h, &value, 0);
	}
	Trace(L"get status");
	if (!ret)
		ThrowException(env, QUERY_GETLASTERROR);
	return value;
}

static int GetInt(JNIEnv* env, jobject obj, const char* name)
{
	return env->GetIntField(obj, env->GetFieldID(env->GetObjectClass(obj), name, "I"));
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    setTimeouts
 * Signature: (Lio/calimero/serial/provider/jni/TtySerialCom/Timeouts;)V
 */
JNIEXPORT void JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_setTimeouts
  (JNIEnv* env, jobject obj, jobject timeouts)
{
	COMMTIMEOUTS to;
	to.ReadIntervalTimeout = GetInt(env, timeouts, "readInterval");
	to.ReadTotalTimeoutMultiplier = GetInt(env, timeouts, "readTotalMultiplier");
	to.ReadTotalTimeoutConstant = GetInt(env, timeouts, "readTotalConstant");
	to.WriteTotalTimeoutMultiplier = GetInt(env, timeouts, "writeTotalMultiplier");
	to.WriteTotalTimeoutConstant = GetInt(env, timeouts, "writeTotalConstant");
	if (!SetCommTimeouts(GetFD(env, obj), &to))
		ThrowException(env, QUERY_GETLASTERROR);
}

/*
 * Class:     io_calimero_serial_provider_jni_TtySerialCom
 * Method:    getTimeouts
 * Signature: ()Lio/calimero/serial/provider/jni/TtySerialCom/Timeouts;
 */
JNIEXPORT jobject JNICALL Java_io_calimero_serial_provider_jni_TtySerialCom_getTimeouts
  (JNIEnv* env, jobject obj)
{
	COMMTIMEOUTS to;
	jobject timeouts = 0;
	if (!GetCommTimeouts(GetFD(env, obj), &to))
		ThrowException(env, QUERY_GETLASTERROR);
	else {
		jclass cls = env->FindClass("io/calimero/serial/provider/jni/TtySerialCom$Timeouts");
		jmethodID ctor = env->GetMethodID(cls, "<init>", "(IIIII)V");
		timeouts = env->NewObject(cls, ctor, to.ReadIntervalTimeout,
			to.ReadTotalTimeoutMultiplier, to.ReadTotalTimeoutConstant,
			to.WriteTotalTimeoutMultiplier, to.WriteTotalTimeoutConstant);
		if (timeouts == 0)
			ThrowException(env, 0);
	}
	Trace(L"get timeouts");
	return timeouts;
}

#ifdef __cplusplus
}
#endif
