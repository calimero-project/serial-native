//#define _POSIX_C_SOURCE 200809L

#include <unistd.h> // write
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <string.h> // strerror
#include <errno.h>
#include <sys/select.h>
#include <signal.h> // kill
