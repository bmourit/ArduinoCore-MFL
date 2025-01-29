//
// MFL stubs to surpress compiler warnings
//
// Copyright (C) 2025 B. Mouritsen <bnmguy@gmail.com>
//
// This file is part of the Microcontroller Firmware Library (MFL).
//
// MFL is free software: you can redistribute it and/or modify it under the terms of the
// GNU Lesser General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// MFL is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License along with MFL.
// If not, see <https://www.gnu.org/licenses/>.
//

#if defined(__GNUC__)
    #include <sys/stat.h>
#endif

#include <errno.h>

// Declare external errno if not defined
#undef errno
extern int errno;

extern "C" {

// Weak attribute for GCC compatibility
__attribute__((weak)) int _close(int file __attribute__((unused))) {
    return -1; // Indicate failure
}

__attribute__((weak)) int _fstat(int file __attribute__((unused)), struct stat* st) {
    if (st) {
        st->st_mode = S_IFCHR; // Set mode to character device
        return 0; // Success
    }
    errno = EFAULT; // Set error for invalid pointer
    return -1;
}

__attribute__((weak)) int _isatty(int file __attribute__((unused))) {
    return 1; // Always indicate it's a terminal
}

__attribute__((weak)) int _lseek(int file __attribute__((unused)), 
                                 int ptr __attribute__((unused)), 
                                 int dir __attribute__((unused))) {
    return 0; // No repositioning support
}


__attribute__((weak)) int _write(int file __attribute__((unused)), 
                                 char* ptr __attribute__((unused)), 
                                 int len __attribute__((unused))) {
    return 0; // No output support
}

__attribute__((weak)) void _exit(int status __attribute__((unused))) {
    while (1) {
        // Infinite loop to prevent returning
    }
}

__attribute__((weak)) int _kill(int pid __attribute__((unused)), 
                                int sig __attribute__((unused))) {
    errno = EINVAL; // Invalid argument
    return -1; // Failure
}

__attribute__((weak)) int _getpid(void) {
    return 1; // Return a dummy process ID
}


int _close_r(struct _reent* reent, int file) {
    return _close(file);
}

int _fstat_r(struct _reent* reent, int file, struct stat* st) {
    return _fstat(file, st);
}

int _isatty_r(struct _reent* reent, int file) {
    return _isatty(file);
}

off_t _lseek_r(struct _reent* reent, int file, off_t ptr, int dir) {
    return _lseek(file, ptr, dir);
}

int _write_r(struct _reent* reent, int file, const char* ptr, int len) {
    return _write(file, const_cast<char*>(ptr), len);
}

int _read_r(struct _reent* reent, int file, char* ptr, int len) {
    errno = ENOSYS; // Not implemented
    return -1;
}

int _kill_r(struct _reent* reent, int pid, int sig) {
    return _kill(pid, sig);
}

int _getpid_r(struct _reent* reent) {
    return _getpid();
}

} // extern "C"
