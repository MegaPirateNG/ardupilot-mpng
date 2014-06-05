#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_F4BY

#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <ctype.h>

#include "Storage.h"
using namespace F4BY;

/*
  This stores eeprom data in the F4BY MTD interface with a 4k size, and
  a in-memory buffer. This keeps the latency and devices IOs down.
 */

// name the storage file after the sketch so you can use the same sd
// card for ArduCopter and ArduPlane
#define STORAGE_DIR "/fs/microsd/APM"
#define OLD_STORAGE_FILE STORAGE_DIR "/" SKETCHNAME ".stg"
#define OLD_STORAGE_FILE_BAK STORAGE_DIR "/" SKETCHNAME ".bak"
#define MTD_PARAMS_FILE "/fs/mtd"
#define SMART_MTD_PARAMS_FILE "/mnt/smart/mtd"
#define MTD_SIGNATURE 0x14012014
#define MTD_SIGNATURE_OFFSET (8192-4)
#define STORAGE_RENAME_OLD_FILE 0

extern const AP_HAL::HAL& hal;

F4BYStorage::F4BYStorage(void) :
    _fd(-1),
    _dirty_mask(0),
    _perf_storage(perf_alloc(PC_ELAPSED, "APM_storage")),
    _perf_errors(perf_alloc(PC_COUNT, "APM_storage_errors")),
    _mtd_file_path(MTD_PARAMS_FILE)
{
}

/*
  put signature bytes at offset MTD_SIGNATURE_OFFSET
 */
void F4BYStorage::_mtd_write_signature(void)
{
    int mtd_fd = open(_mtd_file_path, O_WRONLY);
    if (mtd_fd == -1) {
        hal.scheduler->panic("Failed to open " MTD_PARAMS_FILE);
    }
    uint32_t v = MTD_SIGNATURE;
    if (lseek(mtd_fd, MTD_SIGNATURE_OFFSET, SEEK_SET) != MTD_SIGNATURE_OFFSET) {
        hal.scheduler->panic("Failed to seek in " MTD_PARAMS_FILE);
    }
    if (write(mtd_fd, &v, sizeof(v)) != sizeof(v)) {
        hal.scheduler->panic("Failed to write signature in " MTD_PARAMS_FILE);
    }
    close(mtd_fd);
}

void F4BYStorage::_storage_open(void)
{
	if (_initialised) {
		return;
	}

	struct stat st;
	bool have_mtd = (stat(_mtd_file_path, &st) == 0);

	if (!have_mtd) {
		//trying m25px
		_mtd_file_path = SMART_MTD_PARAMS_FILE;
		
		int smart_fd = open(_mtd_file_path, O_RDONLY);
		if (smart_fd == -1) {
			smart_fd = open(_mtd_file_path, O_WRONLY | O_CREAT);
			if(smart_fd != -1) {
				memset(_buffer, 0, sizeof(_buffer));
				write(smart_fd, _buffer, sizeof(_buffer));
				fsync(smart_fd);
				close(smart_fd);
			}
			else {
				hal.scheduler->panic("Failed to create " SMART_MTD_PARAMS_FILE);
			}
		} else {
			read(smart_fd, _buffer, sizeof(_buffer)) != sizeof(_buffer);
			close(smart_fd);
		}
	}
	else
	{
		// we write the signature every time, even if it already is
		// good, as this gives us a way to detect if the MTD device is
		// functional. It is better to panic now than to fail to save
		// parameters in flight
		_mtd_write_signature();
		_dirty_mask = 0;
		int fd = open(_mtd_file_path, O_RDONLY);
		if (fd == -1) {
				hal.scheduler->panic("Failed to open " MTD_PARAMS_FILE);
		}
		if (read(fd, _buffer, sizeof(_buffer)) != sizeof(_buffer)) {
				hal.scheduler->panic("Failed to read " MTD_PARAMS_FILE);
		}
		close(fd);
	}

	_initialised = true;
}

/*
  mark some lines as dirty. Note that there is no attempt to avoid
  the race condition between this code and the _timer_tick() code
  below, which both update _dirty_mask. If we lose the race then the
  result is that a line is written more than once, but it won't result
  in a line not being written.
 */
void F4BYStorage::_mark_dirty(uint16_t loc, uint16_t length)
{
	uint16_t end = loc + length;
	while (loc < end) {
		uint8_t line = (loc >> F4BY_STORAGE_LINE_SHIFT);
		_dirty_mask |= 1 << line;
		loc += F4BY_STORAGE_LINE_SIZE;
	}
}

uint8_t F4BYStorage::read_byte(uint16_t loc) 
{
	if (loc >= sizeof(_buffer)) {
		return 0;
	}
	_storage_open();
	return _buffer[loc];
}

uint16_t F4BYStorage::read_word(uint16_t loc) 
{
	uint16_t value;
	if (loc >= sizeof(_buffer)-(sizeof(value)-1)) {
		return 0;
	}
	_storage_open();
	memcpy(&value, &_buffer[loc], sizeof(value));
	return value;
}

uint32_t F4BYStorage::read_dword(uint16_t loc) 
{
	uint32_t value;
	if (loc >= sizeof(_buffer)-(sizeof(value)-1)) {
		return 0;
	}
	_storage_open();
	memcpy(&value, &_buffer[loc], sizeof(value));
	return value;
}

void F4BYStorage::read_block(void *dst, uint16_t loc, size_t n) 
{
	if (loc >= sizeof(_buffer)-(n-1)) {
		return;
	}
	_storage_open();
	memcpy(dst, &_buffer[loc], n);
}

void F4BYStorage::write_byte(uint16_t loc, uint8_t value) 
{
	if (loc >= sizeof(_buffer)) {
		return;
	}
	if (_buffer[loc] != value) {
		_storage_open();
		_buffer[loc] = value;
		_mark_dirty(loc, sizeof(value));
	}
}

void F4BYStorage::write_word(uint16_t loc, uint16_t value) 
{
	if (loc >= sizeof(_buffer)-(sizeof(value)-1)) {
		return;
	}
	if (memcmp(&value, &_buffer[loc], sizeof(value)) != 0) {
		_storage_open();
		memcpy(&_buffer[loc], &value, sizeof(value));
		_mark_dirty(loc, sizeof(value));
	}
}

void F4BYStorage::write_dword(uint16_t loc, uint32_t value) 
{
	if (loc >= sizeof(_buffer)-(sizeof(value)-1)) {
		return;
	}
	if (memcmp(&value, &_buffer[loc], sizeof(value)) != 0) {
		_storage_open();
		memcpy(&_buffer[loc], &value, sizeof(value));
		_mark_dirty(loc, sizeof(value));
	}
}

void F4BYStorage::write_block(uint16_t loc, const void *src, size_t n) 
{
	if (loc >= sizeof(_buffer)-(n-1)) {
		return;
	}
	if (memcmp(src, &_buffer[loc], n) != 0) {
		_storage_open();
		memcpy(&_buffer[loc], src, n);
		_mark_dirty(loc, n);
	}
}

void F4BYStorage::_timer_tick(void)
{
	if (!_initialised || _dirty_mask == 0) {
		return;
	}
	perf_begin(_perf_storage);

	if (_fd == -1) {
		_fd = open(_mtd_file_path, O_WRONLY);
		if (_fd == -1) {
			perf_end(_perf_storage);
			perf_count(_perf_errors);
			return;	
		}
	}

	// write out the first dirty set of lines. We don't write more
	// than one to keep the latency of this call to a minimum
	uint8_t i, n;
	for (i=0; i<F4BY_STORAGE_NUM_LINES; i++) {
		if (_dirty_mask & (1<<i)) {
			break;
		}
	}
	if (i == F4BY_STORAGE_NUM_LINES) {
		// this shouldn't be possible
		perf_end(_perf_storage);
		perf_count(_perf_errors);
		return;
	}
	uint32_t write_mask = (1U<<i);
	// see how many lines to write
	for (n=1; (i+n) < F4BY_STORAGE_NUM_LINES && 
		     n < (F4BY_STORAGE_MAX_WRITE>>F4BY_STORAGE_LINE_SHIFT); n++) {
		if (!(_dirty_mask & (1<<(n+i)))) {
			break;
		}		
		// mark that line clean
		write_mask |= (1<<(n+i));
	}

	/*
	  write the lines. This also updates _dirty_mask. Note that
	  because this is a SCHED_FIFO thread it will not be preempted
	  by the main task except during blocking calls. This means we
	  don't need a semaphore around the _dirty_mask updates.
	 */
	if (lseek(_fd, i<<F4BY_STORAGE_LINE_SHIFT, SEEK_SET) == (i<<F4BY_STORAGE_LINE_SHIFT)) {
		_dirty_mask &= ~write_mask;
		if (write(_fd, &_buffer[i<<F4BY_STORAGE_LINE_SHIFT], n<<F4BY_STORAGE_LINE_SHIFT) != n<<F4BY_STORAGE_LINE_SHIFT) {
			// write error - likely EINTR
			_dirty_mask |= write_mask;
			close(_fd);
			_fd = -1;
			perf_count(_perf_errors);
		}
		if (_dirty_mask == 0) {
			if (fsync(_fd) != 0) {
				close(_fd);
				_fd = -1;
				perf_count(_perf_errors);
			}
		}
	}
	perf_end(_perf_storage);
}

#endif // CONFIG_HAL_BOARD
