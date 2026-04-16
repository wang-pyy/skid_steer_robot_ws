#pragma once

#include <atomic>
#include <mutex>
#include <string>

/**
 * Minimal POSIX serial-port wrapper.
 *
 * - Blocking read with configurable VMIN/VTIME.
 * - Thread-safe writes (internal mutex).
 * - Line-oriented helper read_until() that returns the next frame delimited
 *   by start ('$') and end ('#') markers.
 */
class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    /** Open @p device at @p baudrate (8N1, no flow control). */
    bool open(const std::string& device, int baudrate);
    void close();
    bool is_open() const { return fd_ >= 0; }

    /** Write raw bytes. Returns true on full success. */
    bool write_all(const std::string& data);

    /**
     * Read the next full frame starting with '$' and ending with '#'.
     * Any bytes before a '$' are discarded (resync).
     * @param out    Frame contents excluding '$' and '#'.
     * @return true if a frame was read, false on timeout/error/close.
     */
    bool read_frame(std::string& out);

private:
    int fd_{-1};
    std::mutex write_mtx_;
    std::atomic<bool> open_{false};
};
