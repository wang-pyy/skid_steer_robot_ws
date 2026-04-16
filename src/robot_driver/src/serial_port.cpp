#include "robot_driver/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace {

speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B0;
    }
}

}  // namespace

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& device, int baudrate) {
    close();

    int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0) return false;

    struct termios tio{};
    if (tcgetattr(fd, &tio) != 0) { ::close(fd); return false; }

    speed_t spd = baud_to_speed(baudrate);
    if (spd == B0) { ::close(fd); return false; }
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    // 8N1, no flow control, raw mode
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag |= (CLOCAL | CREAD);

    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG | IEXTEN);
    tio.c_oflag &= ~OPOST;

    // Blocking read: at least 1 byte, 0.5 s inter-byte timeout.
    tio.c_cc[VMIN]  = 1;
    tio.c_cc[VTIME] = 5;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) { ::close(fd); return false; }
    tcflush(fd, TCIOFLUSH);

    fd_ = fd;
    open_ = true;
    return true;
}

void SerialPort::close() {
    open_ = false;
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::write_all(const std::string& data) {
    if (fd_ < 0) return false;
    std::lock_guard<std::mutex> lock(write_mtx_);
    const char* p = data.data();
    size_t remaining = data.size();
    while (remaining > 0) {
        ssize_t n = ::write(fd_, p, remaining);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        p += n;
        remaining -= static_cast<size_t>(n);
    }
    return true;
}

bool SerialPort::read_frame(std::string& out) {
    if (fd_ < 0) return false;

    out.clear();
    bool started = false;

    while (open_.load()) {
        char c;
        ssize_t n = ::read(fd_, &c, 1);
        if (n == 0) return false;                 // timeout (VTIME fired with no data since VMIN)
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }

        if (!started) {
            if (c == '$') started = true;         // drop noise until next frame start
            continue;
        }

        if (c == '#') return true;                // complete frame
        if (c == '$') { out.clear(); continue; }  // resync on stray start
        out.push_back(c);

        if (out.size() > 512) { out.clear(); started = false; }  // runaway frame guard
    }
    return false;
}
