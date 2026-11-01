#include "MjpegServer.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <string_view>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

constexpr int kMaxClients = 4;
constexpr char kBoundary[] = "frame";
constexpr char kIndexHtml[] = R"HTML(<!doctype html>
<html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Hikrobot Camera</title>
<style>html,body{margin:0;height:100%;background:#101010;overflow:hidden}
body{display:grid;place-items:center}img{max-width:100vw;max-height:100vh}
#s{position:fixed;left:0;top:0;padding:6px 10px;color:#ddd;background:#0009;font:13px system-ui}</style>
<img id="v" src="/stream" alt="camera stream"><div id="s">connecting...</div>
<script>const v=document.getElementById('v'),s=document.getElementById('s');
function ok(){s.textContent=v.naturalWidth?'live '+v.naturalWidth+'脳'+v.naturalHeight:'connecting...'}
v.onload=ok;v.onerror=()=>{s.textContent='connection lost - retrying';setTimeout(()=>v.src='/stream?'+Date.now(),1000)};
setInterval(ok,1000)</script>)HTML";

bool validFrame(const cv::Mat& frame) {
    return !frame.empty() && (frame.type() == CV_8UC1 || frame.type() == CV_8UC3);
}

}  // namespace

bool StreamConfig::load(const cv::FileNode& node) {
    if (node.empty()) return true;

    int enabledValue = enabled ? 1 : 0;
    if (!node["enabled"].empty()) node["enabled"] >> enabledValue;
    if (!node["port"].empty()) node["port"] >> port;
    if (!node["width"].empty()) node["width"] >> width;
    if (!node["fps"].empty()) node["fps"] >> fps;
    if (!node["quality"].empty()) node["quality"] >> quality;

    if ((enabledValue != 0 && enabledValue != 1) || port <= 0 || port > 65535 ||
        width < 0 || fps <= 0.0 || quality < 1 || quality > 100) {
        std::cerr << "Invalid value in stream config" << std::endl;
        return false;
    }
    enabled = enabledValue == 1;
    return true;
}

MjpegServer::MjpegServer(StreamConfig config) : config_(std::move(config)) {}

MjpegServer::~MjpegServer() { stop(); }

int MjpegServer::createListenSocket(int port) {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    int reuse = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 ||
        ::listen(fd, kMaxClients) < 0) {
        ::close(fd);
        return -1;
    }
    return fd;
}

bool MjpegServer::sendAll(int fd, const void* data, std::size_t size) {
    const char* cursor = static_cast<const char*>(data);
    while (size > 0) {
        const ssize_t written = ::send(fd, cursor, size, MSG_NOSIGNAL);
        if (written < 0 && errno == EINTR) continue;
        if (written <= 0) return false;
        cursor += written;
        size -= static_cast<std::size_t>(written);
    }
    return true;
}

std::string MjpegServer::requestPath(int fd) {
    char buffer[2048];
    const ssize_t count = ::recv(fd, buffer, sizeof(buffer) - 1, 0);
    if (count <= 0) return {};
    buffer[count] = '\0';
    const std::string_view request(buffer, static_cast<std::size_t>(count));
    const std::size_t start = request.find(' ');
    if (start == std::string_view::npos) return {};
    const std::size_t end = request.find(' ', start + 1);
    return std::string(request.substr(start + 1, end - start - 1));
}

bool MjpegServer::sendPage(int fd) {
    const std::string_view body(kIndexHtml);
    const std::string header =
        std::string("HTTP/1.1 200 OK\\r\\nContent-Type: text/html; charset=utf-8\\r\\n") +
        "Content-Length: " + std::to_string(body.size()) +
        "\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n";
    return sendAll(fd, header.data(), header.size()) &&
           sendAll(fd, body.data(), body.size());
}

bool MjpegServer::sendNotFound(int fd) {
    constexpr std::string_view response =
        "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
    return sendAll(fd, response.data(), response.size());
}

bool MjpegServer::start() {
    if (running()) return true;

    listenFd_ = createListenSocket(config_.port);
    if (listenFd_ < 0) {
        std::cerr << "[STREAM] Failed to listen on port " << config_.port << ": "
                  << std::strerror(errno) << std::endl;
        return false;
    }

    stopping_.store(false, std::memory_order_relaxed);
    running_.store(true, std::memory_order_release);
    acceptThread_ = std::thread(&MjpegServer::acceptLoop, this);
    encodeThread_ = std::thread(&MjpegServer::encodeLoop, this);
    std::cout << "[STREAM] http://localhost:" << config_.port << "/ ("
              << config_.fps << " fps, "
              << (config_.width > 0 ? std::to_string(config_.width) + "px" : "native")
              << ", q=" << config_.quality << ')' << std::endl;
    return true;
}

void MjpegServer::stop() {
    if (!running_.exchange(false, std::memory_order_acq_rel)) return;
    stopping_.store(true, std::memory_order_release);
    pendingCv_.notify_all();
    frameCv_.notify_all();

    if (listenFd_ >= 0) {
        ::shutdown(listenFd_, SHUT_RDWR);
        ::close(listenFd_);
    }
    {
        std::lock_guard<std::mutex> lock(clientMutex_);
        for (const int fd : clientFds_) ::shutdown(fd, SHUT_RDWR);
    }

    if (acceptThread_.joinable()) acceptThread_.join();
    if (encodeThread_.joinable()) encodeThread_.join();
    for (auto& thread : clientThreads_) {
        if (thread.joinable()) thread.join();
    }
    clientThreads_.clear();
    clientFds_.clear();
    listenFd_ = -1;
}

void MjpegServer::submit(const cv::Mat& frame) {
    if (!running() || !validFrame(frame)) return;
    const auto now = Clock::now();
    if (now < nextSubmit_) return;
    nextSubmit_ = now + std::chrono::duration_cast<Clock::duration>(
                              std::chrono::duration<double>(1.0 / config_.fps));

    cv::Mat copy = frame.clone();
    {
        std::lock_guard<std::mutex> lock(pendingMutex_);
        pendingFrame_ = std::move(copy);
        ++pendingSequence_;
    }
    pendingCv_.notify_one();
}

void MjpegServer::encodeLoop() {
    cv::Mat frame;
    while (running()) {
        {
            std::unique_lock<std::mutex> lock(pendingMutex_);
            pendingCv_.wait(lock, [this] {
                return pendingSequence_ != consumedSequence_ || !running();
            });
            if (!running()) break;
            consumedSequence_ = pendingSequence_;
            frame = std::move(pendingFrame_);
        }

        std::vector<unsigned char> jpeg;
        if (!frame.empty() && encode(frame, jpeg)) {
            std::lock_guard<std::mutex> lock(frameMutex_);
            latestJpeg_ = std::move(jpeg);
            ++frameSequence_;
            frameCv_.notify_all();
        }
    }
}

bool MjpegServer::encode(const cv::Mat& frame, std::vector<unsigned char>& jpeg) const {
    cv::Mat source = frame;
    cv::Mat scaled;
    if (config_.width > 0 && frame.cols > config_.width) {
        const double scale = static_cast<double>(config_.width) / frame.cols;
        const int height = std::max(1, static_cast<int>(frame.rows * scale + .5));
        cv::resize(frame, scaled, cv::Size(config_.width, height), 0, 0, cv::INTER_AREA);
        source = scaled;
    }
    return cv::imencode(".jpg", source, jpeg,
                        {cv::IMWRITE_JPEG_QUALITY, config_.quality});
}

bool MjpegServer::nextFrame(std::uint64_t& sequence,
                            std::vector<unsigned char>& jpeg) {
    std::unique_lock<std::mutex> lock(frameMutex_);
    frameCv_.wait_for(lock, std::chrono::seconds(1), [this, sequence] {
        return frameSequence_ != sequence || !running();
    });
    if (!running()) return false;
    if (frameSequence_ == sequence) {
        jpeg.clear();
        return true;
    }
    sequence = frameSequence_;
    jpeg = latestJpeg_;
    return true;
}

void MjpegServer::acceptLoop() {
    while (running()) {
        sockaddr_in address{};
        socklen_t length = sizeof(address);
        const int fd = ::accept(listenFd_, reinterpret_cast<sockaddr*>(&address), &length);
        if (fd < 0) {
            if (running() && errno != EINTR) continue;
            break;
        }
        if (clients_.load(std::memory_order_relaxed) >= kMaxClients) {
            ::close(fd);
            continue;
        }
        clients_.fetch_add(1, std::memory_order_relaxed);
        {
            std::lock_guard<std::mutex> lock(clientMutex_);
            clientFds_.push_back(fd);
            clientThreads_.emplace_back(&MjpegServer::clientLoop, this, fd);
        }
    }
}

void MjpegServer::closeClient(int fd) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
    std::lock_guard<std::mutex> lock(clientMutex_);
    const auto it = std::find(clientFds_.begin(), clientFds_.end(), fd);
    if (it != clientFds_.end()) clientFds_.erase(it);
    clients_.fetch_sub(1, std::memory_order_relaxed);
}

void MjpegServer::clientLoop(int fd) {
    const std::string path = requestPath(fd);
    const bool stream = path.rfind("/stream", 0) == 0;
    const bool page = path.empty() || path == "/" || path == "/index.html" ||
                      path.rfind("/?", 0) == 0;
    if (!stream) {
        if (page) sendPage(fd);
        else sendNotFound(fd);
        closeClient(fd);
        return;
    }

    const std::string header =
        "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=" +
        std::string(kBoundary) +
        "\r\nCache-Control: no-cache, no-store, must-revalidate\r\n"
        "Pragma: no-cache\r\nConnection: close\r\n\r\n";
    if (!sendAll(fd, header.data(), header.size())) {
        closeClient(fd);
        return;
    }

    std::uint64_t sequence = 0;
    std::vector<unsigned char> jpeg;
    while (running() && !stopping_.load(std::memory_order_relaxed)) {
        if (!nextFrame(sequence, jpeg)) break;
        if (jpeg.empty()) continue;
        const std::string part =
            "--" + std::string(kBoundary) + "\r\nContent-Type: image/jpeg\r\n" +
            "Content-Length: " + std::to_string(jpeg.size()) + "\r\n\r\n";
        if (!sendAll(fd, part.data(), part.size()) ||
            !sendAll(fd, jpeg.data(), jpeg.size()) || !sendAll(fd, "\r\n", 2)) {
            break;
        }
    }
    closeClient(fd);
}
