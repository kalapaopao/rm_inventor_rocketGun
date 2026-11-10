#include "SerialOutput.hpp"
#include "SerialOverlay.hpp"
#include <cerrno>
#include <cstdlib>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool value, const char* text) { if (!value) throw std::runtime_error(text); }
bool receive(int fd, vision::YawPacket& packet, int timeoutMs) {
    size_t offset=0;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now()<deadline) {
        const auto n=read(fd,packet.data()+offset,packet.size()-offset);
        if(n>0) { offset+=static_cast<size_t>(n); if(offset==packet.size()) return true; }
        pollfd descriptor{fd,POLLIN,0}; poll(&descriptor,1,5);
    }
    return false;
}
int main() {
    int master=-1;
    try {
        using namespace vision;
        require(encodeYaw(123)==YawPacket{255,123,0,0,0,0,0,13}, "positive bytes");
        require(encodeYaw(-123)==YawPacket{255,133,255,0,0,0,0,13}, "negative bytes");
        require(encodeYaw(0)==YawPacket{255,0,0,0,0,0,0,13}, "zero bytes");
        VisionResult result;
        result.captureTimestamp=std::chrono::steady_clock::now();
        result.yawOffsetValid=true; result.yawOffsetPixels=12.6;
        std::int16_t yaw;
        require(selectYaw(result,result.captureTimestamp,100,yaw) && yaw==13,"rounding");
        result.yawOffsetPixels=1e9;
        require(selectYaw(result,result.captureTimestamp,100,yaw) && yaw==32767,"positive saturation");
        result.yawOffsetPixels=-1e9;
        require(selectYaw(result,result.captureTimestamp,100,yaw) && yaw==-32768,"negative saturation");
        result.yawOffsetPixels=std::numeric_limits<double>::quiet_NaN();
        require(!selectYaw(result,result.captureTimestamp,100,yaw),"NaN allowed");
        result.yawOffsetPixels=0;
        require(!selectYaw(result,result.captureTimestamp+std::chrono::milliseconds(101),100,yaw),"stale allowed");
        require(!selectYaw(result,result.captureTimestamp-std::chrono::milliseconds(1),100,yaw),"future timestamp");
        cv::Mat image(240,640,CV_8UC3,cv::Scalar());
        drawSerialPanel(image,result,false,100,{});
        require(cv::countNonZero(image.reshape(1))>0,"overlay missing");

        master=posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK);
        require(master>=0 && grantpt(master)==0 && unlockpt(master)==0,"PTY creation");
        SerialConfig config; config.port=ptsname(master); config.periodMs=10; config.maxAgeMs=100;
        std::mutex mutex;
        result.captureTimestamp=std::chrono::steady_clock::now(); result.yawOffsetPixels=-123;
        SerialOutput serial;
        require(serial.start(config,[&](VisionResult& out){
            std::lock_guard<std::mutex> lock(mutex); out=result; return true;
        }),"PTY sender start");
        YawPacket frame{};
        require(receive(master,frame,100) && frame==encodeYaw(-123),"PTY wire bytes");
        // Drain any pre-expiry bytes before checking that the old result stops TX.
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        unsigned char drain[256]; while(read(master,drain,sizeof(drain))>0) {}
        require(!receive(master,frame,50),"stale result kept sending");
        {
            std::lock_guard<std::mutex> lock(mutex);
            result.captureTimestamp=std::chrono::steady_clock::now(); result.yawOffsetValid=false;
        }
        require(!receive(master,frame,40),"invalid result sent zero");
        {
            std::lock_guard<std::mutex> lock(mutex);
            result.captureTimestamp=std::chrono::steady_clock::now();
            result.yawOffsetValid=true; result.yawOffsetPixels=0;
        }
        require(receive(master,frame,100) && frame==encodeYaw(0),"valid zero not sent");
        serial.stop();
        const auto status=serial.status();
        require(status.hasLastSent && status.lastYaw==0 && status.sent>=2 && !status.running,"TX status");
        // Restart, then disconnect the simulated device. Sender must stop rather
        // than spin or keep claiming successful sends.
        {
            std::lock_guard<std::mutex> lock(mutex);
            result.captureTimestamp=std::chrono::steady_clock::now();
        }
        require(serial.start(config,[&](VisionResult& out){
            std::lock_guard<std::mutex> lock(mutex); out=result; return true;
        }),"restart failed");
        require(receive(master,frame,100),"restart did not transmit");
        close(master); master=-1;
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(200);
        while(serial.running() && std::chrono::steady_clock::now()<deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        require(!serial.running() && serial.status().errors>0,"disconnect not detected");
        serial.stop();
        config.baud=12345;
        require(!serial.start(config,[](VisionResult&){return false;}),"unsupported baud accepted");
        std::cout << "PASS: yaw bytes/rounding/saturation, stale/invalid guards, overlay, PTY TX, status, stop, baud\n";
        return 0;
    } catch (const std::exception& e) {
        if(master>=0) close(master);
        std::cerr<<e.what()<<'\n'; return 1;
    }
}
