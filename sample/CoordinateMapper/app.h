#ifndef __APP__
#define __APP__

#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>

#include <vector>

#include <wrl/client.h>
#include "threadSafeQueue.hpp"
#include <thread>
using namespace Microsoft::WRL;

class Kinect
{
private:
    // Sensor
    ComPtr<IKinectSensor> kinect;

    // Coordinate Mapper
    ComPtr<ICoordinateMapper> coordinateMapper;

    // Reader
    ComPtr<IColorFrameReader> colorFrameReader;
    ComPtr<IDepthFrameReader> depthFrameReader;

    // Color Buffer
    std::vector<BYTE> colorBuffer;
    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;
    cv::Mat colorMat;

    // Depth Buffer
    std::vector<UINT16> depthBuffer;
    int depthWidth;
    int depthHeight;
    unsigned int depthBytesPerPixel;
    cv::Mat depthMat;

bool mRunning{true};
sim::ThreadSafeQ<cv::Mat> mQ;
std::thread mWriterThread;

public:
    // Constructor
    Kinect();

    // Destructor
    ~Kinect();

    // Processing
    void run();

private:
    // Initialize
    void initialize();

    // Initialize Sensor
    inline void initializeSensor();

    // Initialize Color
    inline void initializeColor();

    // Initialize Depth
    inline void initializeDepth();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Color
    inline void updateColor();

    // Update Depth
    inline void updateDepth();

    // Draw Data
    void draw();

    // Draw Color
    inline void drawColor();

    // Draw Depth
    inline void drawDepth();

    // Show Data
    void show();

    // Show Color
    inline void showColor();

    // Show Depth
    inline void showDepth();
};

#endif // __APP__