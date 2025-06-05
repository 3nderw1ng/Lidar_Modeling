// pylidar.cpp
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <numpy/arrayobject.h>
#include <WinSock2.h>
#include <thread>
#include <mutex>
#include <vector>
#include <cmath>
#include <atomic>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#pragma comment(lib, "WS2_32.lib")

// 点结构
struct LiDarPoint {
    float x, y, z;
    uint8_t intensity;
    uint64_t timestamp;
    uint8_t channel;
};

std::vector<LiDarPoint> latest_frame;
std::mutex frame_mutex;
std::atomic<bool> running(true);

float ChList[16] = { -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15 };
float offsetV[16] = { 5.06f, -9.15f, 5.06f, -9.15f, 5.06f, -9.15f, 5.06f, -9.15f,
                      9.15f, -5.06f, 9.15f, -5.06f, 9.15f, -5.06f, 9.15f, -5.06f };
float offsetH[16] = { 21, 21, 21, 21, 21, 21, 21, 21, -21, -21, -21, -21, -21, -21, -21, -21 };

struct DataPoint { uint16_t distance; uint8_t intensity; };
struct DataBlock {
    uint8_t identifier[2];
    uint16_t azimuth;
    DataPoint points[32];
};
struct RawPacket {
    DataBlock blocks[12];
    uint32_t timestamp;
    uint8_t factory[2];
};

void AddPoint(std::vector<LiDarPoint>& frame, float r, float ang, uint8_t intensity, uint8_t ch, uint64_t ts) {
    float _ang = (ang + ch * 0.00108f * 10.0f) * M_PI / 180.0f;
    float _w = ChList[ch] * M_PI / 180.0f;
    float x = r * cos(_w) * cos(_ang) + offsetH[ch] * cos(_ang) * 0.001f;
    float y = r * cos(_w) * sin(_ang) - offsetH[ch] * sin(_ang) * 0.001f;
    float z = r * sin(_w) + offsetV[ch] * 0.001f;
    frame.push_back({ x, y, z, intensity, ts, ch });
}

void recv_loop() {
    WSADATA wsd;
    WSAStartup(MAKEWORD(2,2), &wsd);
    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    SOCKADDR_IN addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(2368);
    addr.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
    bind(s, (SOCKADDR*)&addr, sizeof(addr));

    char buf[1206];
    SOCKADDR_IN from;
    int fromlen = sizeof(from);

    while (running) {
        int n = recvfrom(s, buf, 1206, 0, (SOCKADDR*)&from, &fromlen);
        if (n == 1206) {
            RawPacket* packet = (RawPacket*)buf;
            std::vector<LiDarPoint> frame;
            uint64_t ts = packet->timestamp;
            for (int i = 0; i < 12; ++i) {
                float angle = packet->blocks[i].azimuth * 0.01f;
                for (int j = 0; j < 16; ++j) {
                    float r = packet->blocks[i].points[j].distance * 0.002f;
                    uint8_t inten = packet->blocks[i].points[j].intensity;
                    AddPoint(frame, r, angle, inten, j, ts);
                }
                for (int j = 0; j < 16; ++j) {
                    float r = packet->blocks[i].points[j + 16].distance * 0.002f;
                    uint8_t inten = packet->blocks[i].points[j + 16].intensity;
                    AddPoint(frame, r, angle + 0.18f, inten, j, ts);
                }
            }
            std::lock_guard<std::mutex> lock(frame_mutex);
            latest_frame = std::move(frame);
        }
    }

    closesocket(s);
    WSACleanup();
}

static PyObject* get_latest_frame(PyObject*, PyObject*) {
    std::lock_guard<std::mutex> lock(frame_mutex);
    npy_intp dims[2] = { (npy_intp)latest_frame.size(), 6 };
    PyObject* array = PyArray_SimpleNew(2, dims, NPY_FLOAT32);
    float* data = (float*)PyArray_DATA((PyArrayObject*)array);
    for (size_t i = 0; i < latest_frame.size(); ++i) {
        data[i * 6 + 0] = latest_frame[i].x;
        data[i * 6 + 1] = latest_frame[i].y;
        data[i * 6 + 2] = latest_frame[i].z;
        data[i * 6 + 3] = latest_frame[i].intensity;
        data[i * 6 + 4] = (float)(latest_frame[i].timestamp);
        data[i * 6 + 5] = (float)(latest_frame[i].channel);
    }
    return array;
}

static PyMethodDef Methods[] = {
    { "get_latest_frame", get_latest_frame, METH_NOARGS, "Get the latest LiDAR frame as a numpy array." },
    { NULL, NULL, 0, NULL }
};

static struct PyModuleDef moduledef = {
    PyModuleDef_HEAD_INIT,
    "pylidar",
    NULL,
    -1,
    Methods
};

PyMODINIT_FUNC PyInit_pylidar(void) {
    import_array();
    std::thread(recv_loop).detach();
    return PyModule_Create(&moduledef);
}
