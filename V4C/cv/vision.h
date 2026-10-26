#ifndef __VISION_H__
#define __VISION_H__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <lccv.hpp>
#include "yolov8.h"

//Visual odometry error codes
#define VO_FAILED_TO_START_VIDEO        -1
#define VO_FAILED_TO_CAPTURE_FRAME      -2
#define VO_RUNTIME_ERROR                -3

#define VO_OK                           0
#define VO_NO_OBJECT_RECOGNIZED         10
#define VO_HOMOGRAPHY                   11
#define VO_PNP                          12

#define MA_BUFF_SIZE                    4

class Vision {
    private:
    bool ready;
    long sequence;
    int errors;
    lccv::PiCamera *m_camera;
    char szFileName[128];

    std::string model_path_detect;
    cv::dnn::Net net;
    Yolov8 task_detect_ocv;
    unsigned char *png_data;
    long png_size;
    bool restarted;

    cv::Mat stopsign;
    cv::Mat K, D, frame;

    cv::Rect roi;
    std::vector<cv::KeyPoint> keypoints_stopsign, keypoints_scene;
    cv::Mat descriptors_stopsign, descriptors_scene;
    std::vector<cv::Point3f> stopsign_points;

    int focusMeasure;
    double anchor_alt, measured_alt; //altitude measured by the altimeter
    double lastKnownPitch[MA_BUFF_SIZE], lastKnownRoll[MA_BUFF_SIZE], lastKnownYaw[MA_BUFF_SIZE];
    int totalMeasurements, measurementHeader;

    float ma_values[MA_BUFF_SIZE];
    int ma_header;
    long ma_total;
    int tryingOpenCV;

    public:
    Vision();
    virtual ~Vision();

    bool start();
    void stop();
    int loop(double alt, double *px, double *py, double *pz, long *seq);
    int research();
    void restart();
    void *getLastFrame(long *data_size);
    long getTotalFrames();

    private:
    int doLoop(double alt, double *px, double *py, double *pz, long *seq);
    void yolo_snapshot(cv::Mat& frame, std::vector<OutputParams> result, std::vector<std::string> classNames);
    void camera_snapshot(cv::Mat& frame, std::vector<cv::KeyPoint> *keypoints_scene, bool save_frame);
    void map_snapshot(std::vector<cv::Point2f> &obj, std::vector<cv::Point2f> &scene);
    cv::Mat prehomography();

    void maskOutsideRect(const cv::Mat& input_image, cv::Mat& output_image, cv::Rect roi_rect);
    cv::Vec3d rotationMatrixToEulerZYX(const cv::Mat& R_in, bool degrees = false, double eps = 1e-6);
    
    int varianceOfLaplacian(const cv::Mat& gray);
    float pushback_alt_ma(float value);
    void pushback_pry_ma(double pitch, double roll, double yaw);
    bool get_pry_ma(double *pitch, double *roll, double *yaw);
    int yoloCvDetect(cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);
};

#endif
