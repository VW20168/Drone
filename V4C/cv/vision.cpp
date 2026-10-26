#include <iostream>
#include <time.h>
#include <math.h>
#include "vision.h"
#include "yolov8_utils.h"
#include "../utils.h"

//CLI to take a picture: rpicam-still --output my_picture.png --width 1280 --height 720
#define VIDEO_FRAME_WIDTH  1920
#define VIDEO_FRAME_HEIGHT 1080 //720

#define CV_FRAME_WIDTH  640 //1280
#define CV_FRAME_HEIGHT 360 //720

#define PNG_WIDTH           128
#define PNG_HEIGHT          72

#define MIN_KEY_POINTS      10

#define FLANN_HESSIAN         3000 //was 3600

//The threshold used to filter out weak features in low-contrast regions.
//A larger threshold produces fewer features
#define SIFT_CONTRAST_THRESHOLD     0.1 //default 0.04
//The threshold used to filter out edge-like features.
//A smaller threshold produces fewer features
#define SIFT_EDGE_THRESHOLD         4 //default 10

#define OPENCV_A_FLANN         0
#define OPENCV_A_ORB          1
#define OPENCV_A_SIFT         2

Vision::Vision() {

    ready = false;
    m_camera = NULL;
    png_data = NULL;
    png_size = 0;
    model_path_detect = "../models/yolov8s-pose1.onnx";
    szFileName[0] = '\0';

    //the camera was calibrated at 1280x720
    double scale = 720.0 / CV_FRAME_HEIGHT;
    K = (cv::Mat_<double>(3, 3) <<
            1144.5397030099555/scale, 0, 634.76431909754456/scale,  // Example fx, cx (focal length x, principal point x)
            0, 1142.9056915078113/scale, 348.62474291113233/scale,  // Example fy, cy (focal length y, principal point y)
            0, 0, 1);
    //distortion_ceofficients
    D = (cv::Mat_<double>(5, 1) << -0.11838226480946731, 0.45675398945665519, -0.00024057803430318662, -0.00095775480732317815, 0);

    tryingOpenCV = OPENCV_A_SIFT;//OPENCV_A_FLANN;
}

Vision::~Vision() {
}

void Vision::stop() {
    if (m_camera) {
        m_camera->stopVideo();
        delete m_camera;
        m_camera = NULL;
    }
    if (png_data) {
        free (png_data);
        png_data = NULL;
    }
    std::cout << "[VISION]total frames " << sequence << std::endl;
    ready = false;
}

bool Vision::start() {
    ready = false;

    png_size = 0;
    sequence = 0;
    szFileName[0] = '\0';
    frame = cv::Mat(CV_FRAME_HEIGHT, CV_FRAME_WIDTH, CV_8UC3);
    restart();

    m_camera = new lccv::PiCamera();
    m_camera->options->verbose = false;
    m_camera->options->video_width = VIDEO_FRAME_WIDTH;
    m_camera->options->video_height= VIDEO_FRAME_HEIGHT;
    //https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/64MP-OV64A40/#frame-rate-resolution
    m_camera->options->framerate = 60;

    bool ret = m_camera->startVideo();
    ready = ret && task_detect_ocv.ReadModel(net, model_path_detect, false);
    if (ready) {
        png_data = (unsigned char*)malloc (CV_FRAME_WIDTH * CV_FRAME_HEIGHT * 4);//rgba not compressed
    }
    else {
       stop();
    }

    if (ready) {
        cv::Mat stock = cv::imread("../models/stop_sign.png", cv::IMREAD_COLOR);
        //1:1 or 16:9
        cv::resize(stock, stopsign, cv::Size(stock.cols > stock.rows + 20 ? CV_FRAME_WIDTH : CV_FRAME_HEIGHT,  CV_FRAME_HEIGHT), 0, 0, cv::INTER_LINEAR);
        yoloCvDetect(stopsign, keypoints_stopsign, descriptors_stopsign);
        //save the stock image with keypoints drawing on it
        for (unsigned int j = 0; j < keypoints_stopsign.size(); ++j) {
            cv::KeyPoint kpt = keypoints_stopsign[j];
            cv::circle(stopsign, cv::Point(kpt.pt.x, kpt.pt.y), 4, cv::Scalar(255, 0, 255), -1, 8); // Green circle, filled in
        }
        sprintf(szFileName, "../frames/ss.png");
        cv::imwrite(szFileName, stopsign);

        //object physical dimension
        stopsign_points.clear();
        stopsign_points.push_back(cv::Point3f(-0.15f, -0.15f, 0.0f)); //in meters
        stopsign_points.push_back(cv::Point3f(0.15f, -0.15f, 0.0f));
        stopsign_points.push_back(cv::Point3f(0.15f, 0.15f, 0.0f));
        stopsign_points.push_back(cv::Point3f(-0.15f, 0.15f, 0.0f));
    }

    return ready;
}

void Vision::restart() {
    restarted = true;

    errors = 0;
    ma_header = 0;
    ma_total = 0;
    for (int i = 0; i < MA_BUFF_SIZE; ++i)
        ma_values[i] = 0;

    totalMeasurements = measurementHeader = 0;
}

int Vision::loop(double alt, double *px, double *py, double *pz, long *seq) {
    int ret = VO_RUNTIME_ERROR;
    try {
        ret = doLoop(alt, px, py, pz, seq);
    }
    catch (const cv::Exception& ex) {
        std::cerr << "OpenCV exception caught: " << ex.what() << std::endl;
        //terminate called after throwing an instance of 'cv::Exception'
        //what():
        // OpenCV(4.12.0-dev) /home/pi/opencv/opencv/modules/core/src/matrix.cpp:1171: error: (-13:Image step is wrong)
        // The matrix is not continuous, thus its number of rows can not be changed in function 'reshape'
        ++errors;
    }
    catch (...) {
        ++errors;
    }
    if (ret != VO_OK) {
        //sprintf(szFileName, "../frames/E%05ld_%02d_%03d.png", sequence, ret, focusMeasure);
        //cv::imwrite(szFileName, frame);
    }
    //std::cout << "[" << sequence <<"]Code " << ret << ",RESULT(y) " << *py << " ,focus " << focusMeasure << std::endl;

    return ret;
}

int Vision::doLoop(double alt, double *px, double *py, double *pz, long *seq) {
    *seq = sequence++;
    *px = *py = *pz = 0;
    focusMeasure = 0;

    if (!ready)
        return VO_FAILED_TO_START_VIDEO;

    cv::Mat full_frame;
    //long now = Utils::currentTimeMs();
    cv::Mat video_frame, video_resize, video_gray;
    int failures = 0, max = 30;
    bool ok = true;
    //get one frame with the best focus
    for (int i = 0; i < max; ++i) {
        ok = m_camera->getVideoFrame(video_frame, 60) && video_frame.isContinuous();
        if (!ok) {
            if (++failures >= max / 2)
                return VO_FAILED_TO_CAPTURE_FRAME;
            continue;
        }
        cv::resize(video_frame, video_resize, cv::Size(CV_FRAME_WIDTH, CV_FRAME_HEIGHT), cv::INTER_LINEAR);
        cv::cvtColor(video_resize, video_gray, cv::COLOR_BGR2GRAY);
        int focus = varianceOfLaplacian(video_gray);
        if (focus > focusMeasure) {
            focusMeasure = focus;
            frame = video_resize.clone();
        }
    }

    int ret = yoloCvDetect(frame, keypoints_scene, descriptors_scene);
    if (ret != VO_OK)
        return ret;

    //camera_snapshot(frame, &keypoints_scene, false);

    cv::Mat rotation_matrix = prehomography();
    if (rotation_matrix.empty())
        return VO_HOMOGRAPHY;

    ret = VO_PNP;

    //object physical dimension

    // 2. Define corresponding 2D image points (pixel coordinates)
    std::vector<cv::Point2f> image_points;
    // These points would typically come from feature detection (e.g., ArUco marker corners)
    // Example placeholder values (replace with actual detected points):
    image_points.push_back(cv::Point2f(roi.x, roi.y + roi.height));
    image_points.push_back(cv::Point2f(roi.x + roi.width, roi.y + roi.height));
    image_points.push_back(cv::Point2f(roi.x + roi.width, roi.y));
    image_points.push_back(cv::Point2f(roi.x, roi.y));

    // 3. Output variables for rotation and translation vectors
    cv::Mat rvec, tvec;

    // 4. Call cv::solvePnP
    bool success = cv::solvePnP(
        stopsign_points, image_points,
        K, D,
        rvec, tvec,
        false, // useExtrinsicGuess
        cv::SOLVEPNP_ITERATIVE // flag
    );
    if (success) {
        //std::cout << "MATRIX " << rotation_matrix << std::endl;
        cv::Mat result_mat = rotation_matrix * tvec;
        // Convert the result back to a cv::Vec if needed
        // Assuming the result is a 3x1 matrix, it can be cast to a Vec3f
        cv::Vec3f point = result_mat;
        float vo_measured_y = point[1];

        /*cv::Mat rvec_decomp;
        cv::Rodrigues(rvec, rvec_decomp);
        cv::Mat rvec_m = rvec_decomp * rotation_matrix;
        

        double _pitch = 0, _roll = 0, _yaw = 0;
        try {
            cv::Vec3d angles = rotationMatrixToEulerZYX(rotation_matrix, true); // get degrees
            _pitch = angles[0];
            _roll  = angles[2];
            _yaw   = 0 - angles[1];
        }
        catch (const std::invalid_argument& ex) {
        }
        std::cout << "[" << sequence << "]*" << _pitch <<"," << _roll << "," << _yaw << std::endl;
        */
        
        bool valid = true;
        if (valid && !isnan(vo_measured_y)) {
            if (restarted) {
                restarted = false;

                anchor_alt = vo_measured_y;
                measured_alt = alt;

                //save the template frame with key points to an image file
                for (unsigned int j = 0; j < keypoints_scene.size(); ++j) {
                    cv::KeyPoint kpt = keypoints_scene[j];
                    cv::circle(frame, cv::Point(kpt.pt.x, kpt.pt.y), 2, cv::Scalar(0, 0, 255), -1, 8); // Green circle, filled in
                }
                sprintf(szFileName, "../frames/t00000.png");
                cv::imwrite(szFileName, frame);
            }
            *py = pushback_alt_ma(vo_measured_y - anchor_alt + measured_alt);
            ret = VO_OK;
            //std::cout << "===>Recovered (deg): pitch=" << _pitch << " ? " << pitch  << "! roll=" << _roll << " ? " << roll << "! yaw=" << _yaw << " ? " << yaw << "!@@@ " << point[0] << "," << point[1] << "," << point[2] << std::endl;
            //std::cout << "CV Alt " << vo_measured_y << std::endl;
        }
    }

    return ret;
}

int Vision::yoloCvDetect(cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors) {
    keypoints.clear();

    std::vector<OutputParams> result;
    task_detect_ocv.Detect(img, net, result);
    //yolo_snapshot(frame, result, task_detect_ocv._className);

    int objects = 0;
    for (int i = 0; i < (int)result.size(); i++) {
        //id 11: stop sign, see _className defined in yolov8.h
        if (result[i].id == 11 && result[i].confidence > 0.8f) {
            ++objects;
            roi = result[i].box;
        }
    }
    if (objects != 1)
        return VO_NO_OBJECT_RECOGNIZED;

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    cv::Mat masked;
    maskOutsideRect(gray, masked, roi);

    // Not all combinations of Feature Detector - Extractor - Matcher would work,
    // see http://stackoverflow.com/questions/14808429/classification-of-detectors-extractors-and-matchers/14912160
    //SIFT and SURF are patented by their respective creators
    // (FAST, SURF) / SURF  / FlannBased  <-- in "flann" OpenCV module.
    // (FAST, SIFT) / SIFT  / FlannBased
    // (FAST, ORB)  / ORB  / BruteForce  <-- BruteForce in OpenCV legacy module.
    // (FAST, ORB)  / BRIEF / BruteForce
    // (FAST, SURF) / FREAK / BruteForce
    if (tryingOpenCV == OPENCV_A_FLANN) {
        cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(FLANN_HESSIAN);
        detector->detectAndCompute(masked, cv::noArray(), keypoints, descriptors);
    }
    else if (tryingOpenCV == OPENCV_A_ORB) {
        cv::Ptr<cv::ORB> detector = cv::ORB::create();
        detector->detectAndCompute(masked, cv::noArray(), keypoints, descriptors);
    }
    else if (tryingOpenCV == OPENCV_A_SIFT) {//In modern OpenCV, SIFT/SURF are often more stable than ORB for this purpose, though behavior can vary by version
        cv::Ptr<cv::SIFT> detector = cv::SIFT::create(0, 3, SIFT_CONTRAST_THRESHOLD, SIFT_EDGE_THRESHOLD);
        detector->detectAndCompute(masked, cv::noArray(), keypoints, descriptors);
    }

    return VO_OK;
}

cv::Mat Vision::prehomography() {
    cv::Mat ret;//empty

    if (keypoints_scene.size() < MIN_KEY_POINTS || descriptors_scene.cols != descriptors_stopsign.cols)
        return ret;

    std::vector<cv::DMatch> matches;
    if (tryingOpenCV == OPENCV_A_FLANN) {
        std::vector< std::vector<cv::DMatch> > knn_matches;

        //Matching descriptor vectors with a FLANN based matcher
        //Since SURF is a floating-point descriptor NORM_L2 is used
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        matcher->knnMatch(descriptors_stopsign, descriptors_scene, knn_matches, 2);

        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.7f;
        for (size_t i = 0; i < knn_matches.size(); i++) {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
                matches.push_back(knn_matches[i][0]);
            }
        }
    }
    else if (tryingOpenCV == OPENCV_A_ORB) {
        //Match Descriptors using BFMatcher (Brute Force Matcher)
        cv::BFMatcher matcher(cv::NORM_HAMMING, true);
        //OpenCV exception caught: OpenCV(4.12.0-dev) /home/pi/opencv/opencv/modules/core/src/batch_distance.cpp:274: error: (-215:Assertion failed) type == src2.type() && src1.cols == src2.cols && (type == CV_32F || type == CV_8U) in function 'batchDistance'
        //std::cout << "*****" << descriptors_object.type() << " vs " <<  descriptors_scene.type() << ", cols " << descriptors_object.cols << " vs " << descriptors_scene.cols << std::endl;
        matcher.match(descriptors_stopsign, descriptors_scene, matches);
        //cv::BFMatcher matcher(cv::NORM_L2, true);
        //matcher.match(descriptors_stopsign, descriptors_scene, matches);
    }
    else if (tryingOpenCV == OPENCV_A_SIFT) {
        cv::BFMatcher matcher(cv::NORM_L2, true); // Use NORM_L2 for SIFT
        matcher.match(descriptors_stopsign, descriptors_scene, matches);
    }

    // Extract location of good matches
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;

    for (size_t i = 0; i < matches.size(); i++) {
        obj.push_back(keypoints_stopsign[matches[i].queryIdx].pt);
        scene.push_back(keypoints_scene[matches[i].trainIdx].pt);
    }
    
    //map_snapshot(obj, scene);
    /*

    std::vector<cv::Point3f> obj3f;
    std::vector<cv::Point2f> scene2f;
    float scale = 1280.0f / CV_FRAME_WIDTH;
    for (size_t i = 0; i < matches.size(); i++) {
        cv::Point2f pt = keypoints_stopsign[matches[i].queryIdx].pt;
        float x = (pt.x * scale - 640) / 240 * 0.15f;
        float y = (pt.y * scale - 360) / 240 * -0.15f;
        obj3f.push_back(cv::Point3f(x, y, 0));
        
        scene2f.push_back(keypoints_scene[matches[i].trainIdx].pt);
    }
    
    cv::Mat rvec, tvec;    
    bool success = cv::solvePnPRansac(
        obj3f, scene2f,
        K, D,
        rvec, tvec,
        false, // useExtrinsicGuess
        2000, //iterations count
        cv::SOLVEPNP_IPPE // flag SOLVEPNP_ITERATIVE or SOLVEPNP_IPPE
    );
    if (success) {
        cv::Mat rvec_decomp;
        cv::Rodrigues(rvec, rvec_decomp);
        cv::Vec3d angles = rotationMatrixToEulerZYX(rvec_decomp, true); // get degrees
        double _pitch = angles[0];
        double _roll  = angles[2];
        double _yaw   = angles[1];
        
        //std::cout << "[" << sequence << "]" << rvec << "\n" <<  << std::endl;
        //std::cout << "[" << sequence << "]" << _pitch <<"," << _roll << "," << _yaw << std::endl;
    }
    else {
        std::cout << "[" << sequence << "]" << "PnP Ransac failed" << std::endl;
    }
    */
    
    // Calculate Homography
    // Minimum of 4 points are needed.
    //!std::cout << std::endl << "[" << sequence << "]Points " << obj.size() << "+" << scene.size() << std::endl;
    if (obj.size() >= 4 && scene.size() >= 4) {
        cv::Mat masks;
        cv::Mat H = cv::findHomography(obj, scene, cv::RANSAC, 3, masks); // RANSAC is key for robust estimation
        std::vector<cv::Mat> Rs_decomp, ts_decomp, normals_decomp;
        //OpenCV exception caught: OpenCV(4.12.0-dev) /home/pi/opencv/opencv/modules/core/src/matrix.cpp:1171: error: (-13:Image step is wrong) The matrix is not continuous, thus its number of rows can not be changed in function 'reshape'
        int solutions = H.isContinuous() ? cv::decomposeHomographyMat(H, K, Rs_decomp, ts_decomp, normals_decomp) : -1;
        if (solutions > 1) {
            // Vectors to store only the inlier points
            std::vector<cv::Point2f> srcInliers;
            std::vector<cv::Point2f> dstInliers;
            // Iterate through the mask to filter inliers
            for (int i = 0; i < masks.rows; i++) {
                // The mask values are either 0 (outlier) or 1 (inlier) and should be accessed as uchar
                if (masks.at<uchar>(i, 0) == 1) {
                    srcInliers.push_back(obj[i]);
                    dstInliers.push_back(scene[i]);
                }
            }

            //map_snapshot(srcInliers, dstInliers);

            cv::Mat possibleSolutions; // This output mask will indicate the valid solutions
            cv::filterHomographyDecompByVisibleRefpoints(
                Rs_decomp,
                normals_decomp,
                srcInliers,
                dstInliers,
                possibleSolutions,
                cv::noArray() // Optional points mask
            );
            //!std::cout << "Details " << solutions << "," << possibleSolutions.rows << std::endl;
            //std::cout << "[" << sequence << "]Total solutions > " << possibleSolutions.rows << " < @!" << srcInliers.size() << std::endl;
            int selected = -1; double minScore = 0;
            int required = (totalMeasurements > 0) ? 4 : 1;
            double bestPitch = 0, bestRoll = 0, bestYaw = 0;
            /*
            int positives = 0;
            {
                int *pdata = (int *)possibleSolutions.data;
                for (int j = 0; j < possibleSolutions.rows; ++j) {
                    int i = pdata[j];
                    cv::Vec3f flat = normals_decomp[i];
                    if (flat[2] >= 0)
                        ++positives;
                }
            }
            std::cout << "Positives=" << positives << ", required=" << required << std::endl;
            if (positives > 0 && positives <= required) {*/
            if (possibleSolutions.rows <= required) {
                int *pdata = (int *)possibleSolutions.data;
                for (int j = 0; j < possibleSolutions.rows; ++j) {
                    int i = pdata[j];
                    //cv::Vec3f flat = normals_decomp[i];
                    //if (flat[2] < 0)
                    //    continue;
                    double _pitch = 0, _roll = 0, _yaw = 0;
                    bool valid = true;
                    try {
                        cv::Vec3d angles = rotationMatrixToEulerZYX(Rs_decomp[i], true); // get degrees
                        _pitch = angles[0];
                        _roll  = angles[2];
                        _yaw   = 0 - angles[1];
                        //std::cout << _pitch << " " << _roll << " " << _yaw << normals_decomp[i] << std::endl;
                    }
                    catch (const std::invalid_argument& ex) {
                        std::cerr << ex.what() << std::endl;
                        valid = false;
                    }
                    if (valid) {
                        double diff1 = _pitch;
                        double diff2 = _roll;
                        double diff3 = _yaw;
                        if (totalMeasurements > 0) {
                            double avgP, avgR, avgY;
                            get_pry_ma(&avgP, &avgR, &avgY);
                            diff1 -= avgP;
                            diff2 -= avgR;
                            diff3 -= avgY;
                        }
                        double score = diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
                        if (selected < 0 || score < minScore) {
                            //std::cout << " Solution " << i << ": pitch=" << _pitch  << " roll=" << _roll << " yaw=" << _yaw << std::endl;
                            selected = i;
                            minScore = score;

                            bestPitch = _pitch;
                            bestRoll = _roll;
                            bestYaw = _yaw;
                        }
                    }
                }
            }
            if (selected >= 0) {
                ret = Rs_decomp[selected];
                pushback_pry_ma(bestPitch, bestRoll, bestYaw);
            }
            //else {
            //    std::cout << "***Not enough selections" << std::endl;
            //}                    
        }
        //else {
        //    std::cout << "***Not enough solutions" << std::endl;
        //}        
    }
    //else {
    //    std::cout << "***Not enough key points" << std::endl;
    //}
    return ret;
}

void * Vision::getLastFrame(long *data_size) {
    *data_size = png_size;
    return (png_size <= 0) ? NULL : png_data;
}

long Vision::getTotalFrames() {
    return sequence;
}

void Vision::yolo_snapshot(cv::Mat& frame, std::vector<OutputParams> result, std::vector<std::string> classNames) {
    cv::Mat img = frame.clone();
    //generate random colors
    std::vector<cv::Scalar> color;
    srand(time(0));
    for (int i = 0; i < 80; i++) {
        int b = rand() % 256;
        int g = rand() % 256;
        int r = rand() % 256;
        color.push_back(cv::Scalar(b, g, r));
    }
    DrawPred(img, result, classNames, color);

    sprintf(szFileName, "../frames/y%05ld.png", sequence);
    cv::imwrite(szFileName, img);
}

void Vision::camera_snapshot(cv::Mat& frame, std::vector<cv::KeyPoint> *kps_scene, bool save_frame) {
    //keep the latest frame in memory as a png image
    cv::Mat resizedImage; cv::Size newSize(PNG_WIDTH, PNG_HEIGHT); //16:9
    cv::resize(frame, resizedImage, newSize, 0, 0, cv::INTER_LINEAR);
    cv::circle(resizedImage, cv::Point(10, 10), 4, restarted ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), -1, 8);
    std::vector<uchar> png;
    cv::imencode(".jpg", resizedImage, png);
    png_size = png.size();
    memcpy(png_data, png.data(), png_size);

    if (save_frame) {
        cv::Mat img = frame.clone();
        if (kps_scene) {
            for (unsigned int j = 0; j < kps_scene->size(); ++j) {
                cv::KeyPoint kpt = kps_scene->at(j);
                cv::circle(img, cv::Point(kpt.pt.x, kpt.pt.y), 2, cv::Scalar(255, 0, 0), -1, 8); // Green circle, filled in
            }
        }
        sprintf(szFileName, "../frames/%05ld.png", sequence);
        cv::imwrite(szFileName, img);
    }
}

void Vision::map_snapshot(std::vector<cv::Point2f> &obj, std::vector<cv::Point2f> &scene) {
    cv::Mat img = frame.clone();

    for (unsigned int j = 0; j < keypoints_scene.size(); ++j) {
        cv::KeyPoint kpt = keypoints_scene.at(j);
        cv::circle(img, cv::Point(kpt.pt.x, kpt.pt.y), 2, cv::Scalar(255, 0, 255), -1, 8); // Green circle, filled in
    }

    cv::Mat small;

    int scale_factor = 4;
    int cx = stopsign.cols / scale_factor;
    int cy = stopsign.rows / scale_factor;
    cv::resize(stopsign, small, cv::Size(cx, cy), 0, 0, cv::INTER_LINEAR);

    small.copyTo(img(cv::Rect(0, 0, cx, cy)));

    std::vector<cv::Scalar> color;
    srand(time(0));
    for (int i = 0; i < 80; i++) {
        int b = rand() % 256;
        int g = rand() % 256;
        int r = rand() % 256;
        color.push_back(cv::Scalar(b, g, r));
    }

    for (unsigned int i = 0; i < obj.size(); ++i) {
        cv::Point2f src(obj[i].x / scale_factor,  obj[i].y/scale_factor);
        cv::Point2f dst = scene[i];
        cv::line(img, src, dst, color[i], 1);
    }

    sprintf(szFileName, "../frames/m%05ld.png", sequence);
    cv::imwrite(szFileName, img);
}

void Vision::maskOutsideRect(const cv::Mat& input_image, cv::Mat& output_image, cv::Rect roi_rect) {
    // 1. Create a mask image of the same size and type as the input image, initialized to black (0)
    cv::Mat mask = cv::Mat::zeros(input_image.size(), input_image.type());

    // 2. Define the region of interest (ROI) within the mask using the specified rectangle
    // This allows you to draw only within that area
    cv::Mat mask_roi = mask(roi_rect);

    // 3. Fill the ROI within the mask with white (255, 255, 255)
    // Using a negative thickness value in drawing functions fills the shape
    // Since we are applying to the mask_roi, it will only fill that specific rectangle area
    mask_roi.setTo(cv::Scalar(255, 255, 255));

    // 4. Invert the mask so that the area *outside* the rectangle is white, and the inside is black
    //cv::bitwise_not(mask, mask);

    // 5. Apply the inverted mask to the original image using bitwise_and
    // Pixels in the input_image corresponding to black (0) in the mask will become black in the output
    cv::bitwise_and(input_image, input_image, output_image, mask);
}

// Convert a 3x3 cv::Mat rotation matrix to Euler angles (roll, pitch, yaw).
// Convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)  (ZYX)
// Returns cv::Vec3d(roll, pitch, yaw). Angles are in radians by default; set degrees=true for degrees.
cv::Vec3d Vision::rotationMatrixToEulerZYX(const cv::Mat& R_in, bool degrees, double eps) {
    if (R_in.empty() || R_in.rows != 3 || R_in.cols != 3) {
        throw std::invalid_argument("rotationMatrixToEulerZYX: input must be 3x3 cv::Mat");
    }
    // Convert to double precision single-channel matrix if needed
    cv::Mat R;
    R_in.convertTo(R, CV_64F);
    // Check single channel
    if (R.channels() != 1) {
        throw std::invalid_argument("rotationMatrixToEulerZYX: input must be single-channel 3x3 matrix");
    }
    // Extract elements
    double r00 = R.at<double>(0,0), r01 = R.at<double>(0,1), r02 = R.at<double>(0,2);
    double r10 = R.at<double>(1,0), r11 = R.at<double>(1,1), r12 = R.at<double>(1,2);
    double r20 = R.at<double>(2,0), r21 = R.at<double>(2,1), r22 = R.at<double>(2,2);

    // Clamp r20 to [-1, 1] to avoid NaN from asin due to numerical errors
    r20 = std::max(-1.0, std::min(1.0, r20));

    double roll, pitch, yaw;

    // For ZYX (R = Rz(yaw) * Ry(pitch) * Rx(roll))
    // r20 = -sin(pitch)
    if (std::abs(r20) < 1.0 - eps) {
        pitch = -std::asin(r20);            // pitch = -asin(r20)
        roll  = std::atan2(r21, r22);       // roll  = atan2(r21, r22)
        yaw   = std::atan2(r10, r00);       // yaw   = atan2(r10, r00)
    } else {
        // Gimbal lock: |r20| ~= 1 -> pitch ~= +/- 90 deg
        if (r20 <= -1.0 + eps) {
            // pitch = +pi/2
            pitch = M_PI / 2.0;
            yaw = 0.0; // set to zero (one DOF lost)
            // roll + yaw = atan2(r01, r02) -> roll = atan2(r01, r02)
            roll = std::atan2(r01, r02);
        } else {
            // r20 >= +1 - eps -> pitch = -pi/2
            pitch = -M_PI / 2.0;
            yaw = 0.0;
            // roll - yaw = atan2(-r01, -r02) -> roll = atan2(-r01, -r02)
            roll = std::atan2(-r01, -r02);
        }
    }

    if (degrees) {
        const double rad2deg = 180.0 / M_PI;
        roll  *= rad2deg;
        pitch *= rad2deg;
        yaw   *= rad2deg;
    }

    return cv::Vec3d(roll, pitch, yaw);
}

// Function to calculate the focus measure (variance of Laplacian)
int Vision::varianceOfLaplacian(const cv::Mat& gray) {
    cv::Mat laplacianImage;

    // Compute the Laplacian of the grayscale image
    cv::Laplacian(gray, laplacianImage, CV_64F);

    // Compute the mean and standard deviation
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacianImage, mean, stddev, cv::Mat());

    // Calculate the variance: standard deviation squared
    return (int)(stddev.val[0] * stddev.val[0]);
}

float Vision::pushback_alt_ma(float value) {
    //std::cout <<"READING " << value << std::endl;
    ma_values[ma_header] = value;
    if (++ma_header >= MA_BUFF_SIZE)
        ma_header = 0;
    ++ma_total;
    int end = (ma_total >= MA_BUFF_SIZE) ? MA_BUFF_SIZE : ma_header;
    float total = 0;
    for (int i = 0; i < end; ++i) {
        total += ma_values[i];
    }
    return total / end;
}

void Vision::pushback_pry_ma(double pitch, double roll, double yaw) {
    lastKnownPitch[measurementHeader] = pitch;
    lastKnownRoll[measurementHeader] = roll;
    lastKnownYaw[measurementHeader] = yaw;

    if (++measurementHeader >= MA_BUFF_SIZE)
        measurementHeader = 0;
    ++totalMeasurements;
}

bool Vision::get_pry_ma(double *pitch, double *roll, double *yaw) {
    if (totalMeasurements <= 0)
        return false;

    int end = (totalMeasurements >= MA_BUFF_SIZE) ? MA_BUFF_SIZE : measurementHeader;
    double totalPitch = 0, totalRoll = 0, totalYaw = 0;
    for (int i = 0; i < end; ++i) {
        totalPitch += lastKnownPitch[i];
        totalRoll += lastKnownRoll[i];
        totalYaw += lastKnownYaw[i];
    }
    *pitch = totalPitch / end;
    *roll = totalRoll / end;
    *yaw = totalYaw / end;

    return true;
}
