#include "camera.hpp"
#include "scope_guard.hpp"
#include "rc.hpp"
//#include "raspicam/RaspiCamCV.h"
#include "spdlog/spdlog.h"
#include <raspicam/raspicam_cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <algorithm>
using namespace cv;
using namespace std;

namespace copexp {
namespace signal {
extern volatile sig_atomic_t interrupt;
}
}

int cameraNetwork(copexp::RC2Queue &queue);

void copexp::cameraOpen(copexp::RC2Queue &queue)
{
    auto log = spdlog::get("opencv");
    log->info("OpenCV thread started");

    while (!cameraNetwork(queue));

    log->info("Camera service shutting down");
    return;
}

template <typename R, typename T>
R average(T array)
{
    R sum = 0;
    size_t count = 0;
    for (auto i = array.begin(); i != array.end(); ++i)
    {
        sum += *i;
        ++count;
    }
    return sum / count;
}


#if 0
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        log->error("Cannot open camera with opencv");
        return;
    }

    log->info("Camera capture started");

    cv::Mat frame;
    while (true)
    {
        cap >> frame;

        if (copexp::signal::interrupt)
        {
            break;
        }
    }
    log->info("Camera service shutting down");
#endif

#if 1
int cameraNetwork(copexp::RC2Queue &queue)
{
    auto log = spdlog::get("opencv");

    //--------------------------------------------------------
    //networking stuff: socket, bind, listen
    //--------------------------------------------------------
    int localSocket, remoteSocket, port = 4097;
    struct sockaddr_in localAddr, remoteAddr;

    int addrLen = sizeof(struct sockaddr_in);

    localSocket = socket(AF_INET , SOCK_STREAM , 0);
    if (localSocket == -1)
    {
        log->error("Server socket call failed, port {}", port);
    }

    td::scope_guard sg([&]()
    {
        close(remoteSocket);
    });

    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = INADDR_ANY;
    localAddr.sin_port = htons(port);

    if (bind(localSocket, (struct sockaddr *)&localAddr , sizeof(localAddr)) < 0)
    {
        log->error("Can't bind() socket, port {}", port);
        return 1;
    }

    //Listening
    listen(localSocket, 1);

    log->info("Server waiting for connections, port {}", port);

    //accept connection from an incoming client
    remoteSocket = accept(localSocket, (struct sockaddr *)&remoteAddr, (socklen_t*)&addrLen);
    if (remoteSocket < 0)
    {
        log->error("Accept failed!");
        return 1;
    }
    log->info("Connection accepted");

    //----------------------------------------------------------
    //OpenCV Code
    //----------------------------------------------------------
    int capDev = 0;

#if 0
    auto raspicam = raspiCamCvCreateCameraCapture(capDev);
    if (raspicam == nullptr)
    {
        log->error("Raspicam failed!");
        return 1;
    }
    raspiCamCvSetCaptureProperty(raspicam, RPI_CAP_PROP_FRAME_WIDTH, 640);
    raspiCamCvSetCaptureProperty(raspicam, RPI_CAP_PROP_FRAME_HEIGHT, 480);
    td::scope_guard rspg([&]()
    {
        raspiCamCvReleaseCapture(&raspicam);
    });

#endif

#if 0
    cv::VideoCapture cap(capDev); // open the default camera
#endif

    cv::Mat img, imgGray;
    img = cv::Mat::zeros(480, 640, CV_8UC1);

    //make it continuous
    if (!img.isContinuous())
    {
        img = img.clone();
    }

    int imgSize = img.total() * img.elemSize();
    int bytes = 0;
    int key;

    //make img continuos
    if (!img.isContinuous())
    {
        img = img.clone();
        imgGray = img.clone();
    }

    log->info("Image Size: {}", imgSize);

    raspicam::RaspiCam_Cv camera;
    camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    if (!camera.open())
    {
        log->error("Error opening camera");
        return 1;
    }

    td::scope_guard camg([&]()
    {
        camera.release();
    });

    cv::Mat hsvMat;
    cv::Mat thresh1Mat, thresh2Mat, sumMat;

    cv::Mat src_gray;
    cv::Mat src;


    size_t buffer_position = 0;
    const size_t buffer_size = 16; //
    array<double, buffer_size> distance_buffer;
    array<double, buffer_size> mx_buffer;
    array<double, buffer_size> my_buffer;
    distance_buffer.fill(-1);
    my_buffer.fill(0);
    mx_buffer.fill(0);

    auto buffer_push = [&](double dist, double mx, double my)
    {
        buffer_position = buffer_position >= buffer_size ? 0 : buffer_position + 1;
        distance_buffer[buffer_position] = dist;
        mx_buffer[buffer_position] = mx;
        my_buffer[buffer_position] = my;
    };

    auto buffer_diff = [&](array<double, buffer_size>& array)
    {
        return array[buffer_position] -
                array[buffer_position == 0 ? (buffer_size - 1) : (buffer_position - 1)];
    };

    td::scope_guard sfg([&]()
    {
        queue.exec({ 1500, 1500, 1500, 1500, 1 });
    });

    queue.exec({ 1500, 1500, 1500, 1800, 1 });
    this_thread::sleep_for(chrono::seconds(2));

    while (true)
    {
        camera.grab();
        camera.retrieve(img);

#if 1
        cv::cvtColor(img, hsvMat, cv::COLOR_BGR2HSV);

        cv::inRange( hsvMat, cv::Scalar(0, 120, 70), cv::Scalar(15, 255, 255), thresh1Mat);
        cv::inRange( hsvMat, cv::Scalar(160, 120, 70), cv::Scalar(180, 255, 255), thresh2Mat);

        cv::add( thresh1Mat, thresh2Mat, sumMat );

        cv::Mat src_gray(sumMat);
#endif

        double width = 640.0;
        double height = 480.0;

        const double target_dist = 100.0;
        const double max_err_dist = 50.0;

        double mx, my, dist;
        cv::Moments m = moments(sumMat, true);
        if (m.m00 < 20.0)
        {
            mx = 0;
            my = 0;
            dist = target_dist - max_err_dist;
        }
        else
        {
            mx = ((width / 2) - m.m10 / m.m00);
            my = ((height / 2) - m.m01 / m.m00);
            dist = sqrt(m.m00);
        }

        double dist_error = min(max((target_dist - dist) / max_err_dist, -1.0), 1.0);
        double mx_error = mx / width * 2;
        double my_error = my / height * 2;

        const double Kp = 50.0; // 1000..2000
        const double Ki = 15.0;
        const double Kd = 50.0;

        const double Lp = 0.0;
        const double Li = 80.0;
        const double Ld = 50.0;

        const double throttle_hover = 1600.0;

        buffer_push(dist_error, mx_error, my_error);

        double roll = 1500.0 + my_error * Kp + average<double>(my_buffer)
                * Ki + buffer_diff(my_buffer) * Kd;

        double pitch = 1500.0 + mx_error * Kp + average<double>(mx_buffer)
                * Ki + buffer_diff(mx_buffer) * Kd;

        double throttle = throttle_hover - Lp * dist_error - Li * average<double>(distance_buffer)
                - Ld * buffer_diff(distance_buffer);


#if 0
        log->info("Moments: {}, error: {} {} {}",
                  dist, mx_error, my_error, dist_error);
#endif

        log->info("Command: R:{} P:{} TR:{}", roll, pitch, throttle);


                  //roll, pitch, throttle);

        queue.exec({ uint16_t(roll), uint16_t(pitch), 1500, uint16_t(throttle), 1 });

#if 0
//        cv::cvtColor(img, src_gray, cv::COLOR_BGR2GRAY);

//        GaussianBlur( src_gray, src_gray, Size(5, 5), 2, 2 );

        vector<Vec3f> circles;

        /// Apply the Hough Transform to find the circles
        HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 2,
                      src_gray.rows / 8, 200, 40, 0, 0 );

        /// Draw the circles detected
        for( size_t i = 0; i < circles.size(); i++ )
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle( src_gray, center, 3, Scalar(120), -1, 8, 0 );
            // circle outline
            circle( src_gray, center, radius, Scalar(120), 3, 8, 0 );
        }
#endif






        // do video processing here
//        cv::cvtColor(img, imgGray, CV_BGR2GRAY);

#if 1
        // send processed image
        if ((bytes = send(remoteSocket, src_gray.data, imgSize, 0)) < 0)
        {
            std::cerr << "bytes = " << bytes << std::endl;
            break;
        }
#endif

        if (copexp::signal::interrupt)
        {
            return 1;
        }
    }

    return 0;
}
#endif

void copexp::cameraProcess(RC2Queue &queue, cv::Mat &origMat)
{

#if 0
//        [[[VideoPreviewer instance] videoExtractor] putFrameIntoBuffer:&pbuf];

//        CVPixelBufferLockBaseAddress(pbuf, 0);
//        NSDictionary *opt =  @{ (id)kCVPixelBufferPixelFormatTypeKey :
//                @(kCVPixelFormatType_420YpCbCr8PlanarFullRange) };
//        CIImage *image = [[CIImage alloc]   initWithCVPixelBuffer:pbuf options:opt];
//        CVPixelBufferUnlockBaseAddress(pbuf, 0);

    cv::Mat origMat, outMat;

    const int frameWidth = origMat.cols;
    //const int frameHeight = origMat.rows;

    cv::Mat greyMat;
    cvtColor( origMat, greyMat, cv::COLOR_RGB2GRAY );

    //id18716750
    /*
         origMat = cv::Mat::zeros( 720, 1280, CV_8UC3 );

         cv::circle( origMat, cv::Point(100, 100), 1, cv::Scalar(255, 255, 0), -1 );
         cv::circle( origMat, cv::Point(200, 200), 2, cv::Scalar(255, 255, 0), -1 );
         cv::circle( origMat, cv::Point(300, 300), 3, cv::Scalar(255, 255, 0), -1 );
         cv::circle( origMat, cv::Point(400, 400), 4, cv::Scalar(255, 255, 0), -1 );
         cv::circle( origMat, cv::Point(500, 500), 4, cv::Scalar(255, 255, 0), -1 );

         for (int i=0; i < 6; i++) {
         cv::circle( origMat, cv::Point(820, 125+30*i), 3, cv::Scalar(255, 255, 0), -1 );
         }
         */

    // HSV mat
    /*
         cv::Mat hsvMat;
         cv::cvtColor(origMat, hsvMat, cv::COLOR_RGB2HSV);

         cv::Mat thresh1Mat, thresh2Mat, sumMat, outMat, tempBorderMat, borderMat, rgbBorderMat, outputMat;

         cv::inRange( hsvMat, cv::Scalar(0, 120, 70), cv::Scalar(9, 255, 255), thresh1Mat);
         cv::inRange( hsvMat, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), thresh2Mat);

         cv::add( thresh1Mat, thresh2Mat, sumMat );
         */

    //cv::inRange( hsvMat, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 120), sumMat);

    cv::Mat canny_output;
    canny_output = cv::Mat::zeros( 720, 1280, CV_8U );

    typedef std::vector<std::vector<cv::Point> > TContours;
    TContours contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::Canny(greyMat, canny_output, Ki, Ti);

    //cv::Canny( sumMat, canny_output, 50, 200, 3 );

    cv::findContours( canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );

    std::vector<cv::Moments> mu( contours.size() );
    for( int i = 0; i < contours.size(); i++ ) {
        mu[i] = moments( contours[i], true );
    }

    //  центры масс  // .. фантик по горизонтали видит метр с 44 см
    std::vector<cv::Point2f> mc;
    std::vector<cv::Scalar> meanColor;
    std::vector<double> areaToSqLengthRatio( contours.size() );
    for( int i = 0; i < mu.size(); i++ ) {
        if (mu[i].m00 > 10 && mu[i].m00 < 40000) {
            areaToSqLengthRatio[i] = cv::contourArea( contours[i], false ) / pow( cv::arcLength( contours[i], true ), 2 );
            if ( areaToSqLengthRatio[i] > 0.4 / ( 4*M_PI ) ) {

                if (!cvColorFilterEnabled) {
                    mc.push_back( cv::Point2f( mu[i].m10/mu[i].m00, 720 - mu[i].m01/mu[i].m00 ) );
                    meanColor.push_back( cv::Scalar( 100, 100, 200 ) );
                    continue;
                }
                cv::Mat contourMask;
                contourMask = cv::Mat::zeros( 720, frameWidth, CV_8U );
                cv::drawContours( contourMask, contours, i, cv::Scalar(255), CV_FILLED );
                cv::Scalar thisMeanColor = cv::mean( origMat, contourMask );

                cv::Mat meanPixel(2, 2, CV_8UC3, thisMeanColor);
                cv::Mat hsvMeanPixel(2, 2, CV_8UC3, cv::Scalar(0,0,0));
                cv::cvtColor( meanPixel, hsvMeanPixel, cv::COLOR_RGB2HSV );

                cv::Vec3b hsvMeanColor = hsvMeanPixel.at<cv::Vec3b>(1, 1);
                float h = hsvMeanColor.val[0];
                float s = hsvMeanColor.val[1];
                float v = hsvMeanColor.val[2];

                if ( s>50 && v>50 && ( h > 90 || h < 160) ) { //|| ( s<50 && v>230 ) ) {
                    mc.push_back( cv::Point2f( mu[i].m10/mu[i].m00, 720 - mu[i].m01/mu[i].m00 ) );
                    meanColor.push_back( thisMeanColor );
                }
            }
        }
    }

    std::vector<double> lineAngle( mc.size() * (mc.size() - 1) / 2 );
    std::vector<double> lineAngleDegrees( mc.size() * (mc.size() - 1) / 2 );
    std::vector<double> lineDistFromCenter( mc.size() * (mc.size() - 1) / 2 );

    double centerX = frameWidth / 2;
    double centerY = 720 / 2;

    long iterator = 0;

    for( int i = 0; i < mc.size(); i++ ) {
        double iPointDistFromCenter = sqrt( pow(mc[i].x-centerX, 2) + pow(mc[i].y-centerY, 2) );
        double iPointAngleToCenter = atan2( centerY - mc[i].y, centerX - mc[i].x );

        for( int j = 0; j < mc.size(); j++ ) {
            if ( i >= j ) continue;
            double thisLineAngle = atan2( mc[j].y - mc[i].y, mc[j].x - mc[i].x );
            if ( thisLineAngle < 0 ) thisLineAngle += M_PI;
            lineAngle[ iterator ] = thisLineAngle;
            lineAngleDegrees[ iterator ] = thisLineAngle * 180 / M_PI;
            lineDistFromCenter[ iterator ] = iPointDistFromCenter * sin( thisLineAngle - iPointAngleToCenter );
            iterator++;
        }
    }

    std::vector< std::pair<double, double> > line( mc.size() * (mc.size() - 1) / 2 );
    iterator = 0;

    for( int i = 0; i < mc.size(); i++ ) {
        double iPointDistFromCenter = sqrt( pow(mc[i].x-centerX, 2) + pow(mc[i].y-centerY, 2) );
        double iPointAngleToCenter = atan2( centerY - mc[i].y, centerX - mc[i].x );

        for( int j = 0; j < mc.size(); j++ ) {
            if ( i >= j ) continue;
            double thisLineAngle = atan2( mc[j].y - mc[i].y, mc[j].x - mc[i].x );
            if ( thisLineAngle < 0 ) thisLineAngle += M_PI;
            line[ iterator ].first = thisLineAngle;
            //lineAngleDegrees[ iterator ] = thisLineAngle * 180 / M_PI;
            line[ iterator ].second = iPointDistFromCenter * sin( thisLineAngle - iPointAngleToCenter );
            iterator++;
        }
    }

    struct sortByFirst {
        bool operator()(const std::pair<double,double> &left, const std::pair<double,double> &right) {
            return left.first < right.first;
        }
    };

    std::sort(line.begin(), line.end(), sortByFirst());
    std::vector<double> linesAngleClose(line.size());

    for(long i = 0; i != line.size(); i++) {
        for (int j = -4; j <= 3; j++) {
            if (j==0) continue;
            if (i+j == -1) continue;

            long iter = i+j;
            if (iter < 0) {
                iter = line.size() + iter;
                //linesAngleClose[i] += pow( (line[iterator].first - line[iterator+1].first), 2 );
            }
            linesAngleClose[i] += pow( (line[iter].first - line[iter+1].first), 2 );
            linesAngleClose[i] += pow( (line[iter].second - line[iter+1].second)/150, 2 );
        }
    }

    double bestLineVal = pow(10,100);
    long bestLineIndex = -1;
    for(long i = 0; i != line.size(); i++) {
        if (linesAngleClose[i] < bestLineVal) {
            bestLineIndex = i;
            bestLineVal = linesAngleClose[i];
        }
    }

    double secondBestLineVal = pow(10,100);
    long secondBestLineIndex = -1;
    for(long i = 0; i != line.size(); i++) {
        if ((linesAngleClose[i] < secondBestLineVal) && (fabs(line[i].first-line[bestLineIndex].first)) > M_PI / 6) {
            secondBestLineIndex = i;
            secondBestLineVal = linesAngleClose[i];
        }
    }

    std::vector<double> distanceToFirstLine( mc.size() );
    std::vector<double> distanceToSecondLine( mc.size() );

    std::vector<cv::Point2f> pointsOnFirstLine;
    std::vector<cv::Point2f> pointsOnSecondLine;

    for( int i = 0; i < mc.size(); i++ ) {
        if (bestLineIndex != -1) {
            double distanceToCenter = sqrt( pow(mc[i].x-centerX, 2) + pow(mc[i].y-centerY, 2) );
            double angleToCenter = atan2( centerY - mc[i].y, centerX - mc[i].x );
            distanceToFirstLine[i] = fabs( distanceToCenter * sin( line[bestLineIndex].first - angleToCenter ) - line[bestLineIndex].second );
            if (distanceToFirstLine[i] < 12) {
                pointsOnFirstLine.push_back( cv::Point2f(mc[i].x, mc[i].y) );
            }

            if (secondBestLineIndex != -1) {
                distanceToSecondLine[i] = fabs( distanceToCenter * sin( line[secondBestLineIndex].first - angleToCenter ) - line[secondBestLineIndex].second );
                if (distanceToSecondLine[i] < 12) {
                    pointsOnSecondLine.push_back( cv::Point2f(mc[i].x, mc[i].y) );
                }
            }
        }
    }

    cv::Mat dst;
    dst = cv::Mat::zeros( 720, 1280, CV_8UC3 );

    //cv::inRange( hsvMat, cv::Scalar(95, 155, 70), cv::Scalar(125, 255, 255), sumMat );
    /*
         cv::Moments redMoments;
         redMoments = cv::moments( sumMat, true );

         double massCenterX = (redMoments.m10 / redMoments.m00 - frameWidth/2) * 200/717; // frameWidth total
         double massCenterY = (redMoments.m01 / redMoments.m00 - 717/2) * 200/717; // 717 total
         */

    /*
         double massCenterX = 0;
         double massCenterY = 0;
         double cvHeight = -1;

         if (contours.size() == 4) {
         massCenterX = ( (mc[0].x+mc[2].x)/2 - frameWidth/2 ) * 200/717;
         massCenterY = ( (mc[0].y+mc[2].y)/2 - 717/2 ) * 200/717;
         cvHeight = 500 / sqrt( pow(mc[0].x - mc[2].x, 2) + pow(mc[0].y - mc[2].y, 2) );
         }

         double pitchSignal = [self getRollWithXError:massCenterX];
         double rollSignal = -[self getPitchWithYError:massCenterY];

         double throttleSignal = 0.0;

         if (approach && (fabs(massCenterX) < 50) && (fabs(massCenterY) < 50)) {
         throttleSignal = - 5 / MAX(fabs(massCenterX), fabs(massCenterY));
         if (throttleSignal < -0.5) throttleSignal = -0.5;
         //if (sonarActive) {
         if (baroAltitude > 6) {
         throttleSignal = (6 - baroAltitude) * landingSpeedMultiplier;
         } else {
         throttleSignal = (6 - baroAltitude) * landingSpeedMultiplier;
         }
         //}
         if (throttleSignal < -0.5) throttleSignal = -0.5;
         }

         [self sendCommandWithPitch:pitchSignal Roll:rollSignal Yaw:0.0 Throttle:throttleSignal];

         self.testXLbl.text = [self dblToStr:massCenterX];
         self.testYLbl.text = [self dblToStr:massCenterY];
         */

    //origMat.copyTo(outMat, sumMat);

    cv::cvtColor( canny_output, outMat, cv::COLOR_GRAY2RGB );

    if (bestLineIndex != -1) {
        cv::line(outMat, cv::Point2f(frameWidth/2-line[bestLineIndex].second / sin(line[bestLineIndex].first) - 2000, 360 + 2000 * tan(line[bestLineIndex].first)), cv::Point2f(frameWidth/2-line[bestLineIndex].second / sin(line[bestLineIndex].first) + 2000, 360 - 2000 * tan(line[bestLineIndex].first)), cv::Scalar(255,100,0));
        if (secondBestLineIndex != -1) {
            cv::line(outMat, cv::Point2f(frameWidth/2-line[secondBestLineIndex].second/ sin(line[secondBestLineIndex].first) - 2000, 360 + 2000 * tan(line[secondBestLineIndex].first)), cv::Point2f(frameWidth/2-line[secondBestLineIndex].second / sin(line[secondBestLineIndex].first) + 2000, 360 - 2000 * tan(line[secondBestLineIndex].first)), cv::Scalar(0,255,0));
        }
    }

    for( int i = 0; i < mc.size(); i++ ) {
        cv::circle(outMat, cv::Point2f(mc[i].x, 720-mc[i].y), 4, meanColor[i], -1);
    }

    cv::Point2f firstLineCenter(0,0);
    for( int i = 0; i < pointsOnFirstLine.size(); i++ ) {
        cv::circle(outMat, cv::Point2f(pointsOnFirstLine[i].x, 720-pointsOnFirstLine[i].y), 7, cv::Scalar(255,0,0), 2);
        firstLineCenter.x += pointsOnFirstLine[i].x / pointsOnFirstLine.size();
        firstLineCenter.y += pointsOnFirstLine[i].y / pointsOnFirstLine.size();
    }

    cv::Point2f secondLineCenter(0,0);
    for( int i = 0; i < pointsOnSecondLine.size(); i++ ) {
        cv::circle(outMat, cv::Point2f(pointsOnSecondLine[i].x, 720-pointsOnSecondLine[i].y), 6, cv::Scalar(0,255,100), 2);
        secondLineCenter.x += pointsOnSecondLine[i].x / pointsOnSecondLine.size();
        secondLineCenter.y += pointsOnSecondLine[i].y / pointsOnSecondLine.size();
    }

    cv::circle(outMat, cv::Point2f(firstLineCenter.x, 720-firstLineCenter.y), 10, cv::Scalar(255,100,100), -1);
    cv::circle(outMat, cv::Point2f(secondLineCenter.x, 720-secondLineCenter.y), 10, cv::Scalar(100,255,100), -1);

    cv::Point2f aimCenter( (firstLineCenter.x+secondLineCenter.x) / 2, (firstLineCenter.y+secondLineCenter.y) / 2 );

    double massCenterX = 0;
    double massCenterY = 0;
    //double cvHeight = -1;

    if (bestLineIndex != -1) {
        massCenterX = ( firstLineCenter.x - frameWidth/2 ) * 200/720;
        massCenterY = ( 720 - firstLineCenter.y - 720/2 ) * 200/720;
    }
    //cvHeight = 500 / sqrt( pow(mc[0].x - mc[2].x, 2) + pow(mc[0].y - mc[2].y, 2) );

    double pitchSignal = [self getRollWithXError:massCenterX];
    double rollSignal = -[self getPitchWithYError:( massCenterY /* -35 */ )];
    /*
         double throttleSignal = 0.0;

         if (approach && (fabs(massCenterX) < 50) && (fabs(massCenterY) < 50)) {
         throttleSignal = - 5 / MAX(fabs(massCenterX), fabs(massCenterY));
         if (throttleSignal < -0.5) throttleSignal = -0.5;

         if (sonarActive) {
         if (sonarAltitude < 2) {
         // throttleSignal = (6 - baroAltitude) * landingSpeedMultiplier;
         pitchSignal = pitchSignal / 2 * sonarAltitude;
         }
         }
         if (throttleSignal < -0.5) throttleSignal = -0.5;
         }

         if (approach && sonarActive && sonarAltitude < 0.4) {
         throttleSignal = -0.25;
         pitchSignal = 0;
         rollSignal = 0;
         }

         if (bestLineIndex == -1) {
         pitchSignal = 0;
         rollSignal = 0;
         }
         */

    //double pitchSignal = 0;
    //double rollSignal = 0;

    if (prevPitchSignal < -20) prevPitchSignal = pitchSignal;
    if (prevRollSignal < -20) prevRollSignal = rollSignal;

    pitchSignal = pitchSignal / 3 + prevPitchSignal * 2 / 3;
    rollSignal = rollSignal / 3 + prevRollSignal * 2 / 3;

    prevPitchSignal = pitchSignal;
    prevRollSignal = rollSignal;

    double throttleSignal = 0.0;

    double massCenterDistFromZero = sqrt(pow(massCenterX, 2) + pow(massCenterY, 2));

    if (cvApproachAndHover && (massCenterDistFromZero < 30)) {
        // if (baroAltitude > 10) throttleSignal = - baroAltitude / 10;
        // else throttleSignal = - baroAltitude / 20;
        if (sonarActive) {
            throttleSignal = -0.07;

            /*
                         if ( sonarAltitude < cvTargetSonarHeight + 0.15 ) {
                         if ( cvTargetSonarHeight - sonarAltitude > 0.15 ) {
                         throttleSignal = 0.03;
                         } else {
                         throttleSignal = 0.0;
                         }
                         if (!cvStartedHovering) {
                         startedHovering = [NSDate date];
                         cvStartedHovering = true;
                         } else {
                         NSDate *currentDate = [NSDate date];
                         if ( [currentDate timeIntervalSinceDate:startedHovering] > cvHoverTime ) {
                         throttleSignal = 0.0;
                         apState = dotCVGoingUp;
                         cvStartedHovering = false;
                         }
                         }
                         }
                         */
        }
    }

    [self sendCommandWithPitch:pitchSignal Roll:rollSignal Yaw:0.0 Throttle:throttleSignal];

    cvRoll=rollSignal;
    cvPitch=pitchSignal;
    cvThrottle=throttleSignal;

    self.testXLbl.text = [self dblToStr:massCenterX];
    self.testYLbl.text = [self dblToStr:massCenterY];

    UIImage *uiImage1 = [cvConverter UIImageFromCVMat:outMat];

    self.testImageView.image = uiImage1;

    CVPixelBufferRelease(pbuf);
}

#endif
}
