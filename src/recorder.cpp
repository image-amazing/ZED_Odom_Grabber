#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

#include <fstream>
#include <stdio.h>
#include <cstdio>
#include <cstring>
#include <signal.h>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <thread>
#include <mutex>
#include <iomanip>

//Opencv Include (for display
#include <opencv2/opencv.hpp>

//ZED Include
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

using namespace sl::zed;
using namespace std;

// Exchange structure
typedef struct image_bufferStruct {
    unsigned char* data_lim;
    std::mutex mutex_input_limage;

    unsigned char* data_rim;
    std::mutex mutex_input_rimage;

    int width, height, im_channels;
} image_buffer;

Camera* zed;
image_buffer* buffer;
SENSING_MODE dm_type = RAW;
ofstream out, time_out;
bool stop_signal;
int count_run=0;
bool newFrame=false;
bool init = false;
double starttime = 0.0;

void converter(const nav_msgs::Odometry::ConstPtr & msg)
{
    if(!init)
    {
        init = true;
        starttime = ros::Time::now().toSec();
    }
    cout << "Got a odom data at : " << ros::Time::now().toSec() - starttime << endl;
    out << ros::Time::now().toSec() - starttime<< " "
           << msg->pose.pose.position.x << " "
           << msg->pose.pose.position.y << " "
           << msg->pose.pose.orientation.w << endl;
}

// Grabbing function
void grab_run() {
//    float* p_depth;
    uchar* p_left;
    uchar* p_right;

#ifdef __arm__ //only for Jetson K1/X1
    sl::zed::Camera::sticktoCPUCore(2);
#endif

    while (!stop_signal)
    {

        if (!zed->grab(dm_type,1,1))
        {
        p_left = zed->retrieveImage(SIDE::LEFT).data; // Get the pointer
        p_right = zed->retrieveImage(SIDE::RIGHT).data; // Get the pointer

        if (count_run%100==0)
        {
        std::cout << "* Camera TimeStamp : " << zed->getCameraTimestamp()<< std::endl;
        long long current_ts = zed->getCurrentTimestamp();
        std::cout << "* Current TimeStamp : " <<  current_ts << std::endl;
        }

        // Fill the buffer
        buffer->mutex_input_limage.lock(); // To prevent from data corruption
        memcpy(buffer->data_lim, p_left, buffer->width * buffer->height * buffer->im_channels * sizeof (uchar));
        buffer->mutex_input_limage.unlock();

        buffer->mutex_input_rimage.lock();
        memcpy(buffer->data_rim, p_right, buffer->width * buffer->height * buffer->im_channels * sizeof (uchar));
        buffer->mutex_input_rimage.unlock();

        newFrame=true;
        count_run++;
        }
        else
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int main(int argc, char **argv) {
    stop_signal = false;

    out.open("/home/doom/odom.txt");
    time_out.open("/home/doom/time_svo.txt");

    ros::init(argc, argv, "recorder");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("odom", 1000, converter);

    zed = new Camera(HD720,30);

    int width = zed->getImageSize().width;
    int height = zed->getImageSize().height;

    ERRCODE err = zed->init(MODE::PERFORMANCE, -1, true);
    cout << errcode2str(err) << endl;
    if (err != SUCCESS) {
        delete zed;
        return 1;
    }

    // allocate data
    buffer = new image_buffer();
    buffer->height = height;
    buffer->width = width;
    buffer->im_channels = 4;
    buffer->data_lim = new uchar[buffer->height * buffer->width * buffer->im_channels];
    buffer->data_rim = new uchar[buffer->height * buffer->width * buffer->im_channels];

    cv::Mat left(height, width, CV_8UC4, buffer->data_lim, buffer->width * buffer->im_channels * sizeof (uchar));
    cv::Mat right(height, width, CV_8UC4, buffer->data_rim, buffer->width * buffer->im_channels * sizeof (uchar));

    // Run thread
    std::thread grab_thread(grab_run);
    char key = ' ';

    ros::Rate r(30);
    double last =ros::Time::now().toSec();
    int count = 0;
    while (ros::ok()) {
        if (newFrame)
        {
            newFrame=false; //indicates that we take care of this frame... next frame will be told by the grabbin thread.

            // Retrieve data from buffer
            buffer->mutex_input_limage.try_lock();
            memcpy(left.data, buffer->data_lim, buffer->width * buffer->height * buffer->im_channels * sizeof (uchar));
            buffer->mutex_input_limage.unlock();

            buffer->mutex_input_rimage.try_lock();
            memcpy(right.data, buffer->data_rim, buffer->width * buffer->height * buffer->im_channels * sizeof (uchar));
            buffer->mutex_input_rimage.unlock();

            // Do stuff
            std::stringstream ss;
            ss.str(""); ss << "/home/doom/zed/image_0/"
                              << std::setw(6) << std::setfill('0') << count << ".png";
            cv::imwrite(ss.str(), left);
            ss.str(""); ss << "/home/doom/zed/image_1/"
                              << std::setw(6) << std::setfill('0') << count << ".png";
            cv::imwrite(ss.str(), right);
            count++;

            double now =ros::Time::now().toSec();
            time_out << now - last << endl;

            ros::spinOnce();

            r.sleep();
        }
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(1));


    }

    // Stop the grabbing thread
    stop_signal = true;
    grab_thread.join();

    delete[] buffer->data_lim;
    delete[] buffer->data_rim;
    delete buffer;
    delete zed;

    out.close();
    return 0;
}
