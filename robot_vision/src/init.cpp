#include "parameter.h"
#include "process.h"

#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

// if you want to use existing image as input
// keyword: !
void loadImage(ros::NodeHandle &nh, string path_image, string path_output)
{
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>(path_output,1);
    cv_bridge::CvImage img_bridge;
    img_bridge.encoding = sensor_msgs::image_encodings::MONO8;

    while(ros::ok())
    {
        Mat src = imread(path_image, IMREAD_GRAYSCALE);
        img_bridge.image = src;
        pub.publish(img_bridge.toImageMsg());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "location");
    ros::NodeHandle nh("~");

    readParameters(nh);

    if (fg_existingImages == true)
    {
        thread p{loadImage, ref(nh), path_existImage, path_image_raw}; // !
        p.detach(); // occur error: terminate called without an active exception
    }
    findLocation image(nh, id, path_image_raw);

    ros::spin();
}

