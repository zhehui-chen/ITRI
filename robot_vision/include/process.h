#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;

class findLocation
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub_image_pos;

    image_transport::ImageTransport it;
    image_transport::Subscriber sub_image;
    image_transport::Publisher pub_image;

    string id;
    string path_image_raw;

public:
    findLocation(ros::NodeHandle& nodehandle, string identification, string path_sub):it(nodehandle), nh(nodehandle), id(identification)
    {
        path_image_raw = path_sub;
        sub_image = it.subscribe(path_image_raw, 1, &findLocation::convert_callback, this);
        pub_image_pos = nh.advertise<geometry_msgs::Pose2D>("position_center_of_mass", 1);
        pub_image = it.advertise("contour_output", 1);

        cv::namedWindow(id + "Contours Image");
    }
    ~findLocation()
    {
        cv::destroyWindow(id + "Contours Image");
    }

    void convert_callback(const sensor_msgs::ImageConstPtr& msg);
    void findcontours(Mat src);
    void output(Mat drawing, vector<Point2f> mc);
    void img2rviz(Mat img);
};

