#include <process.h>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

void findLocation::convert_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

    findcontours(cv_ptr->image);
}

void findLocation::img2rviz(Mat img)
{
    cv_bridge::CvImage img_out;
    img_out.encoding = sensor_msgs::image_encodings::BGR8;
    img_out.image = img;
    pub_image.publish(img_out.toImageMsg());
}

void findLocation::output(Mat drawing, vector<Point2f> mc)
{
    imshow(id + "Contours Image",drawing); // show contour in screen
    waitKey(5);
    img2rviz(drawing); // show image in rviz
    geometry_msgs::Pose2D msg;
    msg.x = mc[0].x;
    msg.y = mc[0].y;
    pub_image_pos.publish(msg); // pub center points to topic
}

void findLocation::findcontours(Mat srcimg)
{
    Mat img;
    vector<vector<Point> > g_vContours;
    vector<Vec4i> g_vHierarchy;

    Canny(srcimg, img, 100, 255, 3);
    findContours(img, g_vContours, g_vHierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<Moments> mu(g_vContours.size());                                                                   //计算矩
    for (unsigned int i = 0; i < g_vContours.size(); i++)
    {
        mu[i] = moments(g_vContours[i], false);
    }
    vector<Point2f> mc(g_vContours.size());                                                                   //计算中心矩
    for (unsigned int i = 0; i < g_vContours.size(); i++)
    {
        mc[i] = Point2f(static_cast<float>(mu[i].m10 / mu[i].m00), static_cast<float>(mu[i].m01 / mu[i].m00));
    }

    Mat drawing = Mat::zeros(img.size(), CV_8UC3);                                                         //绘制轮廓
    for (unsigned int i = 0; i < g_vContours.size(); i++)
    {
        Scalar color = Scalar(255, 0, 0);
        drawContours(drawing, g_vContours, i, color, 1, 8, g_vHierarchy, 0, Point());                         //绘制外层和内层轮廓
//            circle(drawing, mc[i], 4, color, -1, 8, 0);
//            printf("\t %d contour: x=%f y=%f\n", i, mc[i].x, mc[i].y);                                            //输出内容
    }

    output(drawing, mc);
}

// the following may be not executable
/*
    void findcontours(Mat srcimg)
    {
        vector<Vec3f> circles;
        HoughCircles(srcimg, circles, CV_HOUGH_GRADIENT, 1, srcimg.rows / 5, 100, 50, 0, 100);

        Mat drawing = Mat::zeros(srcimg.size(), CV_8UC3);                                                         //绘制轮廓
        for (size_t i = 0; i < circles.size(); i++)
        {
            //提取出圓心座標
            Point center(round(circles[i][0]), round(circles[i][1]));
            //提取出圓半徑
            int radius = round(circles[i][2]);
            //圓心
            circle(drawing, center, 3, Scalar(0, 255, 0), -1, 4, 0);
            //圓
            circle(drawing, center, radius, Scalar(0, 0, 255), 3, 4, 0);

            //publish center of circle
            geometry_msgs::Pose2D msg;
            msg.x = center.x;
            msg.y = center.y;
            image_pub_pos_.publish(msg);
        }

        imshow("Contours Image", drawing);
        waitKey(5);
    }
*/
