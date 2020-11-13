#include "itri/parameter.h"

std::string path_image1;
std::string path_image2;

std::string path_topic1;
std::string path_topic2;


cv::Mat r_12;
cv::Mat t_12;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string path_config_file;
    path_config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(path_config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["r_12"] >> r_12;
    fsSettings["t_12"] >> t_12;

    fsSettings["image1_path"] >> path_image1;
    fsSettings["image2_path"] >> path_image2;

    fsSettings["image1_topic"] >> path_topic1;
    fsSettings["image2_topic"] >> path_topic2;

    fsSettings.release();
}
