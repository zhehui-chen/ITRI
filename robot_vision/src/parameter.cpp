#include <parameter.h>

string id;
bool fg_existingImages;
string path_image_raw;
string path_existImage;

template <typename T>
T readParam(ros::NodeHandle &nh, string name)
{
    T ans;
    if (nh.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_INFO_STREAM("Failed to load " << name);
        nh.shutdown();
    }
    return ans;
}

// if you want to use existing image as input
// keyword: !
void readParameters(ros::NodeHandle &nh)
{
    id = readParam<string>(nh, "id");

    fg_existingImages = readParam<bool>(nh, "fg_existingImages");

    path_image_raw = readParam<string>(nh, "path_image_raw");

    path_existImage = readParam<string>(nh, "path_existImage"); // !
}
