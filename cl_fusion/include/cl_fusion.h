#include "point_types.h"

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <pcl/common/transformation_from_correspondences.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/Twist.h>



class Fusion
{
private:
    ros::NodeHandle node;
    ros::Subscriber scan_sub_;
    ros::Subscriber image_sub_;
    ros::Subscriber sub_key;
    ros::Publisher cl_pub;
    ros::Publisher cl_roll; ros::Publisher cl_pitch; ros::Publisher cl_yaw;
    ros::Publisher cl_x; ros::Publisher cl_y; ros::Publisher cl_z;
    ros::Publisher img_pub;

    pcl::PointCloud<pcl::PointXYZI> raw_point;
    cv_bridge::CvImagePtr rawImagePtr;
    cv::Mat rawImage;
    std::vector<float> rpyxyz;
    int count;
public:
    Fusion(ros::NodeHandle node);
    void scanCallback(const sensor_msgs::PointCloud2Ptr scan); //lidar callback
    void subImgCallback(const sensor_msgs::Image& subImgMsgs); //camera callback
    void keycallback(const geometry_msgs::Twist::ConstPtr &msg);

    pcl::PointCloud<pcl::PointXYZI> New_PointCloud(PPointCloud arr);
    pcl::PointCloud<pcl::PointXYZI> Passthrough_ob(pcl::PointCloud<pcl::PointXYZI> point, double minx, double maxx, double miny, double maxy);

    void init();
    cv::Mat undistortion_image(cv::Mat img);
    cv::Mat CameraLidar_Fusion(cv::Mat img, pcl::PointCloud<pcl::PointXYZI> point);
};
