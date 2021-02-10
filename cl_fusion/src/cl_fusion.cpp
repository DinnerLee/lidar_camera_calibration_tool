#include "cl_fusion.h"

Fusion::Fusion(ros::NodeHandle node){
    rpyxyz.resize(6);

    sub_key = node.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, &Fusion::keycallback, this);
    scan_sub_ = node.subscribe<sensor_msgs::PointCloud2Ptr>("/pandar", 100, &Fusion::scanCallback, this);
    image_sub_ = node.subscribe("/camera/image_color", 1, &Fusion::subImgCallback, this); //sub camera

    cl_pub = node.advertise<sensor_msgs::PointCloud2> ("/lidar/clustering", 100, false); //Object Clustering data AD

    cl_roll = node.advertise<std_msgs::Float32>("/cali/roll", 100, false);
    cl_pitch = node.advertise<std_msgs::Float32>("/cali/pitch", 100, false);
    cl_yaw = node.advertise<std_msgs::Float32>("/cali/yaw", 100, false);
    cl_x = node.advertise<std_msgs::Float32>("/cali/x", 100, false);
    cl_y = node.advertise<std_msgs::Float32>("/cali/y", 100, false);
    cl_z = node.advertise<std_msgs::Float32>("/cali/z", 100, false);

    img_pub = node.advertise<sensor_msgs::Image>("/cali/image", 100, false);
}
void Fusion::keycallback(const geometry_msgs::Twist::ConstPtr &msg){
    if(msg->linear.x == 0.5 && msg->angular.z == 1){rpyxyz.at(0) += 0.00174533;}
    if(msg->linear.x == 1.0 && msg->angular.z == 1){rpyxyz.at(1) += 0.00174533;}
    if(msg->linear.x == 1.5 && msg->angular.z == 1){rpyxyz.at(2) += 0.00174533;}
    if(msg->linear.x == 0.5 && msg->angular.z == 0){rpyxyz.at(3) += 0.00174533;}
    if(msg->linear.x == 1.0 && msg->angular.z == 0){rpyxyz.at(4) += 0.00174533;}
    if(msg->linear.x == 1.5 && msg->angular.z == 0){rpyxyz.at(5) += 0.00174533;}\

    if(msg->linear.x == -0.5 && msg->angular.z == 1){rpyxyz.at(0) += -0.00174533;}
    if(msg->linear.x == -1.0 && msg->angular.z == 1){rpyxyz.at(1) += -0.00174533;}
    if(msg->linear.x == -1.5 && msg->angular.z == 1){rpyxyz.at(2) += -0.00174533;}
    if(msg->linear.x == -0.5 && msg->angular.z == 0){rpyxyz.at(3) += -0.00174533;}
    if(msg->linear.x == -1.0 && msg->angular.z == 0){rpyxyz.at(4) += -0.00174533;}
    if(msg->linear.x == -1.5 && msg->angular.z == 0){rpyxyz.at(5) += -0.00174533;}
}

void Fusion::subImgCallback(const sensor_msgs::Image &subImgMsgs){
    rawImagePtr = cv_bridge::toCvCopy(subImgMsgs, sensor_msgs::image_encodings::BGR8);
    rawImage = rawImagePtr->image;
    rawImagePtr->image = rawImage;
    init();
}

void Fusion::scanCallback(const sensor_msgs::PointCloud2Ptr scan)
{
    PPointCloud arr; pcl::PointCloud<pcl::PointXYZI> LiDAR_Point;
    pcl::fromROSMsg(*scan, arr); LiDAR_Point = New_PointCloud(arr);
    raw_point = LiDAR_Point;
}

pcl::PointCloud<pcl::PointXYZI> Fusion::New_PointCloud(PPointCloud arr){
    pcl::PointCloud<pcl::PointXYZI> new_point;
    for(int i = 0; i < arr.size(); i++){
        if(arr.at(i).intensity >= 230){
        pcl::PointXYZI pt;
        pt._PointXYZI::x = arr.at(i).x; pt._PointXYZI::y = arr.at(i).y; pt._PointXYZI::z = arr.at(i).z; pt._PointXYZI::intensity = arr.at(i).intensity;
        new_point.push_back(pt);
        }
    } return new_point;
}

void Fusion::init(){
    cv::Mat undist; cv::Mat Fusion_point;
    pcl::PointCloud<pcl::PointXYZI> roi_point;

    roi_point = Passthrough_ob(raw_point, 0, 50, -30, 30);
    undist = undistortion_image(rawImage);
    Fusion_point = CameraLidar_Fusion(undist, roi_point);

    cv::imshow("fusion", Fusion_point);
    cv::waitKey(4);

    sensor_msgs::ImagePtr msg;
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Fusion_point).toImageMsg();
    img_pub.publish(msg);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(roi_point, output);
    sensor_msgs::PointCloud output_arr;
    sensor_msgs::convertPointCloud2ToPointCloud(output, output_arr);
    output.header.frame_id = "Pandar40M";
    cl_pub.publish(output);
}

pcl::PointCloud<pcl::PointXYZI> Fusion::Passthrough_ob(pcl::PointCloud<pcl::PointXYZI> point, double minx, double maxx, double miny, double maxy){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>); pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> filter;
    pcl::PassThrough <pcl::PointXYZI> pass;

    *cloud = point;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2, 1);
    pass.filter(*cloud_filter);
    pass.setInputCloud(cloud_filter);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(minx, maxx);
    pass.filter(*cloud_filter);
    pass.setInputCloud(cloud_filter);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(miny, maxy);
    pass.filter(*cloud_filter);
    filter = *cloud_filter;

    return filter;
}

cv::Mat Fusion::undistortion_image(cv::Mat img){
    cv::Mat output;
    cv::Mat Camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat DistCoeffs = cv::Mat::zeros(1, 5, CV_64FC1);

    Camera_matrix=(cv::Mat1d(3,3) << 1806.820309, 0.000000, 1033.711216,
                                     0.000000, 1805.927347, 816.626313,
                                     0.000000, 0.000000, 1.000000);
    DistCoeffs=(cv::Mat1d(1,5) << -0.137354, 0.057845, 0.000487, 0.002065, 0.000000);

    cv::undistort(img, output, Camera_matrix, DistCoeffs);

    return output;
}

cv::Mat Fusion::CameraLidar_Fusion(cv::Mat img, pcl::PointCloud<pcl::PointXYZI> point){
    cv::Mat output;
    Eigen::MatrixX3f Camera_matrix = Eigen::Matrix3f::Identity();
    Camera_matrix << 1806.820309, 0.000000, 1033.711216,
                     0.000000, 1805.927347, 816.626313,
                     0.000000, 0.000000, 1.000000;
 
    Eigen::MatrixXf H(3, 4);
    H << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0;

    float rot_x = 3.148333+rpyxyz.at(0); float rot_y = -1.547270+rpyxyz.at(1); float rot_z = -1.581279+rpyxyz.at(2);
    float x = -0.008913+rpyxyz.at(3); float y = -0.134951+rpyxyz.at(4); float z = 0.250173+rpyxyz.at(5);

    std_msgs::Float32 msg_roll;
    msg_roll.data = rot_x;
    cl_roll.publish(msg_roll);
    std_msgs::Float32 msg_pitch;
    msg_pitch.data = rot_y;
    cl_pitch.publish(msg_pitch);
    std_msgs::Float32 msg_yaw;
    msg_yaw.data = rot_z;
    cl_yaw.publish(msg_yaw);
    std_msgs::Float32 msg_x;
    msg_x.data = x;
    cl_x.publish(msg_x);
    std_msgs::Float32 msg_y;
    msg_y.data = y;
    cl_y.publish(msg_y);
    std_msgs::Float32 msg_z;
    msg_z.data = z;
    cl_z.publish(msg_z);

    Eigen::Affine3f transf = pcl::getTransformation(x, y, z, rot_x, rot_y, rot_z);
    
    Eigen::Matrix4f Rotation;
    Rotation = transf.matrix();

    int maxx = 0; int maxy = 0;
    int minx = 2048; int miny = 1536;

    Eigen::MatrixXf OutPoint(3, 1);
    for(int i = 0; i < point.size(); i++){
        Eigen::MatrixXf Point(4, 1);
        Point(0) = point.at(i).x;
        Point(1) = point.at(i).y;
        Point(2) = point.at(i).z;
        Point(3) = 1;

        OutPoint = Camera_matrix * H * Rotation * Point;
        float x = OutPoint(0)/OutPoint(2);
        float y = OutPoint(1)/OutPoint(2);
        if((x < 2048 && x >= 0) && (y < 1536 && y >= 0)){
            img.at<cv::Vec3b>(y, x)[0] = 0; img.at<cv::Vec3b>(y, x)[1] = 0; img.at<cv::Vec3b>(y, x)[2] = 255;
            if(maxx < x){maxx = x;}
            if(minx > x){minx = x;}
            if(maxy < y){maxy = y;}
            if(miny > y){miny = y;}
        }
    }
    output = img;
//    cv::Mat roi;
//    if(maxx != 0){
//        roi = img(cv::Rect(minx, miny-40, maxx-minx, maxy - miny+40));
//        output = roi;
//    }
    
    return output;
}
