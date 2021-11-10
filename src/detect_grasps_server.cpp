#include <string>

#include <gpd/grasp_detector.h>

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "grasp_detect/GraspDetect.h"

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGBA> PCL_Cloud;

bool checkFileExists(const std::string &file_name)
{
    std::ifstream file;
    file.open(file_name.c_str());
    if (!file)
    {
        std::cout << "File " + file_name + " could not be found!\n";
        return false;
    }
    file.close();
    return true;
}

class DetectGraspGpdServer
{
private:
    ros::NodeHandle nh_;
    // ros::Publisher pub_;
    // ros::Subscriber sub_;
    ros::ServiceServer server_;

public:
    DetectGraspGpdServer()
    {
        server_ = nh_.advertiseService("/grasp_detecter/get_pose", &DetectGraspGpdServer::serverCallBack, this);
    }
    ~DetectGraspGpdServer() {}

    bool serverCallBack(grasp_detect::GraspDetect::Request &req, grasp_detect::GraspDetect::Response &res);
};

bool DetectGraspGpdServer::serverCallBack(grasp_detect::GraspDetect::Request &req, grasp_detect::GraspDetect::Response &res)
{
    // Read arguments from command line.
    PCL_Cloud::Ptr rgbdCloud(new PCL_Cloud);

    if (req.pcd_path == "")
    {
        pcl::fromROSMsg(req.point_cloud, *rgbdCloud);
        pcl::io::savePCDFileASCII("./poingcloud.pcd", *rgbdCloud);
        cout << "[info]"
             << "savePCD ./poingcloud.pcd";
    }
    else if (checkFileExists(req.pcd_path))
    {
        cout << "[info]"
             << "load " + req.pcd_path;
        cout << pcl::io::loadPCDFile(req.pcd_path, *rgbdCloud);
    }
    else
    {
        cout << "[error]"
             << "PCD file not found";
        return (-1);
    }

    std::string config_filename = "/home/waha/catkin_research_ws/src/gpd-ros/cfg/caffe_params.cfg";
    if (!checkFileExists(config_filename))
    {
        printf("Error: config file not found!\n");
        return (-1);
    }

    // Read parameters from configuration file.
    gpd::util::ConfigFile config_file(config_filename);
    config_file.ExtractKeys();

    // Set the camera position. Assumes a single camera view.
    std::vector<double> camera_position =
        config_file.getValueOfKeyAsStdVectorDouble("camera_position",
                                                   "0.0 0.0 0.0");
    Eigen::Matrix3Xd view_points(3, 1);
    view_points << camera_position[0], camera_position[1], camera_position[2];

    // Load point cloud from file.
    gpd::util::Cloud cloud(rgbdCloud, view_points);
    if (cloud.getCloudOriginal()->size() == 0)
    {
        std::cout << "Error: Input point cloud is empty or does not exist!\n";
        return (-1);
    }

    // Load surface normals from file.
    // if (argc > 3)
    // {
    //     std::string normals_filename = argv[3];
    //     cloud.setNormalsFromFile(normals_filename);
    //     std::cout << "Loaded surface normals from file: " << normals_filename
    //               << "\n";
    // }

    gpd::GraspDetector detector(config_filename);

    // Preprocess the point cloud.
    detector.preprocessPointCloud(cloud);

    // If the object is centered at the origin, reverse all surface normals.
    bool centered_at_origin =
        config_file.getValueOfKey<bool>("centered_at_origin", false);
    if (centered_at_origin)
    {
        printf("Reversing normal directions ...\n");
        cloud.setNormals(cloud.getNormals() * (-1.0));
    }

    // Detect grasp poses.
    std::vector<std::unique_ptr<gpd::candidate::Hand>> handles = detector.detectGrasps(cloud);

    int handle_count = 0;
    for (int i = 0; i < handles.size(); i++)
    {
        Eigen::Vector3d position = handles.at(i).get()->getPosition();
        auto rotation_matrix = handles.at(i).get()->getOrientation();
        Eigen::Vector3d euler = rotation_matrix.eulerAngles(2, 1, 0);

        Eigen::Quaterniond q(rotation_matrix);

        handle_count++;

        geometry_msgs::PoseStamped grasp_pose_msg;
        grasp_pose_msg.header.frame_id = req.point_cloud.header.frame_id;
        grasp_pose_msg.pose.position.x = position(0);
        grasp_pose_msg.pose.position.y = position(1);
        grasp_pose_msg.pose.position.z = position(2);

        double roll, pitch, yaw;
        roll = euler(2);
        pitch = euler(1);
        yaw = euler(0);

        tf2::Quaternion orientation;
        orientation.setRPY(roll, pitch, yaw);
        grasp_pose_msg.pose.orientation = tf2::toMsg(orientation);
        res.grasp_pose.push_back(tf2::toMsg(grasp_pose_msg));
    }

    if (handle_count == 0)
    {
        cout << "[warning]"
             << "no find handle!";
        return false;
    }

    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "detect_grasps_gpd_server");

    DetectGraspGpdServer calculatet_pose;
    ros::Duration(1).sleep();
    ROS_INFO("detect_grasps_gpd_server server is started");
    ros::spin();

    return 0;
}
