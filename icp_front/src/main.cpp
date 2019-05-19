#include "icp_front.h"

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "champion_nav_msgs/ChampionNavLaserScan.h"

#include "pcl-1.7/pcl/io/pcd_io.h"


class frontDebug
{
public:
    frontDebug()
    {
        m_FrameID = 0;

        m_laserscanSub = m_nh.subscribe("sick_scan",5,&frontDebug::championLaserScanCallback,this);
        m_odomPub      = m_nh.advertise<nav_msgs::Odometry>("front_odom", 5);

        m_laserOdometry.header.frame_id = "odom";
        m_laserOdometry.child_frame_id = "base_link";
        
        m_final_pose = Eigen::Matrix3d::Identity();
    }

    void ConvertChampionLaserScanToEigenPointCloud(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg,
                                                   Eigen::Vector3d odomPose,
                                                   std::vector<Eigen::Vector2d>& eigen_pts)
    {
        //变换矩阵
        Eigen::Matrix3d Tmatrix;
        Tmatrix << std::cos(odomPose(2)),-std::sin(odomPose(2)),odomPose(0),
                   std::sin(odomPose(2)), std::cos(odomPose(2)),odomPose(1),
                   0,0,1;

        //转换到里程计坐标系中．
        eigen_pts.clear();
        for(int i = 0; i < msg->ranges.size();i++)
        {
            // std::cout<<"msg->range_min: "<<msg->range_min<<std::endl;
            // std::cout<<"msg->range_max: "<<msg->range_max<<std::endl;

            if(msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
                continue;
           
            double lx = msg->ranges[i] * std::cos(msg->angles[i]);
            double ly = msg->ranges[i] * std::sin(msg->angles[i]);

            if(std::isnan(lx) || std::isinf(ly)||
               std::isnan(ly) || std::isinf(ly))
                continue;

            Eigen::Vector3d lpt(lx,ly,1.0);

            Eigen::Vector3d opt = Tmatrix * lpt;

            eigen_pts.push_back(Eigen::Vector2d(opt(0),opt(1)));
        }
        std::cout<<"Frame: "<<m_FrameID<<" lidar points size: "<<eigen_pts.size()<<std::endl;
    }

    void championLaserScanCallback(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg)
    {
        static bool isFirstFrame = true;
        m_FrameID++;

        Eigen::Vector3d nowPose;
        if(getOdomPose(msg->header.stamp,nowPose) == false)
        {
            std::cout <<"Failed to get Odom Pose"<<std::endl;
            return ;
        }

        if(isFirstFrame == true)
        {
            std::cout <<"First Frame"<<std::endl;
            isFirstFrame = false;
            m_prevLaserPose = nowPose;
            ConvertChampionLaserScanToEigenPointCloud(msg,m_prevLaserPose,prev_Pts);
            return ;
        }

        double delta_dist2 = std::pow(nowPose(0) - m_prevLaserPose(0),2) + std::pow(nowPose(1) - m_prevLaserPose(1),2);
        double delta_angle = std::fabs(tfNormalizeAngle(nowPose(2) - m_prevLaserPose(2)));

        if(delta_dist2 < 0.1 * 0.1 && delta_angle < tfRadians(5.0))
        {
            // std::cout <<"It's too close!!!"<<std::endl;
            return ;
        }

        m_prevLaserPose = nowPose;

        //转换到里程计坐标系 x,y
        std::vector<Eigen::Vector2d> nowPts;
        ConvertChampionLaserScanToEigenPointCloud(msg,nowPose,nowPts);

        //TODO
        //调用imls进行icp匹配，并输出结果．
        m_frontMatcher.setSourcePointCloud(nowPts);
        m_frontMatcher.setTargetPointCloud(prev_Pts);

        Eigen::Matrix3d final_pose, covariance;

        if(m_frontMatcher.Match(final_pose, covariance))
        {
            std::cout <<"Front match "<<m_FrameID<<" frame success!!!!"<<std::endl;          
        }
        else
        {
            std::cout <<"IMLS Match Failed!!!!"<<std::endl;
            return;
        }
        //end of TODO

        prev_Pts = nowPts;
        m_final_pose = m_final_pose * final_pose;
        odomPublisher(msg->header.stamp, m_final_pose);
        // m_laserscanSub.shutdown();
    }

    bool getOdomPose(ros::Time t, Eigen::Vector3d& pose)
    {
        // Get the robot's pose
        tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                                   tf::Vector3(0,0,0)), t, "/base_link");
        tf::Stamped<tf::Transform> odom_pose;
        try
        {
            m_tfListener.transformPose("/odom", ident, odom_pose);
        }
        catch(tf::TransformException e)
        {
            ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
            return false;
        }

        double yaw = tf::getYaw(odom_pose.getRotation());
        pose << odom_pose.getOrigin().x(), odom_pose.getOrigin().y(), yaw;

        return true;
    }

    void odomPublisher(ros::Time t, Eigen::Matrix3d& odom_pose)
    {
        double pos_x = odom_pose(0,2);
        double pos_y = odom_pose(1,2);
        double yaw = std::atan2(odom_pose(0,1),odom_pose(0,0));

        std::cout<<"Final position: "<<"("<<odom_pose(0,2)<<","<<odom_pose(1,2)<<")"<<std::endl;
        std::cout<<"Final angle: "<<yaw * 57.295<<std::endl;  

        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromYaw(yaw);

        m_laserOdometry.header.stamp = t;
        m_laserOdometry.pose.pose.orientation.x = geoQuat.x;
        m_laserOdometry.pose.pose.orientation.y = geoQuat.y;
        m_laserOdometry.pose.pose.orientation.z = geoQuat.z;
        m_laserOdometry.pose.pose.orientation.w = geoQuat.w;
        m_laserOdometry.pose.pose.position.x = pos_x;
        m_laserOdometry.pose.pose.position.y = pos_y;
        m_laserOdometry.pose.pose.position.z = 0;
        m_odomPub.publish(m_laserOdometry);
    }

    int m_FrameID;

    ros::NodeHandle m_nh;

    ICPMatcher m_frontMatcher;

    Eigen::Vector3d m_prevLaserPose;
    Eigen::Matrix3d m_final_pose;
    std::vector<Eigen::Vector2d> prev_Pts;

    tf::TransformListener m_tfListener;
    tf::TransformBroadcaster m_tfBroadcaster;
    nav_msgs::Odometry m_laserOdometry;
    ros::Subscriber m_laserscanSub;
    ros::Publisher m_pointcloudPub;
    ros::Publisher m_normalsPub;
    ros::Publisher m_odomPub;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "front_debug");

    frontDebug front_debug;

    ros::spin();

    return (0);
}

