#include "imls_icp.h"

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "champion_nav_msgs/ChampionNavLaserScan.h"

#include "pcl-1.7/pcl/io/pcd_io.h"

class imlsOdom
{
public:
    imlsOdom()
    {
        m_FrameID = 0;
        m_laserscanSub = m_nh.subscribe("sick_scan",5,&imlsOdom::championLaserScanCallback,this);
        
        m_trajectoryPub = m_nh.advertise<visualization_msgs::Marker>( "visualization_marker", 10);
        m_pointcloudPub = m_nh.advertise<sensor_msgs::PointCloud>("pointcloud", 1, true);
    }

    ~imlsOdom()
    {

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
    }

    void championLaserScanCallback(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg)
    {
        static bool isFirstFrame = true;
        m_FrameID++;
        std::cout<<"IMLS odom start with mather "<<m_FrameID -1 << " with "<< m_FrameID<<std::endl;
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

        if(delta_dist2 < 0.2 * 0.2 &&
           delta_angle < tfRadians(1.0)) //10.0
        {
            return ;
        }

        m_prevLaserPose = nowPose;

        //转换到里程计坐标系 x,y
        std::vector<Eigen::Vector2d> nowPts;
        ConvertChampionLaserScanToEigenPointCloud(msg,nowPose,nowPts);

        //TODO
        //调用imls进行icp匹配，并输出结果．
        m_imlsMatcher.setSourcePointCloud(nowPts);
        m_imlsMatcher.setTargetPointCloud(prev_Pts);

        Eigen::Matrix3d final_pose, covariance;

        if(m_imlsMatcher.Match(final_pose, covariance))
        {
            std::cout<<"IMLS odom success with mather "<<m_FrameID -1 << " with "<< m_FrameID<<std::endl;
            std::cout<<"Final position: "<<"("<<final_pose(0,2)<<","<<final_pose(1,2)<<")"<<std::endl;
            std::cout<<"Final angle: "<<std::atan2(final_pose(0,1),final_pose(0,0)) * 57.295<<std::endl;
            Eigen::Vector2d cur_pose;
            cur_pose << final_pose(0,2) , final_pose(1,2);
            publishMarker(cur_pose, &m_trajectoryPub);            
        }
        else
        {
            std::cout<<"IMLS odom failed with mather "<<m_FrameID -1 << " with "<< m_FrameID<<std::endl;
            return;
        }
        //end of TODO
        m_prevLaserPose = nowPose;
        ConvertChampionLaserScanToEigenPointCloud(msg,m_prevLaserPose,prev_Pts);
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
        pose << odom_pose.getOrigin().x(),odom_pose.getOrigin().y(),yaw;

        return true;
    }
private:
      void publishMarker(Eigen::Vector2d& cur_pose, ros::Publisher* pub)
      {
        static int marker_id = 0;
        m_marker.header.frame_id = "base_link";
        m_marker.header.stamp = ros::Time::now();
        m_marker.ns = "imls";
        m_marker.id = marker_id;
        m_marker.type = visualization_msgs::Marker::SPHERE;
        m_marker.action = visualization_msgs::Marker::ADD;
        m_marker.pose.position.x = cur_pose(0);
        m_marker.pose.position.y = cur_pose(1);
        m_marker.pose.position.z = 0;
        m_marker.pose.orientation.x = 0.0;
        m_marker.pose.orientation.y = 0.0;
        m_marker.pose.orientation.z = 0.0;
        m_marker.pose.orientation.w = 1.0;
        m_marker.scale.x = 0.1;
        m_marker.scale.y = 0.1;
        m_marker.scale.z = 0.1;
        m_marker.color.a = 1.0;
        m_marker.color.r = 0.0;
        m_marker.color.g = 1.0;
        m_marker.color.b = 0.0;
        m_marker.lifetime = ros::Duration();
        //only if using a MESH_RESOURCE m_marker type:

        pub->publish( m_marker );
        marker_id++;
    }
private:
    int m_FrameID;

    ros::NodeHandle m_nh;

    IMLSICPMatcher m_imlsMatcher;
    visualization_msgs::Marker m_marker;

    Eigen::Vector3d m_prevLaserPose;
    std::vector<Eigen::Vector2d> prev_Pts;

    tf::TransformListener m_tfListener;
    ros::Subscriber m_laserscanSub;
    ros::Publisher m_pointcloudPub;
    ros::Publisher m_trajectoryPub;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imls_odom");

    imlsOdom imls_odom;

    ros::spin();

    return (0);
}

