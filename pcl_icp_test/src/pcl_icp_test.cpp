#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
using namespace std; //hxz

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);//输入点云
pcl::PointCloud<pcl::PointXYZ> cloud_in_after_icp ;//输入点云进过icp匹配后的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);//目标点云
sensor_msgs::PointCloud2 targetCloudMsg,sourceCloudMsg,aftericpCloudMsg;//
ros::Publisher  pubTargetCloud,pubSourceCloud,pubAfterIcpCloud;


// 生成点云  函数
void create_cloud(){
    // 输入点云-------------------------------------------------------------------------------------------------------
    cloud_in->width    = 50;
    cloud_in->height   = 1;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);
    for (int i = 0; i < cloud_in->points.size (); ++i)
    {
    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    // cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].z = 0;//放在一个平面上，方便观察
    }
    pcl::toROSMsg(*cloud_in, sourceCloudMsg);
    sourceCloudMsg.header.stamp = ros::Time::now();
    sourceCloudMsg.header.frame_id = "map";
    std::cout<<"输入点云创建成功"<<std::endl;

    // 目标点云------------------------------------------------------
    cloud_out->points.resize (cloud_in->width * cloud_in->height);
    for (int i = 0; i < cloud_in->points.size (); ++i){
        cloud_out->points[i].x = cloud_in->points[i].x + 2.0f;//简单的刚体变换，每个x都加了5
        cloud_out->points[i].y = cloud_in->points[i].y + 2.0f;//简单的刚体变换，每个x都加了5
        // cloud_out->points[i].z = cloud_in->points[i].z + 0.5f;//简单的刚体变换，每个x都加了0.5
        cloud_out->points[i].z = 0;////放在一个平面上，方便观察
    }
    pcl::toROSMsg(*cloud_out, targetCloudMsg);
    targetCloudMsg.header.stamp = ros::Time::now();
    targetCloudMsg.header.frame_id = "map";
    std::cout<<"目标点云转换成功"<<std::endl;
}

// icp匹配函数
void icp_match(){

    //创建一个IterativeClosestPoint实例，使用的奇异值分解
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);//迭代次数已达到用户施加的最大迭代次数
    icp.setTransformationEpsilon(1e-6);//先前转换和当前估计转换（即两次位姿转换）之间的 epsilon（差异）小于用户施加的值
    icp.setEuclideanFitnessEpsilon(1e-6);//欧几里得平方误差的总和小于用户定义的阈值
    icp.setRANSACIterations(0);// 设置RANSAC运行次数    
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    //cloud_in_after_icp用来存储应用ICP算法之后的结果
    icp.align(cloud_in_after_icp);

    pcl::toROSMsg(cloud_in_after_icp, aftericpCloudMsg);
    aftericpCloudMsg.header.stamp = ros::Time::now();
    aftericpCloudMsg.header.frame_id = "map";
    std::cout<<"发布icp转换后点云"<<std::endl;

    //如果变换前后点云正确Align的话（即变换点云通过刚性变换之后几乎和变换后点云完全重合）
    //则 icp.hasConverged() = 1 (true)，然后输出fitness得分和其他一些相关信息。
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;//获得最后的变换矩阵

}

int  main (int argc, char** argv)
{
    ros::init(argc, argv, "icp_match");
    ros::NodeHandle nh("~");

    create_cloud();
    icp_match();

    pubSourceCloud = nh.advertise<sensor_msgs::PointCloud2>("/source_cloud", 1);//
    pubTargetCloud = nh.advertise<sensor_msgs::PointCloud2>("/target_cloud", 1);//
    pubAfterIcpCloud = nh.advertise<sensor_msgs::PointCloud2>("/after_icp_cloud", 1);//

    ros::Rate rate_10hz(10);  
    while(ros::ok()){
        pubSourceCloud.publish(sourceCloudMsg);
        pubTargetCloud.publish(targetCloudMsg);
        pubAfterIcpCloud.publish(aftericpCloudMsg);
        
        rate_10hz.sleep();
    }

    ros::shutdown();
    return (0);
}