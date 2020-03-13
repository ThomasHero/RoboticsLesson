//
// Created by thomas on 20-3-8.
//

#include "wallFollow.h"
#include "ros/ros.h"
#include "pid_core.h"

#include <iostream>
#include <fstream>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

PID_CORE_T pidCoreParm;

struct SensorLaserPoint {
    float angle;
    float range;
    float intensity;
    SensorLaserPoint &operator = (const SensorLaserPoint &data) {
        angle = data.angle;
        range = data.range;
        intensity = data.intensity;
        return *this;
    }
};

struct SensorLaserConfig{
    //! Start angle for the laser scan [rad].  0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
    float min_angle;
    //! Stop angle for the laser scan [rad].   0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
    float max_angle;
    //! Scan resoltuion [s]
    float time_increment;
    //! Time between scans
    float scan_time;
    //! Minimum range [m]
    float min_range;
    //! Maximum range [m]
    float max_range;
    //! fixed resolution size
    int fixed_size;
    SensorLaserConfig &operator = (const SensorLaserConfig &data) {
        min_angle = data.min_angle;
        max_angle = data.max_angle;
        time_increment = data.time_increment;
        scan_time = data.scan_time;
        min_range = data.min_range;
        max_range = data.max_range;
        fixed_size= data.fixed_size;
        return *this;
    }
};

struct SensorDataScan{
    std::vector<SensorLaserPoint> data;
    long long int systemTimeStamp;
    SensorLaserConfig config;
    SensorDataScan &operator = (const SensorDataScan &data){
        this->data = data.data;
        systemTimeStamp = data.systemTimeStamp;
        config = data.config;
        return *this;
    }
};


static int laserScanUpdate = 0;
static SensorDataScan scan;

ros::Publisher  pubCmd;
ros::Subscriber subScan;


float pid_core()
{

}


double getScanDistByAngle(const SensorDataScan& scan,
                          const double targetTheta,
                          const double angleRange,
                          const int filterMethod){
#define FILTER_TYPE_1 0

    if(laserScanUpdate == 1)
    {
        float angleIncreament = (scan.config.max_angle - scan.config.min_angle) / scan.config.fixed_size;
        int startIndex = (int)((targetTheta - scan.config.min_angle)/ angleIncreament);
        int endIndex = (int)(fabs(angleRange) / angleIncreament + startIndex);
        int midIndex = (int) (startIndex + endIndex) / 2;

        float startDist = constrain(scan.data.at(startIndex).range,0,scan.config.max_range);
        float midDist   = constrain(scan.data.at(midIndex).range,0,scan.config.max_range);
        float endDist   = constrain(scan.data.at(endIndex).range,0,scan.config.max_range);

        float averageDist = (startDist + midDist + endDist)/3.0;
        //printf("start:%f mid:%f end:%f fix size:%d\n",startDist,midDist,endDist,scan.data.size());
        printf("The start index:%d mid index:%d end index:%d, Dist:%f\n",startIndex,midIndex,endIndex,averageDist);
        return averageDist;

    }


}

void conviert_data(sensor_msgs::LaserScan::ConstPtr& msg, SensorDataScan& scanDest)
{
    //scanDest.config.fixed_size = msg.

}

void controlLoop(void){
    double averageDist = getScanDistByAngle(scan,M_PI_2,M_PI_4/8,0);

    float angleVel = pid_core_exe(&pidCoreParm,0.3,averageDist,0.1);

    printf("ang val:%f\n",angleVel);
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.25;
    cmd.angular.z = angleVel;
    pubCmd.publish(cmd);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    //printf("Get one scan pack!%d time:%d\n",msg->header.seq,msg->header.stamp.sec);
    //printf("ang_inc:%f ang_min:%f ang_max:%f rang_min:%f rang_max:%f time_inc:%f scan_time:%f\n",msg->angle_increment,\
    msg->angle_min,\
    msg->angle_max,\
    msg->range_min,\
    msg->range_max,\
    msg->time_increment,\
    msg->scan_time);

    laserScanUpdate = 1;
    scan.systemTimeStamp = msg->header.stamp.sec*1000000000 + msg->header.stamp.nsec;

    scan.config.max_angle      = msg->angle_max;
    scan.config.min_angle      = msg->angle_min;
    scan.config.scan_time      = msg->scan_time;
    scan.config.time_increment = msg->time_increment;
    scan.config.min_range      = msg->range_min;
    scan.config.max_range      = msg->range_max;
    scan.config.fixed_size     = msg->ranges.size();

    int lidarSize = msg->ranges.size();

    scan.data.resize(lidarSize);

    for(int i=0;i<msg->ranges.size();i++)
    {
        SensorLaserPoint laserPoint;

        if(msg->ranges[i] == 0.0)
        {
            laserPoint.range = std::numeric_limits<float>::infinity();
            laserPoint.intensity = 0;
            laserPoint.angle     = i*msg->angle_increment + msg->angle_min;
        }else{
            laserPoint.range     = msg->ranges[i];
            laserPoint.intensity = msg->intensities[i];
            laserPoint.angle     = i*msg->angle_increment + msg->angle_min;
        }

        scan.data.operator[](i) = laserPoint;
    }
    //printf("Lidar size:%d %d\n",lidarSize,scan.data.size());

}


void timer_callBack(const ros::TimerEvent& event)
{
    controlLoop();
}


void setPidParm(PID_CORE_T* pParm)
{
    pParm->kp = 5.0;
    pParm->ki = 0.0;
    pParm->kd_fdbk = 0.002;
}

int main(int argc, char **argv)
{

    pid_init(&pidCoreParm);
    setPidParm(&pidCoreParm);

    ros::init(argc, argv,"wallFollow");

    ros::NodeHandle n;

    subScan = n.subscribe("scan", 1, laserCallback);
    pubCmd  = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    //create a timer duration = 20ms
    ros::Timer timer = n.createTimer(ros::Duration(0.1),timer_callBack);

    ros::spin();

    return 0;
}








