#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

class amcl_scan_remapper {
    private:
        ros::Subscriber 	laserFrontSubscriber;
        ros::Subscriber		laserBackSubscriber;

        ros::Publisher      laserCombined;
        ros::NodeHandle 	n;

    public:
        amcl_scan_remapper();
        //virtual ~amcl_scan_remapper() {}

        void laserFrontCallback(const sensor_msgs::LaserScan& laserFrontMsg);
        void laserBackCallback(const sensor_msgs::LaserScan& laserBackMsg);
};
