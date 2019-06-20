#include<navigation_exercise/amcl_scan_remapper.h>

amcl_scan_remapper::amcl_scan_remapper()
{
    laserFrontSubscriber   = n.subscribe("/scan", 1, &amcl_scan_remapper::laserFrontCallback, this);
    laserBackSubscriber    = n.subscribe("/scan_back", 1, &amcl_scan_remapper::laserBackCallback, this);
    laserCombined          = n.advertise<sensor_msgs::LaserScan>("/scan_combined", 1, this);
}


void amcl_scan_remapper::laserFrontCallback(const sensor_msgs::LaserScan& laserFrontMsg)
{
    laserCombined.publish(laserFrontMsg);
}

void amcl_scan_remapper::laserBackCallback(const sensor_msgs::LaserScan& laserBackMsg)
{
    laserCombined.publish(laserBackMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "amcl_scan_remapper");

    amcl_scan_remapper remapper;

    ROS_INFO("amcl_scan_remapper started");

    ros::spin();
}
