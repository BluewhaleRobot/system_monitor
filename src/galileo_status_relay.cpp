#include <ros/ros.h>
#include <galileo_serial_server/GalileoStatus.h>


ros::Publisher status_pub;

void pub_status(const galileo_serial_server::GalileoStatusConstPtr status){
    status_pub.publish(status);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "galileo_status_relay");
    ros::NodeHandle private_nh("~");
    status_pub = private_nh.advertise<galileo_serial_server::GalileoStatus>("/galileo/status", 10);
    ros::Subscriber chatter_sub = private_nh.subscribe("galileo/status", 10, pub_status);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    while (ros::ok())
    {
        sleep(1);
    }
}