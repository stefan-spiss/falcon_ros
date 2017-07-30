//
// Created by stefan spiss on 06.07.17.
//

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <memory>

#include <falcon_ros/FalconRosDriver.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "falcon_ros_node");
    ros::NodeHandle node;

    FalconRosDriver driver(node, 10.0, "/falcon/position", "/falcon/velocity", "/falcon/buttons", "/falcon/force", true);

/*    chai3d::cThread* hapticsThread = new chai3d::cThread();
    hapticsThread->start(falconCallback, chai3d::CTHREAD_PRIORITY_HAPTICS, &driver);

    boost::thread publisherThread(&FalconRosDriver::publishFalconData, &driver);

    publisherThread.join();

    delete(hapticsThread);
*/

    driver.startFalconRosNode();

    ros::shutdown();
}
