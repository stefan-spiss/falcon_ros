//
// Created by stefan spiss on 06.07.17.
//

#include <falcon_ros/FalconRosDriver.h>


FalconRosDriver::FalconRosDriver(ros::NodeHandle node, float loopRate, std::string positionTopic,
                                 std::string velocityTopic, std::string buttonsTopic, std::string forceSubTopic,
                                 bool forceOutput) :
        node(node), loopRate(loopRate), positionTopic(positionTopic), velocityTopic(velocityTopic),
        buttonsTopic(buttonsTopic), forceSubTopic(forceSubTopic), forceOutput(forceOutput),
        position(0.0f, 0.0f, 0.0f), velocity(0.0f, 0.0f, 0.0f), force(0.0f, 0.0f, 0.0f), buttons(4, 0),
        hapticLoop(true), hapticLoopFinished(false) {
    int initState = this->initFalcon();
    if(initState == 0) {
        std::cout << "Error, no device found!" << std::endl;
        std::exit(-1);
    } else if(initState == -1) {
        std::cout << "Error, could not open connection to device 0!" << std::endl;
        std::exit(-2);
    }

    this->position_pub = this->node.advertise<geometry_msgs::Vector3Stamped>(this->positionTopic.c_str(), 1);
    this->velocity_pub = this->node.advertise<geometry_msgs::Vector3Stamped>(this->velocityTopic.c_str(), 1);
    this->buttons_pub = this->node.advertise<std_msgs::Int8MultiArray>(this->buttonsTopic.c_str(), 1);

    this->force_sub = this->node.subscribe<geometry_msgs::Vector3>(this->forceSubTopic, 1,
                                                                   &FalconRosDriver::forceCallback, this);
}

FalconRosDriver::~FalconRosDriver() {
    //cleanUpFalcon();
}

int FalconRosDriver::initFalcon() {
    // create device handler
    handler = new chai3d::cHapticDeviceHandler();

    // get a handle to the first haptic device
    if (!handler->getDevice(hapticDevice, 0)) {
        return 0;
    }

    // open a connection to haptic device
    if (!hapticDevice->open()) {
        return -1;
    }

    // calibrate device (if necessary)
    hapticDevice->calibrate();

    // retrieve information about the current haptic device
    chai3d::cHapticDeviceInfo info = hapticDevice->getSpecifications();
    std::cout << "Connected to: " << info.m_manufacturerName << " " << info.m_modelName << std::endl;
}

void FalconRosDriver::setPosition(chai3d::cVector3d position) {
    this->position = position;
}

chai3d::cVector3d FalconRosDriver::getPosition() {
    return this->position;
}

void FalconRosDriver::setVelocity(chai3d::cVector3d velocity) {
    this->velocity = velocity;
}

chai3d::cVector3d FalconRosDriver::getVelocity() {
    return this->velocity;
}

void FalconRosDriver::setForceRendering(bool on) {
    forceOutput = on;
}

void FalconRosDriver::setButtons(int button1, int button2, int button3, int button4) {
    this->buttons[0] = button1;
    this->buttons[1] = button2;
    this->buttons[2] = button3;
    this->buttons[3] = button4;
}

std::vector<int> &FalconRosDriver::getButtons() {
    return this->buttons;
}

void FalconRosDriver::setForce(double x, double y, double z) {
    this->force.set(x, y, z);
}

chai3d::cVector3d FalconRosDriver::getForce() {
    return this->force;
}

void FalconRosDriver::stopHapticLoop() {
    hapticLoop = false;
}

bool FalconRosDriver::getHapticLoop() {
    return hapticLoop;
}

void FalconRosDriver::setHapticLoopFinished(bool state) {
    hapticLoopFinished = state;
}

bool FalconRosDriver::getHapticLoopFinished() {
    return hapticLoopFinished;
}

void FalconRosDriver::saturatAndSetForces(double x, double y, double z) {
    if (x < -MAX_FORCE) x = -MAX_FORCE;
    if (y < -MAX_FORCE) y = -MAX_FORCE;
    if (z < -MAX_FORCE) z = -MAX_FORCE;
    if (x > MAX_FORCE) x = MAX_FORCE;
    if (y > MAX_FORCE) y = MAX_FORCE;
    if (z > MAX_FORCE) z = MAX_FORCE;
    this->force.set(z, x, y);
}

void FalconRosDriver::forceCallback(const geometry_msgs::Vector3::ConstPtr &data) {
    this->saturatAndSetForces(data->x, data->y, data->z);
    forceConsumed = false;
}

const chai3d::cGenericHapticDevicePtr &FalconRosDriver::getHapticDevice() const {
    return hapticDevice;
}

bool FalconRosDriver::isForceOutput() const {
    return forceOutput;
}

void FalconRosDriver::setForceConsumed(bool forceConsumed) {
    FalconRosDriver::forceConsumed = forceConsumed;
}

void FalconRosDriver::publishFalconData() {

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok()) {
        geometry_msgs::Vector3Stamped pos;
        geometry_msgs::Vector3Stamped velocity;
        std_msgs::Int8MultiArray but;
        geometry_msgs::Vector3Stamped angles;

        pos.header.frame_id = velocity.header.frame_id = angles.header.frame_id = ros::this_node::getName();
        pos.header.stamp = velocity.header.stamp = angles.header.stamp = ros::Time::now();
        pos.vector.x = this->position.y();
        pos.vector.y = this->position.z();
        pos.vector.z = this->position.x();
        velocity.vector.x = this->velocity.y();
        velocity.vector.y = this->velocity.z();
        velocity.vector.z = this->velocity.x();

        but.data.push_back(buttons[0]);
        but.data.push_back(buttons[1]);
        but.data.push_back(buttons[2]);
        but.data.push_back(buttons[3]);

        this->position_pub.publish(pos);
        this->velocity_pub.publish(velocity);
        this->buttons_pub.publish(but);

        if(forceConsumed) {
            force.set(0.0f, 0.0f, 0.0f);
        }

        this->loopRate.sleep();
    }
    spinner.stop();
    hapticLoop = false;
}

void FalconRosDriver::startFalconRosNode() {
    hapticsThread = new chai3d::cThread();
    hapticsThread->start(falconCallback, chai3d::CTHREAD_PRIORITY_HAPTICS, &this);

    //atexit(cleanUpFalcon, &this);

    publishFalconData();

    cleanUpFalcon();
}

void FalconRosDriver::cleanUpFalcon() {
    while(!hapticLoopFinished) {
        chai3d::cSleepMs(100);
    }
    hapticDevice->close();
    delete(hapticThread);
    delete(handler);
}

void falconCallback(void *falconDriver) {
    FalconRosDriver *rosDriver = static_cast<FalconRosDriver *>(falconDriver);

    // main haptic simulation loop
    while (rosDriver->getHapticLoop()) {

        chai3d::cVector3d currentPos;
        chai3d::cVector3d currentVel;
        
        // read position
        rosDriver->getHapticDevice()->getPosition(currentPos);

        // read linear velocity
        rosDriver->getHapticDevice()->getLinearVelocity(currentVel);

        // read user-switch status (button 0)
        bool button0, button1, button2, button3;
        button0 = false;
        button1 = false;
        button2 = false;
        button3 = false;

        rosDriver->getHapticDevice()->getUserSwitch(0, button0);
        rosDriver->getHapticDevice()->getUserSwitch(1, button1);
        rosDriver->getHapticDevice()->getUserSwitch(2, button2);
        rosDriver->getHapticDevice()->getUserSwitch(3, button3);

        rosDriver->setPosition(currentPos);
        rosDriver->setVelocity(currentVel);
        rosDriver->setButtons(button0, button1, button2, button3);

        /**
         * Apply forces
         */
        if (rosDriver->isForceOutput()) {
            //std::cout << "Force: " << rosDriver->getForce() << std::endl;
            rosDriver->getHapticDevice()->setForce(rosDriver->getForce());
            rosDriver->setForceConsumed(true);
        } else {
            rosDriver->getHapticDevice()->setForce(chai3d::cVector3d(0.0f, 0.0f, 0.0f));
        }
    }
    rosDriver->setHapticLoopFinished(true);
}
