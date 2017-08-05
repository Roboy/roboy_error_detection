void handleDebugNotification(const roboy_communication_control::DebugNotification::ConstPtr &msg);

void handleInfoNotification(const roboy_communication_control::InfoNotification::ConstPtr &msg);

void handleWarningNotification(const roboy_communication_control::WarningNotification::ConstPtr &msg);

void handleErrorNotification(const roboy_communication_control::ErrorNotification::ConstPtr &msg);

void handleDangerNotification(const roboy_communication_control::DangerNotification::ConstPtr &msg);

void sendMotorHealthStateMsg(ros::Publisher publisher, bool isMotorHealthy);

void initSystemNotificationSubscriber(ros::NodeHandlePtr nh);

void sendJointAngle(ros::Publisher publisher, uint32_t absAngle = 0, uint32_t relAngle = 0, uint32_t tacho = 0,
                    uint8_t agcGain = 0, bool tooFar = false, bool tooClose = false, int msgId = 0);