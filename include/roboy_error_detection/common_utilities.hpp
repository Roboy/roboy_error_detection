enum NotificationCode {
    UNDEFINED_NOTIFICATION = 0,
    MOTOR_DEAD_NOTIFICATION,
    MOTOR_ALIVE_NOTIFICATION,
    JOINT_INVALID_REL_ANGLE_NOTIFICATION,
    MOTOR_IS_RUNNING_BUT_TENDENT_NOT_NOTIFICATION,
    JOINT_TOO_FAR_NOTIFICATION,
    JOINT_TOO_CLOSE_NOTIFICATION
};

std::map<NotificationCode, string> notificationMessages = {
        {UNDEFINED_NOTIFICATION, "undefined"},
        {MOTOR_DEAD_NOTIFICATION, "Motor is dead, please check first all cables"},
        {MOTOR_ALIVE_NOTIFICATION, "Motor works fine"},
        {JOINT_INVALID_REL_ANGLE_NOTIFICATION, "Joint has an invalid angle"},
        {MOTOR_IS_RUNNING_BUT_TENDENT_NOT_NOTIFICATION, "Motor is running but the corresponding tendent is not moving"},
        {JOINT_TOO_FAR_NOTIFICATION, "Joint magnet too far away"},
        {JOINT_TOO_CLOSE_NOTIFICATION, "Joint magnet too close"}
};

std::map<NotificationCode, string> notificationExtraMessages = {
        {UNDEFINED_NOTIFICATION, "undefined"},
        {MOTOR_DEAD_NOTIFICATION, "Please check all calbles"},
        {MOTOR_ALIVE_NOTIFICATION, "..."},
        {JOINT_INVALID_REL_ANGLE_NOTIFICATION, "..."},
        {MOTOR_IS_RUNNING_BUT_TENDENT_NOT_NOTIFICATION, "..."},
        {JOINT_TOO_FAR_NOTIFICATION, "..."},
        {JOINT_TOO_CLOSE_NOTIFICATION, "..."}
};