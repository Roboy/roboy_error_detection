enum NotificationCode {
    UNDEFINED_NOTIFICATION = 0,
    MOTOR_DEAD_NOTIFICATION,
    MOTOR_ALIVE_NOTIFICATION
};

std::map<NotificationCode, string> notificationMessages = {
        {UNDEFINED_NOTIFICATION, "undefined"},
        {MOTOR_DEAD_NOTIFICATION, "Motor is dead, please check first all cables"},
        {MOTOR_ALIVE_NOTIFICATION, "Motor works fine"}
};