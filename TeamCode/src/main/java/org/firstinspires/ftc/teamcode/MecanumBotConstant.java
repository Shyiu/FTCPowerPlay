package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MecanumBotConstant {
    public static String fr = "frontRight";
    public static String fl = "frontLeft";
    public static String bl = "backLeft";
    public static String br = "backRight";
    public static String intake = "flapper";
    public static String right_servo = "left";
    public static String left_servo = "right";
    public static double TOP_HARDSTOP = 7350;
    public static double BOTTOM_HARDSTOP = -498;
    public static double[] SLIDE_POSITIONS = {BOTTOM_HARDSTOP, 2761, 5020, TOP_HARDSTOP};
    public static double[] CONE_STACK_LEVELS = {-498, -498, 350, 650, 854}; //Find these values
    public static String slides = "slides";
    public static String cameraApril = "Webcam 1";
    public static String cameraJunction = "Junction";
    public static String imu = "imu";
    public static String distance = "sensor_distance";
    public static String color = "sensor_color";
    public static double rightServoOpenPosition = -0.175, leftServoOpenPosition = -0.068;
    public static double rightServoClosedPosition = -.025, leftServoClosedPosition = -.23;
    public static double rightHardStopOut = -.575;
    public static double rightHardStopIn = .015;
    public static double leftHardStopOut = .422;
    public static double leftHardStopIn = -.28;
    public static String led = "led";
    public static String tape = "tape_motor";
}
