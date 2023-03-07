package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MecanumBotConstant {
    public static String br = "backRight"; //Control Hub 0 20
    public static String fr = "frontRight";//Control Hub 1 20
    public static String bl = "backLeft";  //Control Hub 2 20
    public static String fl = "frontLeft"; //Control Hub 3 20
    public static String intake = "flapper";
    public static String left_servo = "right";//Control Hub 4
    public static String right_servo = "left";//Control Hub 5
    public static double TOP_HARDSTOP = 4300;
    public static double BOTTOM_HARDSTOP = -550;
    public static double[] SLIDE_POSITIONS = {BOTTOM_HARDSTOP, 1615, 3023, TOP_HARDSTOP};
    public static double[] CONE_STACK_LEVELS = {-498, -222, -65, 159, 310}; //Find these values
    public static String slides = "slides";//Expansion Hub 0  40
    public static String cameraApril = "Webcam 1";
    public static String cameraJunction = "Junction";
    public static String imu = "imu";
    public static String distance = "sensor_distance";
    public static String color = "sensor_color";//I2C 1 Control Hub Color/Range Sensor
    public static double rightServoOpenPosition = -0.175, leftServoOpenPosition = -0.068;
    public static double rightServoClosedPosition = -.025, leftServoClosedPosition = -.23;
    public static double rightHardStopOut = -.575;
    public static double rightHardStopIn = .015;
    public static double leftHardStopOut = .422;
    public static double leftHardStopIn = -.28;
    public static String led = "led";
    public static String tape = "tape_motor";
}
