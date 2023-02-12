package org.firstinspires.ftc.teamcode;

public final class MecanumBotConstant {
    public final String fr = "frontRight";
    public final String fl = "frontLeft";
    public final String bl = "backLeft";
    public final String br = "backRight";
    public final String intake = "flapper";
    public final String right_servo = "right";
    public final String left_servo = "left";
    public static double TOP_HARDSTOP = 7115;
    public static double BOTTOM_HARDSTOP = -339;//Actually supposed to be 0
    public static double[] SLIDE_POSITIONS = {BOTTOM_HARDSTOP, 2761, 5020, TOP_HARDSTOP};
    public final String slides = "slides";
    public final String cameraApril = "Webcam 1";
    public final String cameraJunction = "Webcam 2";
    public final String imu = "imu";
    public final String distance = "sensor_distance";
    public final String color = "sensor_color";
    public final double rightServoOpenPosition = -.221, leftServoOpenPosition = -.05;
    public final double rightServoClosedPosition = -.055, leftServoClosedPosition = -.175;
    public final double rightHardStopOut = -.721;
    public final double rightHardStopIn = 0.15;
    public final double leftHardStopOut = .5;
    public final double leftHardStopIn = -.35;
    public final String tape = "tape_motor";
}
