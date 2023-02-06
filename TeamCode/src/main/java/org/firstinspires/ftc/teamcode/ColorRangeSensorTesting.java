package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name="ColorRangeSensor", group="Testing_Servo")
public class ColorRangeSensorTesting extends LinearOpMode {
    ColorRangeSensor color;
    Rev2mDistanceSensor distance;
    @Override
    public void runOpMode() {
        color = hardwareMap.get(ColorRangeSensor.class, "color_range_sensor");
        distance = hardwareMap.get(Rev2mDistanceSensor.class, "distance_sensor");
        color.setGain(30);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Red", color.red());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("Green", color.green());
            telemetry.addData("Distance", color.getDistance(DistanceUnit.CM));
            telemetry.addData("aRGB?", color.argb());
            telemetry.addData("Distance sensor", distance.getDistance(DistanceUnit.CM));
            telemetry.update();



        }

    }

}
