package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Wall Detection + Rumble Check", group = "Testing_Servo")
@Disabled
public class Wall_Detection extends LinearOpMode {

    // Initializes all four motors used in the drive train
    // This code will be using a Tank-drive system
    ModernRoboticsI2cRangeSensor rangeSensor;
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            if (rangeSensor.getDistance(DistanceUnit.CM) < 4){
                gamepad1.rumble(1 - rangeSensor.getDistance(DistanceUnit.CM)/4.0,1 - rangeSensor.getDistance(DistanceUnit.CM)/4.0,10);
            }
            if((int)(getRuntime()) == 15){
                gamepad1.rumble(1,1,250);
                gamepad2.rumble(1,1,250);
                gamepad1.rumble(1,1,250);
                gamepad2.rumble(1,1,250);
                gamepad2.rumble(1,1,250);
                gamepad1.rumble(1,1,250);
            }
            telemetry.update();
        }
    }

}
