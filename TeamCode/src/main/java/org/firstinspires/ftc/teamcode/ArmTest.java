package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Arm Tester", group="Testing_Servo")
public class ArmTest extends LinearOpMode {
    DcMotor slides;



    @Override
    public void runOpMode(){
        telemetry.addLine("Run for very slow increment (moves every 1 second.");
        telemetry.update();
        slides = hardwareMap.get(DcMotor.class, "slides");//change name to servo that is being tested.
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            slides.setPower(-gamepad1.right_stick_y/2.0);
        }
    }
}
