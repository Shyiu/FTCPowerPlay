package org.firstinspires.ftc.teamcode.obselete;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Disabled
@TeleOp(name="Slides Testing_Servo", group="Testing_Servo")
public class Slides_Testing extends LinearOpMode {
    DcMotor slides;
    DistanceSensor slides_distance;
    Boolean a;//changes servo position by -0.1
    Boolean b;//changes servo position by -0.01
    Boolean y;//changes servo position by  0.01
    Boolean x;//changes servo position by  0.1


    @Override
    public void runOpMode(){
        telemetry.addLine("Run for very slow increment (moves every 1 second.");
        telemetry.update();
        slides = hardwareMap.get(DcMotor.class, "slides");//change name to servo that is being tested.
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides_distance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            slides.setPower(-gamepad1.right_stick_y/2.0);
        }
    }
}
