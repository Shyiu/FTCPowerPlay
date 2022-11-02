package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Motor Testing", group="Testing_Servo")
public class Testing_Motor extends LinearOpMode {
    DcMotor testingServo;
    Boolean a;//changes servo position by -0.1
    Boolean b;//changes servo position by -0.01
    Boolean y;//changes servo position by  0.01
    Boolean x;//changes servo position by  0.1
    //1540 1008 86

    @Override
    public void runOpMode(){
        telemetry.addLine("Gamepad1 Controls:\nLeft Stick Y Moves Motor");
        telemetry.update();
        testingServo = hardwareMap.get(DcMotor.class, "joint_motor");//change name to servo that is being tested.
        testingServo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testingServo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        testingServo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            testingServo.setPower(gamepad1.left_stick_y/3.0);
            telemetry.addLine("Gamepad1 Controls:\nLeft Stick Y Moves Motor");
            telemetry.addData("Servo Position", testingServo.getCurrentPosition());
            telemetry.update();
        }
    }
}
