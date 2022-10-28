package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Manual Mecanum Drive", group = "Mecanum Code")
public class ManualMecanumDrive extends LinearOpMode {


    //initialize motors
    DcMotor frontRight, frontLeft, backRight, backLeft, armJoint1;

    @Override
    public void runOpMode() {

        waitForStart();
        if (isStopRequested()) return;



        //Motors controlled by Game Controller 1
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        //DcMotor motorFrontRight = hardwareMap.get(DcMotor.class, MecanumHardware.fr);
        //DcMotor motorFrontLeft = hardwareMap.get(DcMotor.class, MecanumHardware.fl);
        //DcMotor motorBackRight = hardwareMap.get(DcMotor.class, MecanumHardware.br);
        //DcMotor motorBackLeft = hardwareMap.get(DcMotor.class, MecanumHardware.bl);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Motors controlle by Game Controller 1
        DcMotor armJoint1 = hardwareMap.get(DcMotor.class, "joint_motor");
        Servo armJoint2 = hardwareMap.get(Servo.class, "joint_servo");
        Servo claw = hardwareMap.get(Servo.class, "claw_servo");

        armJoint1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double clawOpen = .16;
        double clawClose = -.5;
        double lockJoint2 = 0;

        claw.setPosition(clawClose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //Function of Game Controller 1
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double y2 = -gamepad2.left_stick_y;
            Servo.Direction direction = Servo.Direction.FORWARD;

            double positionSave = 0;
            double joint2Position;
            if (lockJoint2 == 0) {
                joint2Position = gamepad2.right_trigger;
            }else{
                joint2Position = positionSave;
            }
            if (gamepad2.a){
                positionSave = gamepad2.right_trigger;
                if (lockJoint2 == 0){
                    lockJoint2 = 1;
                    sleep(30);
                }
                else{
                    lockJoint2 = 0;
                    sleep(30);
                }
            }




            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;




            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            //Function of Game Controller 2


            armJoint1.setPower(Math.pow(y2,2));
            if (Math.abs(y2) > 0.25){
                armJoint2.setPosition(armJoint2.getPosition()-(y2/10.0));
            }
            else{
                armJoint2.setPosition(joint2Position);
            }

            claw.setDirection(direction);
            if (gamepad2.y) {
                claw.setPosition(clawOpen);
            }
            else if(gamepad2.x){
                claw.setPosition(clawClose);
            }

            telemetry.addData("Servo Power" , claw.getPosition());
            telemetry.update();


        }
        }


    public double signedSquare(double num) {
        if (num > 0) {
            return (num*num);
        }
        else {
            return (-1*num*num);
        }
    }

    public double signedSquareRoot (double num) {
        if (num > 0) {
            return (Math.pow(num, 0.5));
        } else {
            return (-Math.pow(-num, 0.5));
        }
    }
}