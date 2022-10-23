package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Manual Mecanum Drive", group = "Mecanum Code")
public class ManualMecanumDrive extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();
        if (isStopRequested()) return;



        //Motors controlled by Game Controller 1
        DcMotor motorFrontRight = hardwareMap.get(DcMotor.class, MecanumHardware.fr);
        DcMotor motorFrontLeft = hardwareMap.get(DcMotor.class, MecanumHardware.fl);
        DcMotor motorBackRight = hardwareMap.get(DcMotor.class, MecanumHardware.br);
        DcMotor motorBackLeft = hardwareMap.get(DcMotor.class, MecanumHardware.bl);

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Motors controlle by Game Controller 1
        DcMotor armJoint1 = hardwareMap.get(DcMotor.class, "joint_motor");
        DcMotorSimple armJoint2 = hardwareMap.get(DcMotorSimple.class, "joint_servo");
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




            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            //Function of Game Controller 2


            armJoint1.setPower(Math.pow(y2,2));
            if (Math.abs(y2) > 0.25){
                armJoint2.setPower(armJoint2.getPower()-(y2/10.0));
            }
            else{
                armJoint2.setPower(joint2Position);
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