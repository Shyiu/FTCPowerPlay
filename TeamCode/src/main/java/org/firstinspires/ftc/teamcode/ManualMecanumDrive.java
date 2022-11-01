package org.firstinspires.ftc.teamcode;
//hi
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
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
        double clawOpen = .78;
        double clawClose = 1;
        double lockJoint2 = 0;
        double armUp = 1;
        double armDown = 0;
        double armJoint2Max = 1;
        double armJoint2Min = 0;
        double armJoint2Increment = .05;
        double y,x,rx,y2;
        double denominator,frontLeftPower,backLeftPower,frontRightPower,backRightPower;
        double incrementWait = .5;
        double armJoint2CurrentPos = armJoint2.getPosition();
        claw.setPosition(clawClose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //Function of Game Controller 1
            y = -gamepad1.left_stick_y; // Remember, this is reversed!
            x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            rx = gamepad1.right_stick_x;
            y2 = -gamepad2.left_stick_y;
            Servo.Direction direction = Servo.Direction.FORWARD;





            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;




            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            //Function of Game Controller 2
            armJoint1.setPower(signedSquare(y2));
            if((gamepad2.left_trigger) > 0){
                armJoint2CurrentPos = armJoint2.getPosition();
                if (armJoint2CurrentPos + armJoint2Increment <= armJoint2Max){
                    armJoint2CurrentPos += armJoint2Increment;
                }
                else{
                    armJoint2CurrentPos = armJoint2Max;
                }

                incrementWait = 1.001 - gamepad2.left_trigger;
                armJoint2.setPosition(armJoint2CurrentPos);
                sleep((long)(incrementWait*900));
            }
            else if((gamepad2.right_trigger) > 0){
                armJoint2CurrentPos = armJoint2.getPosition();
                if (armJoint2CurrentPos - armJoint2Increment >= armJoint2Min){
                    armJoint2CurrentPos -= armJoint2Increment;
                }
                else{
                    armJoint2CurrentPos = armJoint2Min;
                }

                incrementWait = 1.001 - gamepad2.right_trigger;
                armJoint2.setPosition(armJoint2CurrentPos);
                sleep((long)(incrementWait*900));
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