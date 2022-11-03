package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TileRunner extends LinearOpMode {

    // Initializes all four motors used in the drive train
    // This code will be using a Tank-drive system
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;

    final int[] LEVELS = {86, 1008, 1540};
    final double[] SERVO_POS = {.58,1,.22,0};
    int servo_position = 0;
    int current_level = 0;
    @Override
    public void runOpMode() {
        DcMotor armJoint1 = hardwareMap.get(DcMotor.class, "joint_motor");
        Servo armJoint2 = hardwareMap.get(Servo.class, "joint_servo");
        Servo claw = hardwareMap.get(Servo.class, "claw_servo");

        armJoint1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double clawOpen = .78;
        double clawClose = 1;
        double y2;
        double armJoint1CurrentPos = armJoint1.getCurrentPosition();

        armJoint1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armJoint1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double armJoint1Min = armJoint1CurrentPos;
        claw.setPosition(clawClose);


        // Pulls the motors from the robot configuration so that they can be manipulated
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");



        // Reverses the direction of the left motors, to allow a positive motor power to equal
        // forwards and a negative motor power to equal backwards
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);



        // Makes the Driver Hub output the message "Status: Initialized"
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Servo.Direction direction = Servo.Direction.FORWARD;


            // Takes the current y-position of both the left and right joy-sticks
            // The highest position of a joystick is equal to -1, and the bottommost position of the
            // joystick is equal to 1
            double leftTgtPower = -this.gamepad1.left_stick_y;
            double rightTgtPower = -this.gamepad1.right_stick_y;
            y2 = gamepad2.left_stick_y;

            armJoint1.setPower(negSqrt(y2));



            if (gamepad2.dpad_up){
                if (servo_position != SERVO_POS.length - 1){
                    servo_position++;
                }
                else{
                    servo_position=0;
                }
                armJoint2.setPosition(SERVO_POS[servo_position]);
                while (gamepad2.dpad_up){
                    ;
                }
            }
            if (gamepad2.dpad_down){
                servo_position--;
                if (servo_position < 0){
                    servo_position = 0;
                }
                armJoint2.setPosition(SERVO_POS[servo_position]);
                while (gamepad2.dpad_down){
                    ;
                }
            }


            if (gamepad2.y) {
                claw.setPosition(clawOpen);
                armJoint2.setPosition(1);
                servo_position = 0;
            }
            else if(gamepad2.x){
                claw.setPosition(clawClose);
            }


            // Uses the current position of the joy-sticks and makes the motors move at the
            // respective speed
            frontLeft.setPower(leftTgtPower);
            backLeft.setPower(leftTgtPower);
            frontRight.setPower(rightTgtPower);
            backRight.setPower(rightTgtPower);


            // Makes the Driver Hub output the message
            // Left Target Power: A float from [-1,1]
            // Right Target Power: A float from [-1,1]
            // The current power of each motor
            telemetry.addData("Left Target Power", leftTgtPower);
            telemetry.addData("Right Target Power", rightTgtPower);
            telemetry.addData("Arm Position", armJoint1.getCurrentPosition());
            telemetry.addData("Arm Min Position", armJoint1Min);
            telemetry.addData("Front Right Motor Power", frontRight.getPower());
            telemetry.addData("Front Left Motor Power", frontLeft.getPower());
            telemetry.addData("Back Right Motor Power", backRight.getPower());
            telemetry.addData("Back Left Motor Power", backLeft.getPower());




            // Status: Running
            telemetry.addData("Status", "Running");
            telemetry.update();

        }

    }
    public static double negSqrt(double value){
        if(value >= 0){
            return Math.sqrt(value);
        }
        else{
            return -1 * Math.sqrt((-1*value));
        }
    }
}
