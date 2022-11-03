package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FiniteStateMachineOpMode extends LinearOpMode {

    public enum CommandState {
        MOVE_ARM,
        MOVE_JOINT,
        MOVE_CLAW,
        MOVE_ROBOT
    };

    CommandState state = CommandState.MOVE_ARM;


    // Initializes all four motors used in the drive train
    // This code will be using a Tank-drive system
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;

    final int[] LEVELS = {86, 1008, 1540};
    final double[] SERVO_POS = {.58,.22,0};
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
        double armJoint2Max = 1;
        double armJoint2Min = 0;
        double armJoint2Increment = .025;
        double y2;
        double incrementWait = .5;
        double armJoint2CurrentPos = armJoint2.getPosition();
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
            switch (state) {
                case MOVE_ROBOT:
                    if (this.gamepad1.left_stick_y != 0 || this.gamepad1.right_stick_y != 0) {
                        double leftTgtPower = -this.gamepad1.left_stick_y;
                        double rightTgtPower = -this.gamepad1.right_stick_y;

                        frontLeft.setPower(leftTgtPower);
                        backLeft.setPower(leftTgtPower);
                        frontRight.setPower(rightTgtPower);
                        backRight.setPower(rightTgtPower);
                        telemetry.addData("Left Target Power", leftTgtPower);
                        telemetry.addData("Right Target Power", rightTgtPower);
                        state = CommandState.MOVE_ARM;

                    }
                    break;
                case MOVE_ARM:
                    if (gamepad2.left_stick_y != 0) {
                        y2 = gamepad2.left_stick_y;
                        armJoint1.setPower(negSqrt(y2));
                        state = CommandState.MOVE_JOINT;
                    }
                    break;
                case MOVE_JOINT:
                    if (gamepad2.dpad_down || gamepad2.dpad_up) {
                        armJoint1CurrentPos = armJoint1.getCurrentPosition();

                        if (-armJoint1CurrentPos < armJoint1Min * 1.25){
                            armJoint2Min = 0;
                        }
                        else {
                            armJoint2Min = .57;
                        }
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
                        state = CommandState.MOVE_CLAW;
                    }
                    break;
                case MOVE_CLAW:
                    if (gamepad2.x || gamepad2.y) {
                        if (gamepad2.y) {
                            claw.setPosition(clawOpen);
                            armJoint2.setPosition(1);
                            servo_position = 0;
                        }
                        else if(gamepad2.x){
                            claw.setPosition(clawClose);
                        }
                        state = CommandState.MOVE_ROBOT;
                    }
                    break;

            }



//            if (gamepad2.dpad_up){
//                if (!(current_level == LEVELS.length - 1)){
//                    current_level++;
//                    armJoint1.setTargetPosition(LEVELS[current_level]);
//                    armJoint1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    armJoint1.setPower(-.5);
//                    while(armJoint1.isBusy()){
//                        ;
//                    }
//                    armJoint1.setPower(0);
//                    armJoint1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                }
//            }
//            if (gamepad2.dpad_down){
//                if (!(current_level == 0)){
//                    current_level--;
//                    armJoint1.setTargetPosition(LEVELS[current_level]);
//                    armJoint1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    armJoint1.setPower(-.5);
//                    while(armJoint1.isBusy()){
//                            ;
//                    }
//                    armJoint1.setPower(0);
//                    armJoint1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//               }
//            }




//            if((gamepad2.left_trigger) > 0){
//                armJoint2CurrentPos = armJoint2.getPosition();
//                if (armJoint2CurrentPos + armJoint2Increment <= armJoint2Max){
//                    armJoint2CurrentPos += armJoint2Increment;
//                }
//                else{
//                    armJoint2CurrentPos = armJoint2Max;
//                }
//
//                incrementWait = 1.001 - gamepad2.left_trigger;
//                armJoint2.setPosition(armJoint2CurrentPos);
//                sleep((long)(incrementWait*10));
//            }
//            else if((gamepad2.right_trigger) > 0){
//                armJoint2CurrentPos = armJoint2.getPosition();
//                if (armJoint2CurrentPos - armJoint2Increment >= armJoint2Min){
//                    armJoint2CurrentPos -= armJoint2Increment;
//                }
//                else{
//                    armJoint2CurrentPos = armJoint2Min;
//                }
//
//                incrementWait = 1.001 - gamepad2.right_trigger;
//                armJoint2.setPosition(armJoint2CurrentPos);
//                sleep((long)(incrementWait*1500));
//            }





//            if (gamepad2.dpad_up || current_level == 10){
//                if(current_level < LEVELS.length -1)  {
//                    current_level++;
//                }
//                slides.setTargetPosition(LEVELS[current_level]);
//                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//            else if (gamepad2.dpad_down || current_level == 10){
//                if(current_level > 0) {
//                    current_level--;
//                }
//                slides.setTargetPosition(LEVELS[current_level]);
//                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//            else if (Math.abs(gamepad2.left_stick_y) > 0){
//                slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                slides.setPower(-1*(negSqrt(gamepad2.left_stick_y)/2.0));
//            }
//            else{
//                slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                slides.setPower(0);
//            }




            // Uses the current position of the joy-sticks and makes the motors move at the
            // respective speed



            // Makes the Driver Hub output the message
            // Left Target Power: A float from [-1,1]
            // Right Target Power: A float from [-1,1]
            // The current power of each motor

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