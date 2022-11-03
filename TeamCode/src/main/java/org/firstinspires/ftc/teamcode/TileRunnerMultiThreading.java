package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.threadopmode.TaskThread;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;

@TeleOp
public class TileRunnerMultiThreading extends ThreadOpMode {
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;
    protected DcMotor armJoint1;
    protected Servo armJoint2;
    protected Servo claw;

    final int[] LEVELS = {86, 1008, 1540};
    final double[] SERVO_POS = {.58,.22,0};
    int servo_position = 0;
    int current_level = 0;
    @Override
    public void mainInit() {
        armJoint1 = hardwareMap.get(DcMotor.class, "joint_motor");
        armJoint2 = hardwareMap.get(Servo.class, "joint_servo");
        claw = hardwareMap.get(Servo.class, "claw_servo");

        armJoint1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double clawOpen = .78;
        double clawClose = 1;
        double armJoint2Max = 1;
        double armJoint2Min = 0;
        double armJoint2Increment = .025;
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


        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                //The loop method should contain loop code
                double leftTgtPower = -gamepad1.left_stick_y;
                double rightTgtPower = -gamepad1.right_stick_y;
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
                telemetry.addData("Front Right Motor Power", frontRight.getPower());
                telemetry.addData("Front Left Motor Power", frontLeft.getPower());
                telemetry.addData("Back Right Motor Power", backRight.getPower());
                telemetry.addData("Back Left Motor Power", backLeft.getPower());




                // Status: Running
                telemetry.addData("Status", "Running");
                telemetry.update();

            }
        }));
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                double y2 = gamepad2.left_stick_y;


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
            }
        }));
    }
    public static double negSqrt(double value){
        if(value >= 0){
            return Math.sqrt(value);
        }
        else{
            return -1 * Math.sqrt((-1*value));
        }
    }
    @Override
    public void mainLoop() {

    }

}
