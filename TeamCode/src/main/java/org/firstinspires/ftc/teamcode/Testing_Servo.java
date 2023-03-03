package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Servo Testing_Servo", group="Testing_Servo")
public class Testing_Servo extends LinearOpMode {
    DcMotorSimple rightServo;
    DcMotorSimple leftServo;
    Boolean a;//changes servo position by -0.1
    Boolean b;//changes servo position by -0.01
    Boolean y;//changes servo position by  0.01
    Boolean x;//changes servo position by  0.1
    Boolean a2;//changes servo position by -0.1
    Boolean b2;//changes servo position by -0.01
    Boolean y2;//changes servo position by  0.01
    Boolean x2;//changes servo position by  0.1
    public enum FLAPPER_STATE{
        MOVE,
        WAIT
    }
    FLAPPER_STATE flapperToggle = FLAPPER_STATE.WAIT;
    ElapsedTime timer = new ElapsedTime();
    double currentTime = 0;
    boolean closedPrior = false;
    final double flapUp = .379;
    final double flapDown = .77;
    MecanumBotConstant m = new MecanumBotConstant();
    @Override
    public void runOpMode(){

        telemetry.addLine("Gamepad1 and 2 Controls:\nx:0.1\ny:0.01\n:a:-0.1\nb:-0.01");
        telemetry.update();
        rightServo = hardwareMap.get(DcMotorSimple.class, m.right_servo);//change name to servo that is being tested.
        leftServo = hardwareMap.get(DcMotorSimple.class, m.left_servo);
        rightServo.setPower(m.rightServoOpenPosition);
        leftServo.setPower(m.leftServoOpenPosition);
        waitForStart();
        timer.reset();
        while (opModeIsActive() && !isStopRequested()){
            a = gamepad1.a;
            b = gamepad1.b;
            y = gamepad1.y;
            x = gamepad1.x;
            a2 = gamepad2.a;
            b2 = gamepad2.b;
            y2 = gamepad2.y;
            x2 = gamepad2.x;
            if(a){
                rightServo.setPower(rightServo.getPower() - .1);
                sleep(1000);
                while(!a){
                    sleep(250);
                }

            }
            if(b){
                rightServo.setPower(rightServo.getPower() - .01);
                sleep(1000);
                while(!b){
                    sleep(250);
                }

            }
            if(x){
                rightServo.setPower(rightServo.getPower() + .1);
                sleep(1000);
                while(!x){
                    sleep(250);
                }

            }
            if(y){
                rightServo.setPower(rightServo.getPower() + .01);
                sleep(1000);
                while(!y){
                    sleep(250);
                }

            }
            if(a2){
                leftServo.setPower(leftServo.getPower() - .1);
                sleep(1000);
                while(!a2){
                    sleep(250);
                }

            }
            if(b2){
                leftServo.setPower(leftServo.getPower() - .01);
                sleep(1000);
                while(!b2){
                    sleep(250);
                }

            }
            if(x2){
                leftServo.setPower(leftServo.getPower() + .1);
                sleep(1000);
                while(!x2){
                    sleep(250);
                }

            }
            if(y2){
                leftServo.setPower(leftServo.getPower() + .01);
                sleep(1000);
                while(!y2){
                    sleep(250);
                }

            }
            switch(flapperToggle){
                case MOVE:
                    if(timer.time(TimeUnit.MILLISECONDS) - currentTime > 500) {
                        flapperToggle = FLAPPER_STATE.WAIT;
                    }
                    break;
                case WAIT:
                    if(gamepad2.right_bumper){
                        if (closedPrior){
                            flapperToggle = FLAPPER_STATE.MOVE;
                            rightServo.setPower(m.rightServoOpenPosition);
                            leftServo.setPower(m.leftServoOpenPosition);
                            currentTime = timer.time(TimeUnit.MILLISECONDS);
                            closedPrior = false;
                        }
                        else{
                            flapperToggle = FLAPPER_STATE.MOVE;
                            rightServo.setPower(m.rightServoClosedPosition);
                            leftServo.setPower(m.leftServoClosedPosition);
                            currentTime = timer.time(TimeUnit.MILLISECONDS);
                            closedPrior = true;
                        }
                    }
                    break;

            }

            if(gamepad2.right_stick_x != 0){
                double rightTarget = rightServo.getPower() - gamepad2.right_stick_x/150.0;
                double leftTarget = leftServo.getPower() - gamepad2.right_stick_x/150.0;
                if (rightTarget > m.rightHardStopOut && leftTarget < m.leftHardStopOut && rightTarget < m.rightHardStopIn && leftTarget > m.leftHardStopIn) {
                    rightServo.setPower(rightTarget);
                    leftServo.setPower(leftTarget);
                }

            }
            telemetry.addLine("Gamepad1 Controls:\nx:0.1\ny:0.01\n:a:-0.1\nb:-0.01");
            telemetry.addData("Right Servo Position", rightServo.getPower());
            telemetry.addData("Right Servo In Hardstop", m.rightHardStopIn);
            telemetry.addData("Right Servo Out Hardstop", m.rightHardStopOut);
            telemetry.addData("Left Servo Position", leftServo.getPower());
            telemetry.addData("Left Servo In Hardstop", m.leftHardStopIn);
            telemetry.addData("Left Servo Out Hardstop", m.leftHardStopOut);
            telemetry.update();
        }
        rightServo.setPower(m.rightServoClosedPosition);
        leftServo.setPower(m.leftServoClosedPosition);
    }

    public double closerToV2(double v1, double v2, double v3){
        double diff1 = Math.abs(v1-v2);
        double diff2 = Math.abs(v2-v3);
        if (diff1 > diff2){
            return v1;
        }
        return v3;
    }
}
