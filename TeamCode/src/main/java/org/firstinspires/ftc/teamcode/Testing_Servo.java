package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp(name="Servo Testing_Servo", group="Testing_Servo")
public class Testing_Servo extends LinearOpMode {
    DcMotorSimple testingServo;
    Boolean a;//changes servo position by -0.1
    Boolean b;//changes servo position by -0.01
    Boolean y;//changes servo position by  0.01
    Boolean x;//changes servo position by  0.1

    final double flapUp = .379;
    final double flapDown = .77;

    @Override
    public void runOpMode(){
        telemetry.addLine("Gamepad1 Controls:\nx:0.1\ny:0.01\n:a:-0.1\nb:-0.01");
        telemetry.update();
        testingServo = hardwareMap.get(DcMotorSimple.class, "flapper");//change name to servo that is being tested.

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            a = gamepad1.a;
            b = gamepad1.b;
            x = gamepad1.y;
            y = gamepad1.x;
            if(a){
                testingServo.setPower(testingServo.getPower() - .1);
                sleep(1000);
                while(!a){
                    sleep(250);
                }

            }
            if(b){
                testingServo.setPower(testingServo.getPower() - .01);
                sleep(1000);
                while(!b){
                    sleep(250);
                }

            }
            if(x){
                testingServo.setPower(testingServo.getPower() + .1);
                sleep(1000);
                while(!x){
                    sleep(250);
                }

            }
            if(y){
                testingServo.setPower(testingServo.getPower() + .01);
                sleep(1000);
                while(!y){
                    sleep(250);
                }

            }
            if(gamepad2.x){
                testingServo.setPower(closerToV2(flapUp, testingServo.getPower(), flapDown));
                while(gamepad2.x){
                    telemetry.addLine("Waiting for User to Release X");
                    telemetry.update();
                }
                telemetry.clear();
                telemetry.update();

            }
            telemetry.addLine("Gamepad1 Controls:\nx:0.1\ny:0.01\n:a:-0.1\nb:-0.01");
            telemetry.addData("Servo Position", testingServo.getPower());
            telemetry.update();
        }

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
