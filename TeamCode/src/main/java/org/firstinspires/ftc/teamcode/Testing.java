package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Testing", group="Testing")
public class Testing extends LinearOpMode {
    Servo testingServo;
    Boolean a;
    Boolean b;
    Boolean y;
    Boolean x;


    @Override
    public void runOpMode(){
        telemetry.addLine("Gamepad1 Controls:\nx:0.1\ny:0.01\n:a:-0.1\nb:-0.01");
        telemetry.update();
        testingServo = hardwareMap.get(Servo.class, "servo_one"); //change name to servo that is being tested.
        a = gamepad1.a;
        b = gamepad1.b;
        x = gamepad1.y;
        y = gamepad1.x;
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            if(a){
                testingServo.setPosition(testingServo.getPosition() - .1);
                while(!a){
                    sleep(500);
                }

            }
            if(b){
                testingServo.setPosition(testingServo.getPosition() - .01);
                while(!b){
                    sleep(500);
                }

            }
            if(x){
                testingServo.setPosition(testingServo.getPosition() + .1);
                while(!x){
                    sleep(500);
                }

            }
            if(y){
                testingServo.setPosition(testingServo.getPosition() + .01);
                while(!y){
                    sleep(500);
                }

            }
            telemetry.addData("Servo Position", testingServo.getPosition());
            telemetry.update();
        }
    }
}
