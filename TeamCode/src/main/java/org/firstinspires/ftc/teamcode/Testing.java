package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Testing", group="Testing")
public class Testing extends LinearOpMode {
    Servo testingServo;
    Boolean a;//changes servo position by -0.1
    Boolean b;//changes servo position by -0.01
    Boolean y;//changes servo position by  0.01
    Boolean x;//changes servo position by  0.1


    @Override
    public void runOpMode(){
        telemetry.addLine("Gamepad1 Controls:\nx:0.1\ny:0.01\n:a:-0.1\nb:-0.01");
        telemetry.update();
        testingServo = hardwareMap.get(Servo.class, "claw_servo");//change name to servo that is being tested.

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            a = gamepad1.a;
            b = gamepad1.b;
            x = gamepad1.y;
            y = gamepad1.x;
            if(a){
                testingServo.setPosition(testingServo.getPosition() - .1);
                sleep(1000);
                while(!a){
                    sleep(500);
                }

            }
            if(b){
                testingServo.setPosition(testingServo.getPosition() - .01);
                sleep(1000);
                while(!b){
                    sleep(500);
                }

            }
            if(x){
                testingServo.setPosition(testingServo.getPosition() + .1);
                sleep(1000);
                while(!x){
                    sleep(500);
                }

            }
            if(y){
                testingServo.setPosition(testingServo.getPosition() + .01);
                sleep(1000);
                while(!y){
                    sleep(500);
                }

            }
            telemetry.addLine("Gamepad1 Controls:\nx:0.1\ny:0.01\n:a:-0.1\nb:-0.01");
            telemetry.addData("Servo Position", testingServo.getPosition());
            telemetry.update();
        }
    }
}
