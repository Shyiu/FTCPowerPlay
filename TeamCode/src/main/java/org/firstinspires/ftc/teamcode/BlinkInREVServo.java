package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="REV STUFF WHY", group="Testing_Servo")
public class BlinkInREVServo extends LinearOpMode {
    RevBlinkinLedDriver rightServo;
    DcMotorSimple leftServo;
    Boolean a;//changes servo position by -0.1
    Boolean b;//changes servo position by -0.01
    Boolean y;//changes servo position by  0.01
    Boolean x;//changes servo position by  0.1
    public enum FLAPPER_STATE{
        MOVE,
        WAIT
    }
    FLAPPER_STATE flapperToggle = FLAPPER_STATE.WAIT;
    ElapsedTime timer = new ElapsedTime();
    double currentTime = 0;
    boolean closedPrior = false;
    final double flapDown = .77;
    MecanumBotConstant m = new MecanumBotConstant();
    @Override
    public void runOpMode(){

        telemetry.addLine("Gamepad1 and 2 Controls:\nx:0.1\ny:0.01\n:a:-0.1\nb:-0.01");
        telemetry.update();
        rightServo = hardwareMap.get(RevBlinkinLedDriver.class, m.led);//change name to servo that is being tested.
        rightServo.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        waitForStart();
        timer.reset();
        while (opModeIsActive() && !isStopRequested()){

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
