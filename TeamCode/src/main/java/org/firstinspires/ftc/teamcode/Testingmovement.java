package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.obselete.PowerplayBot;

@TeleOp(name="Test that All motors are equal", group="Testing_Servo")
public class Testingmovement extends LinearOpMode {
    DcMotor a, b, c, d;

    //1540 1008 86
    private ElapsedTime runtime = new ElapsedTime();
    public final PowerplayBot names = new PowerplayBot();
    final int[] LEVELS = {86, 985};
    int current_level = 0;

    @Override
    public void runOpMode() {
        a = hardwareMap.get(DcMotor.class, names.fr);
        b = hardwareMap.get(DcMotor.class, names.fl);
        c = hardwareMap.get(DcMotor.class, names.bl);
        d = hardwareMap.get(DcMotor.class, names.br);

        a.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        c.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        a.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        c.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        a.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        c.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        b.setDirection(DcMotorSimple.Direction.REVERSE);
        c.setDirection(DcMotorSimple.Direction.REVERSE);


        a.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        c.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            a.setPower(1);
            sleep(1000);
            a.setPower(0);
            telemetry.addLine(names.fr + " position is " + a.getCurrentPosition());

            b.setPower(1);
            sleep(1000);
            b.setPower(0);
            telemetry.addLine(names.fl + " position is " + b.getCurrentPosition());


            c.setPower(1);
            sleep(1000);
            c.setPower(0);
            telemetry.addLine(names.bl + " position is " + c.getCurrentPosition());

            d.setPower(1);
            sleep(1000);
            d.setPower(0);
            telemetry.addLine(names.br + " position is " + d.getCurrentPosition());

            telemetry.update();
            sleep(500000);
//            testingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            testingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}