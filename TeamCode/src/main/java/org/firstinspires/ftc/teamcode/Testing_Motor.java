package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name="Motor Testing", group="Testing_Servo")
public class Testing_Motor extends LinearOpMode {
    DcMotor testingMotor;
    Boolean a;//changes servo position by -0.1
    Boolean b;//changes servo position by -0.01
    Boolean y;//changes servo position by  0.01
    Boolean x;//changes servo position by  0.1
    //1540 1008 86
    private ElapsedTime runtime = new ElapsedTime();

    final int[] LEVELS = {86, 985};
    int current_level = 0;

    @Override
    public void runOpMode() {
        testingMotor = hardwareMap.get(DcMotor.class, "slides");//change name to servo that is being tested.
        testingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addLine(testingMotor.getCurrentPosition() + "");
        telemetry.update();
        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addLine(testingMotor.getCurrentPosition() + "");
            testingMotor.setPower(gamepad1.left_stick_y/3.0);
            telemetry.addLine("Gamepad1 Controls:\nLeft Stick Y Moves Motor");
            telemetry.addData("Slide Position", testingMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    public void encoderDrive(int ticks) {
        int newLeftTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = ticks;


            testingMotor.setTargetPosition(newLeftTarget);


            // Turn On RUN_TO_POSITION
            testingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            testingMotor.setPower(-.5);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < 5) &&
                    (testingMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d", newLeftTarget);
                telemetry.addData("Currently at", " at %7d",
                        testingMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            testingMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            testingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
