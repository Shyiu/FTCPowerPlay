package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//cd C:\Users\vihas\Android\platform-tools
//adb.exe
// adb pull sdcard/Logging.txt c:\temp\robot_Logging.txt

//Above command for Log File
@Config
@TeleOp(name = "Controller Testing", group = "Tele-Op")
public class ControllerTesting extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Right Y", gamepad1.right_stick_y + " Game 1 Right Y");

            telemetry.addData("Right X", gamepad1.right_stick_x + " Game 1 Right X");

            telemetry.addData("Left Y", gamepad1.left_stick_y + " Game 1 Left Y");

            telemetry.addData("Left X", gamepad1.left_stick_x + " Game 1 Left X");

            telemetry.addData("2 Right Y", gamepad2.right_stick_y + " Game 2 Right Y");

            telemetry.addData("2 Right X", gamepad2.right_stick_x + " Game 2 Right X");

            telemetry.addData("2 Left Y", gamepad2.left_stick_y + " Game 2 Left Y");

            telemetry.addData("2 Left X", gamepad2.left_stick_x + " Game 2 Left X");

            telemetry.update();
        }
    }
}


