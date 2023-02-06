package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "StickObserverTest")

public class JunctionRunner extends LinearOpMode {

    @Override
    public void runOpMode() {
//        initialize camera and pipeline
        JunctionRecognition cv = new JunctionRecognition(this);
//      call the function to startStreaming
        cv.observeStick();
        waitForStart();
        while (opModeIsActive()) {
            sleep(500);
        }
//        stopStreaming
        cv.stopCamera();
    }
}

