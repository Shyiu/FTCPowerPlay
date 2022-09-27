package org.firstinspires.ftc.teamcode;

import org.openftc.easyopencv.OpenCvPipeline;

public class PowerplayPipeline extends OpenCvPipeline

    {
        // Notice this is declared as an instance variable (and re-used), not a local variable
        Mat grey = new Mat();

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
            return grey;
        }
    }


