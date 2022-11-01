package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
@Disabled
public class PowerplayPipeline extends OpenCvPipeline

    {
        static final double PERCENT_COLOR_THRESHOLD = .2;
        Telemetry telemetry;

        // Notice this is declared as an instance variable (and re-used), not a local variable
        Mat mat = new Mat();
        public enum Location {
            LEFT,
            RIGHT,
            NOT_FOUND

        }
        private Location location;
        static final Rect LEFT_ROI  = new Rect(
                new Point(60, 35),
                new Point(120, 75));
        static final Rect RIGHT_ROI = new Rect(
                new Point(140, 35),
                new Point(200, 75));
        public PowerplayPipeline(Telemetry t){ telemetry = t;}
        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            Scalar lowYellowHsv = new Scalar(23, 50, 70);
            Scalar highYellowHsv = new Scalar(32, 255, 255);

            Core.inRange(mat, lowYellowHsv, highYellowHsv, mat);

            Mat right = mat.submat(RIGHT_ROI);
            Mat left = mat.submat(LEFT_ROI);
            double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area()/255;
            double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area()/255;

            left.release();
            right.release();

            telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Right raw value",(int) Core.sumElems(right).val[0]);
            telemetry.addData("Left percentage",Math.round(leftValue * 100) + "%");
            telemetry.addData("Right percentage",Math.round(rightValue * 100) + "%");

            boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
            boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

            if (stoneLeft && stoneRight) {
                location = Location.NOT_FOUND;
                telemetry.addData("Skystone Location", "not found");
            }
            else if (stoneLeft) {
                location = Location.RIGHT;
                telemetry.addData("Skystone Location", "left");
            }
            else {
                location = Location.LEFT;
                telemetry.addData("Skystone Location", "right");
            }
            telemetry.update();

            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

            Scalar colorStone = new Scalar(255, 0, 0);
            Scalar colorSkystone = new Scalar(0, 255, 0);

            Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
            Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);

            return mat;
        }

        public Location getLocation() {
            return location;
        }
    }


