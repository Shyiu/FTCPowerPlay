package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//for dashboard
@Config
public class BlueConeStackPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public static int lowBlueBlueValue =102;
    public static int lowBlueGreenValue =120;
    public static int lowBlueRedValue =103;

    public static int highBlueBlueValue = 255;
    public static int highBlueGreenValue = 198;
    public static int highBlueRedValue = 168;
    public enum Location {
        TARGET,
        UP,
        BELOW,
        NOT_FOUND
    }
    private Location location;
    public static int x1 = 530;
    public static int x2 = 320;
    static Rect TARGET = new Rect(
            new Point(x2, 0),
            new Point(x1, 150));
    static  Rect BOTTOM_TARGET = new Rect(
            new Point(x1, 150),
            new Point(x2, 448));
    int[] values = {320,375,420,475,530};
    int[] values2 = {120,175,220,275,320};
    static double PERCENT_COLOR_THRESHOLD = 0.33;

    public BlueConeStackPipeline(Telemetry t, int conesLeft) { telemetry = t;
        TARGET = new Rect(
            new Point(values2[conesLeft-1], 0),
            new Point(values[conesLeft-1], 150));
        BOTTOM_TARGET = new Rect(
                new Point(values2[conesLeft-1], 150),
                new Point(values[conesLeft-1], 448));
    }


    @Override
    public Mat processFrame(Mat input) {
        TARGET = new Rect(
                new Point(x2, 0),
                new Point(x1, 150));
        BOTTOM_TARGET = new Rect(
                new Point(x1, 150),
                new Point(x2, 448));
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowBlueHSV = new Scalar(lowBlueRedValue, lowBlueGreenValue, lowBlueBlueValue);
        Scalar highBlueHSV = new Scalar(highBlueRedValue, highBlueGreenValue, highBlueBlueValue);
        Core.inRange(mat, lowBlueHSV, highBlueHSV, mat);

        Mat targetMat = mat.submat(TARGET);
        Mat bottomMat = mat.submat(BOTTOM_TARGET);
//
//
        double targetValue = Core.sumElems(targetMat).val[0] / TARGET.area() / 255;
        double bottomValue = Core.sumElems(bottomMat).val[0] / BOTTOM_TARGET.area() / 255;

        bottomMat.release();
        targetMat.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(targetMat).val[0]);
        telemetry.addData("Left percentage", Math.round(targetValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(bottomValue * 100) + "%");

        boolean onTarget = targetValue > PERCENT_COLOR_THRESHOLD;
        boolean bottomTarget = bottomValue > .26;

        if (bottomTarget){
            location = Location.BELOW;
            telemetry.addData("Junction Location", "Below");
        }
        else if (onTarget) {
            location = Location.TARGET;
            telemetry.addData("Junction Location", "Target");
        }

        else {
            if(targetValue + bottomValue > .27){
                location = Location.BELOW;
                telemetry.addData("Junction Location", "BELOW");

            }
            else {
                location = Location.UP;
                telemetry.addData("Junction Location", "UP");

            }
        }
        telemetry.update();
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);
        Imgproc.rectangle(mat, TARGET, location == Location.TARGET ? colorSkystone:colorStone);
        Imgproc.rectangle(mat, BOTTOM_TARGET, location == Location.UP ? colorSkystone:colorStone);

        return mat;
    }

    public Location getLocation() {
        return location;
    }

}