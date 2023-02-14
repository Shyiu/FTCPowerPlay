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
public class JunctionPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        TARGET,
        UP,
        BELOW,
        NOT_FOUND
    }
    private Location location;

    static final Rect TARGET = new Rect(
            new Point(0, 150),
            new Point(800, 350));
    static final Rect TOP_TARGET = new Rect(
            new Point(0, 0),
            new Point(800, 150));
    static final Rect BOTTOM_TARGET = new Rect(
            new Point(0, 350),
            new Point(800, 448));

    static double PERCENT_COLOR_THRESHOLD = 0.4;

    public JunctionPipeline(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat targetMat = mat.submat(TARGET);
        Mat topMat = mat.submat(TOP_TARGET);
        Mat bottomMat = mat.submat(BOTTOM_TARGET);


        double targetValue = Core.sumElems(targetMat).val[0] / TARGET.area() / 255;
        double topValue = Core.sumElems(topMat).val[0] / TOP_TARGET.area() / 255;
        double bottomValue = Core.sumElems(bottomMat).val[0] / BOTTOM_TARGET.area() / 255;

        bottomMat.release();
        targetMat.release();
        topMat.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(targetMat).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(topMat).val[0]);
        telemetry.addData("Left percentage", Math.round(targetValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(topValue * 100) + "%");

        boolean onTarget = targetValue > PERCENT_COLOR_THRESHOLD;
        boolean aboveTarget = topValue > PERCENT_COLOR_THRESHOLD;
        boolean bottomTarget = bottomValue > PERCENT_COLOR_THRESHOLD;


        if (onTarget && aboveTarget && bottomTarget) {
            location = Location.NOT_FOUND;
            telemetry.addData("Junction Location", "Literally Trash");
        }
        else if (onTarget) {
            location = Location.TARGET;
            telemetry.addData("Junction Location", "Target");
        }
        else if (bottomTarget){
            location = Location.BELOW;
            telemetry.addData("Junction Location", "Below");
        }
        else {
            location = Location.UP;
            telemetry.addData("Junction Location", "UP");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, TARGET, location == Location.TARGET ? colorSkystone:colorStone);
        Imgproc.rectangle(mat, TOP_TARGET, location == Location.UP ? colorSkystone:colorStone);

        return mat;
    }

    public Location getLocation() {
        return location;
    }

}