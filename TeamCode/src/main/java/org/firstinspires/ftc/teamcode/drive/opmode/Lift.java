package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class Lift {
    MecanumBotConstant m = new MecanumBotConstant();
    DcMotor slides;
    NormalizedColorSensor color;
    public double targetPos;
    double P, I, D;
    double error, lastError;
    int startPos = Integer.MAX_VALUE;
    boolean up = true;
    double GAIN = 30;
    public boolean stopped = false;
    private double powerReduction = 1;
    private boolean reached = false;
    private double threshold = .60;
    private boolean auto = false;
    HardwareMap hardware;

    public Lift(HardwareMap hardwareMap, double P, double I, double D) {
        slides = hardwareMap.get(DcMotor.class, m.slides);
        targetPos = slides.getCurrentPosition()-100;
        color = hardwareMap.get(NormalizedColorSensor.class, m.color);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.hardware = hardwareMap;
        this.P = P;
        this.I = I;
        this.D = D;

    }
    public Lift(HardwareMap hardwareMap, double P, double I, double D, double powerReduction) {
        slides = hardwareMap.get(DcMotor.class, m.slides);
        targetPos = slides.getCurrentPosition()-100;
        color = hardwareMap.get(NormalizedColorSensor.class, m.color);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.hardware = hardwareMap;
        this.powerReduction = powerReduction;
        this.P = P;
        this.I = I;
        this.D = D;

    }
    public void setMode(boolean b){
        auto = b;
    }
    public boolean barelyBusy() {
        return Math.abs(startPos - slides.getCurrentPosition()) < Math.abs(targetPos-startPos)*0.2;
    }

    public boolean isBusy() {
        if(reached){
            return false;
        }
        if (up) {
            reached = slides.getCurrentPosition() >= targetPos;
            return slides.getCurrentPosition() < targetPos;
        } else {
            reached = slides.getCurrentPosition() <= targetPos;
            return slides.getCurrentPosition() > targetPos;
        }
    }

    public double getCurrentPos() {
        return slides.getCurrentPosition();
    }

    public void init() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        color.setGain((float) GAIN);
        slides.setPower(.3);
        NormalizedRGBA colors = color.getNormalizedColors();
        while (colors.green < threshold && colors.red < threshold) {
            if (timer.seconds() > 1) {
                slides.setPower(0);
                reset();
                break;
            }
            colors = color.getNormalizedColors();

        }
        slides.setPower(0);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // control(-650, 2, .7);
    }

    public void reset() {
        color.setGain((float) GAIN);
        slides.setPower(-.4);
        NormalizedRGBA colors = color.getNormalizedColors();
        while (colors.green < threshold && colors.red < threshold) {

            colors = color.getNormalizedColors();

        }
        slides.setPower(0);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double power) {
        slides.setPower(power);
    }

    public int getCurrentPosition() {
        return slides.getCurrentPosition();
    }

    public void control(double target, double timeoutS, double SLIDE_POWER) {
        double currentTime = System.currentTimeMillis();
        double slidesPosition = slides.getCurrentPosition();
        if (slidesPosition > target) {
            slides.setPower(-SLIDE_POWER);
            while (slidesPosition > target && System.currentTimeMillis() - currentTime < timeoutS * 1000) {
                slidesPosition = slides.getCurrentPosition();
            }
            slides.setPower(0);
        } else if (slidesPosition < target) {
            slides.setPower(SLIDE_POWER);
            while (slidesPosition < target && System.currentTimeMillis() - currentTime < timeoutS * 1000) {
                slidesPosition = slides.getCurrentPosition();
            }
            slides.setPower(0);
        }
    }

    public void move(double target) {
        targetPos = target;
        up = slides.getCurrentPosition() < target;
        startPos = slides.getCurrentPosition();
        reached = false;
    }
    public boolean getReached(){
        return reached;
    }
    public void update() {
        double Kp = P;
        double Ki = I;
        double Kd = D;

        double reference = targetPos;

        double integralSum = 0;
        if (isBusy() && !reached) {
// Elapsed timer class from SDK, please use it, it's epic
            ElapsedTime timer = new ElapsedTime();

            // obtain the encoder position
            double encoderPosition = slides.getCurrentPosition();
            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            slides.setPower(out/powerReduction);

            // reset the timer for next time
            timer.reset();
//            if(getBatteryVoltage() < 9.5){
//                move(getCurrentPosition() - 100);
//                stopped = true;
//            }
//            else{
//                stopped = false;
//            }
        } else if(auto){
            slides.setPower(0);
        }
        else if(!reached){
            slides.setPower(0);
            reached = true;
        }
    }


    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardware.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}