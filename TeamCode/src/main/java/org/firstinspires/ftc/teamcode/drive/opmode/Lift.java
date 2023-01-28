package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBot;

class Lift {
    MecanumBot m = new MecanumBot();
    DcMotor slides;
    NormalizedColorSensor color;
    double targetPos;
    double P, I, D;
    double error, lastError;
    boolean up = true;
    public Lift(HardwareMap hardwareMap, double P, double I, double D) {
        slides = hardwareMap.get(DcMotor.class, m.slides);
        color = hardwareMap.get(NormalizedColorSensor.class, m.color);
        this.P = P;
        this.I = I;
        this.D = D;

    }

    public boolean isBusy() {
        if (up){
            return slides.getCurrentPosition() < targetPos;
        }
        else{
            return slides.getCurrentPosition() > targetPos;
        }
    }
    public double getCurrentPos(){
        return slides.getCurrentPosition();
    }

    public void init() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        color.setGain((float) 15);
        slides.setPower(.7);
        NormalizedRGBA colors = color.getNormalizedColors();
        while (colors.green < .42 && colors.red < .42) {
            if(timer.seconds() > 1){
                slides.setPower(0);
                reset();
                break;
            }
            colors = color.getNormalizedColors();

        }
        slides.setPower(0);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        control(-650, 2, .7);
    }

    public void reset() {
        color.setGain((float) 15);
        slides.setPower(-.7);
        NormalizedRGBA colors = color.getNormalizedColors();
        while (colors.green < .37 && colors.red < .37) {

            colors = color.getNormalizedColors();

        }
        slides.setPower(0);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    }

    public void update() {
        double Kp = P;
        double Ki = I;
        double Kd = D;

        double reference = targetPos;

        double integralSum = 0;
        if(isBusy()) {
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

            slides.setPower(out);

            // reset the timer for next time
            timer.reset();
        }
        else{
            slides.setPower(0);
        }
        }

    }

