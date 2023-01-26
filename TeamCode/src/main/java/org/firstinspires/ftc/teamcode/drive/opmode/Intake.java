package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumBot;

class Intake {
    MecanumBot m = new MecanumBot();
    DcMotorSimple flapper;

    public Intake(HardwareMap hardwareMap) {
        flapper = hardwareMap.get(DcMotorSimple.class, m.intake);
    }

    public void move(double position) {
        flapper.setPower(position);
    }
}
