package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class Intake {
    MecanumBotConstant m = new MecanumBotConstant();
    DcMotorSimple right;
    DcMotorSimple left;
    enum state{
            OPEN,
            CLOSE
    };
    state position = state.CLOSE;
    public Intake(HardwareMap hardwareMap) {
        right = hardwareMap.get(DcMotorSimple.class, m.right_servo);
        left = hardwareMap.get(DcMotorSimple.class, m.left_servo);
    }

    public void move(state pos) {
        switch(pos){
            case OPEN:
                right.setPower(m.rightServoOpenPosition);
                left.setPower(m.leftServoOpenPosition);
                break;
            case CLOSE:
                right.setPower(m.rightServoClosedPosition);
                left.setPower(m.leftServoClosedPosition);
                break;
        }



    }
}
