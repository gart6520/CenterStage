package org.firstinspires.ftc.team24751.subsystems.arm;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Extender {
    DcMotorEx extenderMotor;
    LinearOpMode opMode;

    public Extender(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        extenderMotor = opMode.hardwareMap.get(DcMotorEx.class, EXTENDER_MOTOR);
        extenderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        extenderMotor.setPower(power);
    }
}
