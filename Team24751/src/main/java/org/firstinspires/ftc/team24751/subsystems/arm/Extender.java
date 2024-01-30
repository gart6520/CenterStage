package org.firstinspires.ftc.team24751.subsystems.arm;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Extender {
    DcMotorEx extenderMotor;
    LinearOpMode opMode;

    public Extender(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        extenderMotor = opMode.hardwareMap.get(DcMotorEx.class, EXTENDER_MOTOR);
        extenderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extenderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.resetPosition();
    }

    /**
     * Set power for extend motor
     * Negative -> out
     * Positive -> in
     * @param power
     */
    public void setPower(double power) {
        extenderMotor.setPower(-power);
    }

    public int getPosition() {
        return extenderMotor.getCurrentPosition();
    }

    public void resetPosition() {
        extenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
