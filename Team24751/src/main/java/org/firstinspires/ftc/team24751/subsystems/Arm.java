package org.firstinspires.ftc.team24751.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm {
    DcMotorEx armMotor;
    LinearOpMode opMode;
    public Arm(LinearOpMode _opMode)
    {
        opMode = _opMode;
    }
    private double getAngle()
    {
        return armMotor.getCurrentPosition() / 1000.0;
    }
    public void init()
    {
        armMotor = opMode.hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder()
    {
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
