package org.firstinspires.ftc.team24751.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class Lift {
    DcMotorEx leftLift = null;
    DcMotorEx rightLift = null;
    LinearOpMode opMode;
    public Lift(LinearOpMode _opMode)
    {
        opMode = _opMode;
    }
    public void init()
    {
        leftLift = opMode.hardwareMap.get(DcMotorEx.class, "leftLiftMotor");
        rightLift = opMode.hardwareMap.get(DcMotorEx.class, "rightLiftMotor");
    }
    public void setPower(double left, double right)
    {
        leftLift.setPower(left);
        rightLift.setPower(right);
    }
}
