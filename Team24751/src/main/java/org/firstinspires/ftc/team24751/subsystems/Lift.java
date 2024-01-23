package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Lift {
    DcMotorEx leftLift = null;
    LinearOpMode opMode;
    public Lift(LinearOpMode _opMode)
    {
        opMode = _opMode;
    }
    public void init()
    {
        leftLift = opMode.hardwareMap.get(DcMotorEx.class, LIFT_MOTOR);
    }
    public void setPower(double left)
    {
        leftLift.setPower(left);
    }
}
