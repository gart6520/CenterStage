package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Climber {
    DcMotorEx climber = null;
    LinearOpMode opMode;
    public Climber(LinearOpMode _opMode)
    {
        opMode = _opMode;
    }
    public void init()
    {
        climber = opMode.hardwareMap.get(DcMotorEx.class, CLIMBER_MOTOR);
    }
    public void setPower(double left)
    {
        climber.setPower(left);
    }
}
