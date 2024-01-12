package org.firstinspires.ftc.team24751.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team24751.roadrunner.ThreeDeadWheelLocalizer;

public class OdometryPod {
    ThreeDeadWheelLocalizer localizer;
    LinearOpMode opMode;
    Pose2d currentPose = new Pose2d(0,0,0);
    public OdometryPod(LinearOpMode _opMode, double inPerTick) {
        opMode = _opMode;
        localizer = new ThreeDeadWheelLocalizer(opMode.hardwareMap, inPerTick);
    }
    public Twist2dDual<Time> update()
    {
        Twist2dDual<Time> twist = localizer.update();
        currentPose = currentPose.plus(twist.value());
        return twist;
    }
    public Pose2d getCurrentPose()
    {
        return currentPose;
    }
}
