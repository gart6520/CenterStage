package org.firstinspires.ftc.team24751.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
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
    public void update()
    {
        currentPose = currentPose.plus(localizer.update().value());
    }
    public Pose2d getCurrentPose()
    {
        return currentPose;
    }

}
