package org.firstinspires.ftc.team24751.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team24751.roadrunner.ThreeDeadWheelLocalizer;

public class OdometryPod {
    ThreeDeadWheelLocalizer localizer;
    LinearOpMode opMode;
    Gyro gyro = null;
    Pose2d currentPose = new Pose2d(0, 0, 0);

    public OdometryPod() {
    }

    public void init(LinearOpMode _opMode, double inPerTick, Gyro gyro) {
        opMode = _opMode;
        localizer = new ThreeDeadWheelLocalizer(opMode.hardwareMap, inPerTick);
        this.gyro = gyro;
    }

    public Twist2dDual<Time> update() {
        final Twist2dDual<Time> twist = localizer.update();
        currentPose = currentPose.plus(twist.value());
        gyro.currentAngle = currentPose.heading.toDouble();
        return twist;
    }

    public Pose2d getCurrentPose() {
        return currentPose;
    }
}
