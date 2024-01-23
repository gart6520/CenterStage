package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.BOT_PARAMETERS.robotToCamera;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.function.DoubleFunction;
import java.util.function.Supplier;

public class FullPoseEstimator {
    private FuseSensor poseXFuse = new FuseSensor(0);
    private FuseSensor poseYFuse = new FuseSensor(0);
    private DoubleFunction<Vector2d> aprilTagResult;
    private Supplier<Pose2d> odometryResult;

    public FullPoseEstimator(DoubleFunction<Vector2d> aprilTagResult, Supplier<Pose2d> odometryResult) {
        this.aprilTagResult = aprilTagResult;
        this.odometryResult = odometryResult;
    }

    public Pose2d update() {
        Pose2d odoPose = odometryResult.get();
        Vector2d aprilTagPos = aprilTagResult.apply(odoPose.getHeading());
        if (aprilTagPos != null) aprilTagPos = aprilTagPos.minus(robotToCamera);
        Double xAprilTag = aprilTagPos == null ? null : aprilTagPos.getX();
        Double yAprilTag = aprilTagPos == null ? null : aprilTagPos.getY();
        return new Pose2d(
                poseXFuse.update(odoPose.getX(), xAprilTag),
                poseYFuse.update(odoPose.getY(), yAprilTag),
                odoPose.getHeading());
    }
}
