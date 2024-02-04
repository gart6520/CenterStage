package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.BOT_PARAMETERS.ROBOT_TO_CAMERA;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.function.DoubleFunction;
import java.util.function.Supplier;

public class FullPoseEstimator {
    private final FuseSensor poseXFuse;
    private final FuseSensor poseYFuse;
    private final DoubleFunction<Vector2d> aprilTagResult;
    private final Supplier<Pose2d> odometryResult;
    public double cameraAngle;

    public FullPoseEstimator(DoubleFunction<Vector2d> aprilTagResult, Supplier<Pose2d> odometryResult, Pose2d initPose) {
        this.aprilTagResult = aprilTagResult;
        this.odometryResult = odometryResult;
        poseXFuse = new FuseSensor(initPose.getX(), 1);
        poseYFuse = new FuseSensor(initPose.getY(), 1);
    }

    public Pose2d update() {
        final Pose2d odoPose = odometryResult.get();
        Vector2d aprilTagPos = aprilTagResult.apply(Math.toDegrees(odoPose.getHeading()) + cameraAngle);
        if (aprilTagPos != null) aprilTagPos = aprilTagPos.minus(ROBOT_TO_CAMERA);
        Double xAprilTag = aprilTagPos == null ? null : aprilTagPos.getX();
        Double yAprilTag = aprilTagPos == null ? null : aprilTagPos.getY();
        return new Pose2d(
                poseXFuse.update(odoPose.getX(), xAprilTag),
                poseYFuse.update(odoPose.getY(), yAprilTag),
                odoPose.getHeading());
    }

    public void setCameraAngle(double cameraAngle) {
        this.cameraAngle = cameraAngle;
    }
}
