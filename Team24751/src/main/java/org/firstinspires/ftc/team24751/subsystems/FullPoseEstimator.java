package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.BOT_PARAMETERS.ROBOT_TO_CAMERA;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.function.DoubleFunction;
import java.util.function.Supplier;

public class FullPoseEstimator {
    private FuseSensor poseXFuse = new FuseSensor(0);
    private FuseSensor poseYFuse = new FuseSensor(0);
    private DoubleFunction<Vector2d> aprilTagResult;
    private Supplier<Pose2d> odometryResult;

    public double cameraAngle;
    public FullPoseEstimator(DoubleFunction<Vector2d> aprilTagResult, Supplier<Pose2d> odometryResult) {
        this.aprilTagResult = aprilTagResult;
        this.odometryResult = odometryResult;
    }    public FullPoseEstimator(DoubleFunction<Vector2d> aprilTagResult, Supplier<Pose2d> odometryResult, double initCameraAngle) {
        this.aprilTagResult = aprilTagResult;
        this.odometryResult = odometryResult;
        cameraAngle = initCameraAngle;
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
}
