package org.firstinspires.ftc.team24751.opmodes.test;

import static org.openftc.apriltag.ApriltagDetectionJNI.getPoseEstimate;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.drivebase.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "TestTrajectory", group = "Test")
public class TestTrajectory extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drivebase drive = new Drivebase(this);

        /*TrajectorySequence Test = drive.trajectorySequenceBuilder(new Pose2d(-36.26, -65.62, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-58.65, -38.48), Math.toRadians(180.00))
                .lineTo(new Vector2d(43.38, -38.78))
                .setReversed(true)
                .build();

         */
        /*TrajectorySequence Test = drive.trajectorySequenceBuilder(new Pose2d(64.88, -62.66, Math.toRadians(180.00)))
                .splineTo(new Vector2d(-62.81, -62.66), Math.toRadians(180.00))
                .build();
         */
        drive.setPoseEstimate(new Pose2d(-36.2, -62.21, Math.toRadians(90.00)));

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-36.2, -38.93))
                .build();

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(traj1);
        telemetry.update();
        drive.turn(Math.toRadians(90));
        telemetry.update();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-57.00, -38.93))
                .lineTo(new Vector2d(49.00, -38.93))
                .setReversed(true)
                .build();

        drive.followTrajectorySequence(traj2);
        telemetry.update();

        while (!isStopRequested()) {
            TrajectorySequence traj3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(-57.00, -38.93))
                    .lineTo(new Vector2d(49.00, -38.93))
                    .setReversed(true)
                    .build();
            drive.followTrajectorySequence(traj3);
            telemetry.update();
        }

        waitForStart();
    }
}
