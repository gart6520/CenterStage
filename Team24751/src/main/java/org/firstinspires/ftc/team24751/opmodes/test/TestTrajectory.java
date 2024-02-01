package org.firstinspires.ftc.team24751.opmodes.test;

import static org.openftc.apriltag.ApriltagDetectionJNI.getPoseEstimate;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.drivebase.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "TestTrajectory", group = "Test")
public class TestTrajectory extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        Drivebase drive = new Drivebase(this);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-36.43, -63.21, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-49.00, -39), Math.toRadians(180.00))
                .lineTo(new Vector2d(46.50, -39.00))
                .setReversed(true)
                .build();
        TrajectorySequence repeatTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-49.00, -39.00))
                .lineTo(new Vector2d(46.50, -39.00))
                .build();

        drive.setPoseEstimate(traj.start());
        waitForStart();
        timer.reset();
        drive.followTrajectorySequence(traj);
        while (30 - timer.seconds() > repeatTraj.duration())
        {
            drive.followTrajectorySequence(repeatTraj);
        }
    }
}
