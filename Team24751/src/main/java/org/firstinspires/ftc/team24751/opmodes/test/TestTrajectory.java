package org.firstinspires.ftc.team24751.opmodes.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

        TrajectorySequence redFarAfterPurple = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -49.00, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(-60.00, -9.50), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(30.00, -9.50), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(50.65, -36.00), Math.toRadians(180.00))
                .build();

        TrajectorySequence repeatRed = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(30.00, -9.50), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(-60.00, -9.50), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(30.00, -9.50), Math.toRadians(180.00))
                .lineToConstantHeading(new Vector2d(50.65, -36.00), Math.toRadians(180.00))
                .build();

        drive.setPoseEstimate(redFarAfterPurple.start());
        waitForStart();
        timer.reset();
        drive.followTrajectorySequence(redFarAfterPurple);
        while (30 - timer.seconds() > repeatRed.duration())
        {
            drive.followTrajectorySequence(repeatRed);
        }
        sleep((long)(30 - timer.seconds() * 1000));
    }
}
