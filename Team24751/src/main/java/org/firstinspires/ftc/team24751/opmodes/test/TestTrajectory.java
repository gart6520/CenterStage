package org.firstinspires.ftc.team24751.opmodes.test;

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

        waitForStart();

        if (isStopRequested()) return;
        TrajectorySequence Test = drive.trajectorySequenceBuilder(new Pose2d(-38.02, -63.93, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-37.81, -35.89), Math.toRadians(0.00))
                .splineTo(new Vector2d(38.44,  -35.89), Math.toRadians(0.00))
                .build();
        drive.setPoseEstimate(Test.start());

        drive.followTrajectorySequence(Test);
        telemetry.update();
    }
}
