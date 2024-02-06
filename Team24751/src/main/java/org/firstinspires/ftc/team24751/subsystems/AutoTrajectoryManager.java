package org.firstinspires.ftc.team24751.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.drivebase.trajectorysequence.TrajectorySequence;

import java.util.function.Supplier;

public class AutoTrajectoryManager {
    /**
     * Center is (0,0).
     * Wing is starting position that closer to the wing.
     * Backdrop is starting position that closer to the backdrop.
     * Red/blue is the alliance color (unrelated to the color of the
     * wing or backdrop on the field).
     */
    public enum StartingPos {
        center, wingBlue, wingRed, backdropBlue, backdropRed;
    }

    StartingPos pos;
    Drivebase drive;
    LinearOpMode opMode;
    ElapsedTime timer = new ElapsedTime();

    public AutoTrajectoryManager(StartingPos startingPos, Drivebase drivebase, LinearOpMode _opMode) {
        pos = startingPos;
        drive = drivebase;
        opMode = _opMode;
    }

    public static class AutoTrajectory {
        /**
         * Trajectory for randomization task
         * */
        public TrajectorySequence randomTraj = null;
        public Supplier<TrajectorySequence> mainTraj = null;

        public AutoTrajectory() {
        }
    }


    private AutoTrajectory getAutoTrajectory() {
        AutoTrajectory result = new AutoTrajectory();
        if (pos == StartingPos.center) {
            return null; //Return sth u want to test or sth idk
        } else if (pos == StartingPos.wingRed || pos == StartingPos.backdropRed) {
            result.mainTraj = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(46.50, -39.00, Math.toRadians(180)))
                    .lineTo(new Vector2d(-49.00, -39.00))
                    .lineTo(new Vector2d(46.50, -39.00))
                    .build();
        } else {
            result.mainTraj = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(46.50, 39.00, Math.toRadians(180)))
                    .lineTo(new Vector2d(-49.00, 39.00))
                    .lineTo(new Vector2d(46.50, 39.00))
                    .build();
        }
        switch (pos)
        {
            case wingRed:
                result.randomTraj = drive.trajectorySequenceBuilder(new Pose2d(-36.43, -63.21, Math.toRadians(90.00)))
                        .splineTo(new Vector2d(-49.00, -39), Math.toRadians(180.00))
                        .lineTo(new Vector2d(46.50, -39.00))
                        .setReversed(true)
                        .build();
                break;
            case wingBlue:
                result.randomTraj = drive.trajectorySequenceBuilder(new Pose2d(-36.43, 63.21, Math.toRadians(-90.00)))
                        .splineTo(new Vector2d(-49.00, 39), Math.toRadians(180.00))
                        .lineTo(new Vector2d(46.50, 39.00))
                        .setReversed(true)
                        .build();
                break;
            case backdropRed:
                result.randomTraj = drive.trajectorySequenceBuilder(new Pose2d(36.43, -63.21, Math.toRadians(90.00)))
                        .splineTo(new Vector2d(25.00, -39.00), Math.toRadians(-180))
                        .lineTo(new Vector2d(46.50, -39.00))
                        .build();
                break;
            case backdropBlue:
                result.randomTraj = drive.trajectorySequenceBuilder(new Pose2d(36.43, 63.21, Math.toRadians(-90.00)))
                        .splineTo(new Vector2d(25.00, 39.00), Math.toRadians(-180))
                        .lineTo(new Vector2d(46.50, 39.00))
                        .build();
                break;
        }
        return result;
    }
    public void followTrajectory ()
    {
        AutoTrajectory autoTrajectory = getAutoTrajectory();
        if (autoTrajectory == null) return;
        drive.setPoseEstimate(autoTrajectory.randomTraj.start());
        opMode.waitForStart();
        timer.reset();
        drive.followTrajectorySequence(autoTrajectory.randomTraj);
        while (opMode.opModeIsActive())
        {
            TrajectorySequence repeat = autoTrajectory.mainTraj.get();
            if (30 - timer.seconds() <= repeat.duration()) break;
            drive.followTrajectorySequence(repeat);
        }
    }
}
