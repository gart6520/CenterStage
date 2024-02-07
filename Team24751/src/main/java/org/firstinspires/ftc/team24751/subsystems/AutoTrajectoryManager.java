package org.firstinspires.ftc.team24751.subsystems;

import static java.lang.Thread.sleep;

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
        public TrajectorySequence purplePixelDrop = null;
        public TrajectorySequence yellowPixelDrop = null;
        public Supplier<TrajectorySequence> repeat = null;

        public AutoTrajectory(TrajectorySequence purple, TrajectorySequence yellow, Supplier<TrajectorySequence> repeatTrajectory) {
            purplePixelDrop = purple;
            yellowPixelDrop = yellow;
            repeat = repeatTrajectory;
        }

        public AutoTrajectory() {
        }
    }


    private AutoTrajectory getAutoTrajectory() {
        AutoTrajectory result = new AutoTrajectory();

        // TODO: Purple pixel drop trajectory

        // Yellow pixel drop trajectory
        switch (pos)
        {
            case wingRed:
                result.yellowPixelDrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(-22.00, -9.50), Math.toRadians(0.00))
                        .lineToConstantHeading(new Vector2d(30.00, -9.50), Math.toRadians(0.00))
                        .lineToConstantHeading(new Vector2d(50.65, -36.00), Math.toRadians(0.00))
                        .turn(180)
                        .build();
                break;
            case wingBlue:
                result.yellowPixelDrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(-22.00, 9.50), Math.toRadians(0.00))
                        .lineToConstantHeading(new Vector2d(30.00, 9.50), Math.toRadians(0.00))
                        .lineToConstantHeading(new Vector2d(50.65, 36.00), Math.toRadians(0.00))
                        .turn(180)
                        .build();
                break;
            case backdropRed:
                result.yellowPixelDrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(47.00, -36.00), Math.toRadians(0.00))
                        .turn(180)
                        .build();
                break;
            case backdropBlue:
                result.yellowPixelDrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(47.00, 36.00), Math.toRadians(0.00))
                        .turn(180)
                        .build();
                break;
        }

        // Repeat trajectory
        if (pos == StartingPos.center) {
            return null; //Return sth u want to test or sth idk
        } else if (pos == StartingPos.wingRed || pos == StartingPos.backdropRed) {
            result.repeat = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(30.00, -9.50), Math.toRadians(180.00))
                    .lineToConstantHeading(new Vector2d(-60.00, -9.50), Math.toRadians(180.00))
                    .lineToConstantHeading(new Vector2d(30.00, -9.50), Math.toRadians(180.00))
                    .lineToConstantHeading(new Vector2d(50.65, -36.00), Math.toRadians(180.00))
                    .build();
        } else {
            result.repeat = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(30.00, 9.50), Math.toRadians(180.00))
                    .lineToConstantHeading(new Vector2d(-60.00, 9.50), Math.toRadians(180.00))
                    .lineToConstantHeading(new Vector2d(30.00, 9.50), Math.toRadians(180.00))
                    .lineToConstantHeading(new Vector2d(50.65, 36.00), Math.toRadians(180.00))
                    .build();
        }
        return result;
    }
    public void followTrajectory() {
        AutoTrajectory autoTrajectory = getAutoTrajectory();
        if (autoTrajectory == null) return;
        drive.setPoseEstimate(autoTrajectory.purplePixelDrop.start());
        opMode.waitForStart();
        timer.reset();
        drive.followTrajectorySequence(autoTrajectory.purplePixelDrop);
        // TODO: Go to desired position after dropping purple pixel
        drive.followTrajectorySequence(autoTrajectory.yellowPixelDrop);
        // TODO: Align robot according to desired AprilTag
        while (opMode.opModeIsActive())
        {
            TrajectorySequence repeat = autoTrajectory.repeat.get();
            if (30 - timer.seconds() <= repeat.duration()) break;
            drive.followTrajectorySequence(repeat);
        }
    }
}
