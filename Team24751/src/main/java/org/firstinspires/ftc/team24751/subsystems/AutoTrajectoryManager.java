package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.BACKDROP_BLUE_START_POSE;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.BACKDROP_RED_START_POSE;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.CENTER_SPIKE_MARK;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.LEFT_SPIKE_MARK;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.RIGHT_SPIKE_MARK;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.WING_BLUE_START_POSE;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.WING_RED_START_POSE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.Constants;
import org.firstinspires.ftc.team24751.Utility;
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
        center, wingBlue, wingRed, backdropBlue, backdropRed
    }

    StartingPos pos;
    Constants.VISION.CV.TeamPropPosition teamPropPosition;
    Drivebase drive;
    LinearOpMode opMode;
    ElapsedTime timer = new ElapsedTime();

    public AutoTrajectoryManager(StartingPos startingPos, Constants.VISION.CV.TeamPropPosition teamPropPos, Drivebase drivebase, LinearOpMode _opMode) {
        pos = startingPos;
        teamPropPosition = teamPropPos;
        drive = drivebase;
        opMode = _opMode;
    }

    public static class AutoTrajectory {
        public TrajectorySequence purplePixelDrop = null;
        // Use supplier to update init pose
        public Supplier<TrajectorySequence> yellowPixelDrop = null;
        public Supplier<TrajectorySequence> repeat = null;

        public AutoTrajectory() {
        }
    }


    private AutoTrajectory getAutoTrajectory() {
        if (pos == StartingPos.center) {
            return null; //Return sth u want to test or sth idk
        }
        AutoTrajectory result = new AutoTrajectory();
        Pose2d initPose = null;
        Vector2d toSpikeMark = null;
        switch (teamPropPosition) {
            case LEFT:
                toSpikeMark = LEFT_SPIKE_MARK;
                break;
            case RIGHT:
                toSpikeMark = RIGHT_SPIKE_MARK;
                break;
            case CENTER:
                toSpikeMark = CENTER_SPIKE_MARK;
                break;
            case NONE:
                toSpikeMark = new Vector2d();
        }
        switch (pos) {
            case wingRed:
                initPose = WING_RED_START_POSE;
                break;
            case wingBlue:
                initPose = WING_BLUE_START_POSE;
                break;
            case backdropRed:
                initPose = BACKDROP_RED_START_POSE;
                break;
            case backdropBlue:
                initPose = BACKDROP_BLUE_START_POSE;
                break;
        }
        toSpikeMark = Utility.rotateVector(toSpikeMark, initPose.getHeading());
        Vector2d toSpikeMarkWorld = new Vector2d(initPose.getX() + toSpikeMark.getX(), initPose.getY() + toSpikeMark.getY());
        result.purplePixelDrop = drive.trajectorySequenceBuilder(initPose)
                .lineToConstantHeading(toSpikeMarkWorld)
                .build();
        // Yellow pixel drop trajectory
        switch (pos) {
            case wingRed:
                result.yellowPixelDrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(-54.00, -9.50), Math.toRadians(180.00))
                        .lineToConstantHeading(new Vector2d(30.00, -9.50))
                        .lineToConstantHeading(new Vector2d(50.65, -36.00))
                        .build();
                break;
            case wingBlue:
                result.yellowPixelDrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(-54.00, 9.50), Math.toRadians(180.00))
                        .lineToConstantHeading(new Vector2d(30.00, 9.50))
                        .lineToConstantHeading(new Vector2d(50.65, 36.00))
                        .build();
                break;
            case backdropRed:
                // Test both cases
                result.yellowPixelDrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(47.00, -36.00), Math.toRadians(0.00))
                        .turn(180)
                        .build();
                /*
                result.yellowPixelDrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(47.00, -36.00), Math.toRadians(180.00))
                        .build();
                */
                break;
            case backdropBlue:
                // Test both cases
                result.yellowPixelDrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(47.00, 36.00), Math.toRadians(0.00))
                        .turn(180)
                        .build();
                /*
                result.yellowPixelDrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(47.00, 36.00), Math.toRadians(180.00))
                        .build();
                */
                break;
        }

        // Repeat trajectory
        if (pos == StartingPos.wingRed || pos == StartingPos.backdropRed) {
            result.repeat = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(30.00, -9.50))
                    .lineToConstantHeading(new Vector2d(-60.00, -9.50))
                    .lineToConstantHeading(new Vector2d(30.00, -9.50))
                    .lineToConstantHeading(new Vector2d(50.65, -36.00))
                    .build();
        } else {
            result.repeat = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(30.00, 9.50))
                    .lineToConstantHeading(new Vector2d(-60.00, 9.50))
                    .lineToConstantHeading(new Vector2d(30.00, 9.50))
                    .lineToConstantHeading(new Vector2d(50.65, 36.00))
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
        drive.followTrajectorySequence(autoTrajectory.yellowPixelDrop.get());
        // TODO: Align robot according to desired AprilTag
        while (opMode.opModeIsActive()) {
            TrajectorySequence repeat = autoTrajectory.repeat.get();
            if (30 - timer.seconds() <= repeat.duration()) break;
            drive.followTrajectorySequence(repeat);
        }
    }
}
