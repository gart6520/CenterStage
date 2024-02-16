package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.BACKDROP_BLUE_START_POSE;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.BACKDROP_RED_START_POSE;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.CENTER_BACKDROP;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.CENTER_SPIKE_MARK;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.LEFT_BACKDROP;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.LEFT_SPIKE_MARK;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.RIGHT_BACKDROP;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.RIGHT_SPIKE_MARK;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.WING_BLUE_START_POSE;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.WING_RED_START_POSE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.ARM_BACKDROP_PARALLEL_ANGLE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.ARM_BACKDROP_PARALLEL_ANGLE_AUTO;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.CLOSE_CLAW_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.OPEN_CLAW_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.WRIST_GROUND_PARALLEL_DEG;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.Constants;
import org.firstinspires.ftc.team24751.Utility;
import org.firstinspires.ftc.team24751.commands.AutoArmFSM;
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
    AutoArmFSM autoArmFSM;

    public AutoTrajectoryManager(StartingPos startingPos, Constants.VISION.CV.TeamPropPosition teamPropPos,
                                 Drivebase drivebase, AutoArmFSM autoArm, LinearOpMode _opMode) {
        pos = startingPos;
        teamPropPosition = teamPropPos;
        drive = drivebase;
        opMode = _opMode;
        autoArmFSM = autoArm;
    }

    public static class AutoTrajectory {
        public TrajectorySequence purplePixelDrop = null;
        // Use supplier to update init pose
        public Supplier<TrajectorySequence> yellowPixelDrop = null;
        public Supplier<TrajectorySequence> repeatToStack = null;
        public Supplier<TrajectorySequence> repeatToBackdrop = null;

        public AutoTrajectory() {
        }

        public AutoTrajectory(TrajectorySequence traj) {
            purplePixelDrop = traj;
        }
    }


    private AutoTrajectory getAutoTrajectory() {
        // TODO Refactor all these to new framework
        if (pos == StartingPos.center) {
            return new AutoTrajectory(
                    drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                            .addDisplacementMarker(() ->
                            {
                                autoArmFSM.wrist.setAngle(WRIST_GROUND_PARALLEL_DEG);
                                autoArmFSM.grabber.setPosition(OPEN_CLAW_POSITION, OPEN_CLAW_POSITION);
                            })
                            .lineTo(new Vector2d(5, 0))
                            .addDisplacementMarker(() ->
                            {
                                autoArmFSM.grabber.setPosition(CLOSE_CLAW_POSITION, CLOSE_CLAW_POSITION);
                            })
                            .lineTo(new Vector2d(0, 0))
                            .build()
            );
        }
        AutoTrajectory result = new AutoTrajectory();
        Pose2d initPose = null;
        Vector2d toSpikeMark = null;
        double toSpikeMarkRot = 0;
        switch (teamPropPosition) {
            case LEFT:
                toSpikeMark = new Vector2d(LEFT_SPIKE_MARK.getX(), LEFT_SPIKE_MARK.getY());
                toSpikeMarkRot = LEFT_SPIKE_MARK.getHeading();
                break;
            case RIGHT:
                toSpikeMark = new Vector2d(RIGHT_SPIKE_MARK.getX(), RIGHT_SPIKE_MARK.getY());
                toSpikeMarkRot = RIGHT_SPIKE_MARK.getHeading();
                break;
            case CENTER:
                toSpikeMark = new Vector2d(CENTER_SPIKE_MARK.getX(), CENTER_SPIKE_MARK.getY());
                toSpikeMarkRot = CENTER_SPIKE_MARK.getHeading();
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
        Pose2d toSpikeMarkWorld = new Pose2d(initPose.getX() + toSpikeMark.getX(), initPose.getY() + toSpikeMark.getY(),
                initPose.getHeading() + toSpikeMarkRot);
        result.purplePixelDrop = drive.trajectorySequenceBuilder(initPose)
                .lineToLinearHeading(toSpikeMarkWorld)
                .build();
        // Yellow pixel drop trajectory
        switch (pos) {
            // TODO: Make all start of yellow traj go to a point
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
            // TODO split
            result.repeatToStack = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(30.00, -9.50))
                    .lineToConstantHeading(new Vector2d(-60.00, -9.50))
                    .lineToConstantHeading(new Vector2d(30.00, -9.50))
                    .lineToConstantHeading(new Vector2d(50.65, -36.00))
                    .build();
        } else {
            result.repeatToStack = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
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
        if (pos == StartingPos.center) {
            drive.followTrajectorySequence(autoTrajectory.purplePixelDrop);
            return;
        }
        drive.followTrajectorySequence(autoTrajectory.purplePixelDrop);

        autoArmFSM.timeoutTimer.reset();
        autoArmFSM.state = AutoArmFSM.ArmState.purple_pixel;
        while (autoArmFSM.state != AutoArmFSM.ArmState.roadrunner) {
            autoArmFSM.update();
        }
        // Go to desired position after dropping purple pixel
        drive.followTrajectorySequence(autoTrajectory.yellowPixelDrop.get());

        autoArmFSM.waitServoTimer.reset();
        autoArmFSM.state = AutoArmFSM.ArmState.yellow_pixel;
        while (autoArmFSM.state != AutoArmFSM.ArmState.roadrunner) {
            autoArmFSM.update();
        }
        Vector2d toCorrectBackdropPos = new Vector2d();
        switch (teamPropPosition) {
            case LEFT:
                toCorrectBackdropPos = LEFT_BACKDROP;
                break;
            case RIGHT:
                toCorrectBackdropPos = RIGHT_BACKDROP;
                break;
            case CENTER:
                toCorrectBackdropPos = CENTER_BACKDROP;
                break;
        }
        Pose2d pose0 = drive.getPoseEstimate();
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(pose0)
                .lineTo(new Vector2d(pose0.getX(), pose0.getY()).plus(toCorrectBackdropPos))
                .build()
        );
        while (opMode.opModeIsActive()) {
            TrajectorySequence repeatToStack = autoTrajectory.repeatToStack.get();
            drive.followTrajectorySequence(repeatToStack);
            autoArmFSM.state = AutoArmFSM.ArmState.prepare_intaking;
            Pose2d pose1 = drive.getPoseEstimate();
            drive.followTrajectorySequenceAsync(
                    drive.trajectorySequenceBuilder(pose1)
                            .lineTo(new Vector2d(pose1.getX() - 5, pose1.getY()))
                            .build());
            while (drive.isBusy()) {
                autoArmFSM.update();
                drive.update();
            }

            autoArmFSM.waitServoTimer.reset();
            autoArmFSM.state = AutoArmFSM.ArmState.intaking;
            while (autoArmFSM.state != AutoArmFSM.ArmState.roadrunner) {
                autoArmFSM.update();
            }
            Pose2d pose2 = drive.getPoseEstimate();
            drive.followTrajectorySequenceAsync(
                    drive.trajectorySequenceBuilder(pose2)
                            .lineTo(new Vector2d(pose2.getX() + 5, pose2.getY()))
                            .build());
            autoArmFSM.state = AutoArmFSM.ArmState.after_intake;
            autoArmFSM.update();
            TrajectorySequence repeatToBackdrop = autoTrajectory.repeatToBackdrop.get();
            drive.followTrajectorySequence(repeatToBackdrop);

            autoArmFSM.timeoutTimer.reset();
            autoArmFSM.arm.setTargetAngle(ARM_BACKDROP_PARALLEL_ANGLE_AUTO);
            autoArmFSM.state = AutoArmFSM.ArmState.arm_moving_up;
            while (autoArmFSM.state != AutoArmFSM.ArmState.roadrunner) {
                autoArmFSM.update();
            }
            if (30 - timer.seconds() <= repeatToStack.duration() + repeatToBackdrop.duration() + 5)
                break;
        }
    }
}
