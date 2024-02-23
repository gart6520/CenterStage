package org.firstinspires.ftc.team24751.commands;

import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.BACKDROP_BLUE_START_POSE;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.BACKDROP_RED_START_POSE;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.CENTER_BACKDROP;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.LEFT_BACKDROP;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.RIGHT_BACKDROP;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.WING_BLUE_START_POSE;
import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.WING_RED_START_POSE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.ARM_BACKDROP_PARALLEL_ANGLE_AUTO;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.CLOSE_CLAW_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.OPEN_CLAW_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.WRIST_GROUND_PARALLEL_DEG;
import static org.firstinspires.ftc.team24751.Constants.startingPos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.Constants;
import org.firstinspires.ftc.team24751.Utility;
import org.firstinspires.ftc.team24751.subsystems.drivebase.DriveConstants;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.drivebase.trajectorysequence.TrajectorySequence;

import java.util.function.Supplier;

public class AutoTrajectoryManager {
    Constants.VISION.CV.TeamPropPosition teamPropPosition;
    Drivebase drive;
    LinearOpMode opMode;
    ElapsedTime timer = new ElapsedTime();
    AutoFSM autoFSM;

    public AutoTrajectoryManager(StartingPos startingPos, Constants.VISION.CV.TeamPropPosition teamPropPos, Drivebase drivebase, AutoFSM autoArm, LinearOpMode _opMode) {
        Constants.startingPos = startingPos;
        teamPropPosition = teamPropPos;
        drive = drivebase;
        opMode = _opMode;
        autoFSM = autoArm;
    }

    private AutoTrajectory getAutoTrajectory() {
        // TODO Refactor all these to new framework
        if (startingPos == StartingPos.center) {
            return new AutoTrajectory(drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0)).addDisplacementMarker(() -> {
                autoFSM.wrist.setAngle(WRIST_GROUND_PARALLEL_DEG);
                autoFSM.grabber.setPosition(OPEN_CLAW_POSITION, OPEN_CLAW_POSITION);
            }).lineTo(new Vector2d(5, 0)).addDisplacementMarker(() -> {
                autoFSM.grabber.setPosition(CLOSE_CLAW_POSITION, CLOSE_CLAW_POSITION);
            }).lineTo(new Vector2d(0, 0)).build());
        }
        AutoTrajectory result = new AutoTrajectory();
        Pose2d initPose = null;

        // Drop purple Pixel
        switch (startingPos) {
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
        Pose2d initPoseToPurplePose = new Pose2d();
        // Truss to the right
        if (startingPos == StartingPos.wingRed || startingPos == StartingPos.backdropBlue) {
            switch (teamPropPosition) {
                case LEFT:
                    initPoseToPurplePose = new Pose2d(18.7, 3, Math.toRadians(20));
                    break;
                case RIGHT:
                    initPoseToPurplePose = new Pose2d(21.00, -5, Math.toRadians(-40));
                    break;
                case CENTER:
                    initPoseToPurplePose = new Pose2d(22.00, 0, Math.toRadians(0));
                    break;
                case NONE:
            }
        } else { // Truss to the left
            switch (teamPropPosition) {
                case LEFT:
                    initPoseToPurplePose = new Pose2d(20.00, 0, Math.toRadians(40));
                    break;
                case RIGHT:
                    initPoseToPurplePose = new Pose2d(18.7, -7, Math.toRadians(-25));
                    break;
                case CENTER:
                    initPoseToPurplePose = new Pose2d(22.00, 0, Math.toRadians(0));
                    break;
                case NONE:
            }
        }


        double firstSidewayMove = Double.MIN_VALUE;
        switch (startingPos) {
            case wingRed:
            case backdropBlue:
                firstSidewayMove = 4;
                break;
            case wingBlue:
            case backdropRed:
                firstSidewayMove = -4;
                break;
        }

        Vector2d initPosToPurplePos = Utility.rotateVector(new Vector2d(initPoseToPurplePose.getX(), initPoseToPurplePose.getY()), initPose.getHeading());
        Vector2d getAwayTruss = Utility.rotateVector(new Vector2d(5, firstSidewayMove), initPose.getHeading());
        result.purplePixelDrop = drive.trajectorySequenceBuilder(initPose).lineTo(new Vector2d(initPose.getX(), initPose.getY()).plus(getAwayTruss)).lineToLinearHeading(new Pose2d(initPose.getX() + initPosToPurplePos.getX(), initPose.getY() + initPosToPurplePos.getY(), initPose.getHeading() + initPoseToPurplePose.getHeading())).build();

        Vector2d goBack = Utility.rotateVector(new Vector2d(15, 0), initPose.getHeading());
        Vector2d toCorrectBackdropPos = new Vector2d();
        // Yellow Pixel Offset for each team prop case
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

        // Yellow pixel drop trajectory
        Vector2d finalToCorrectBackdropPos = toCorrectBackdropPos;
        Pose2d finalInitPose = initPose;
        switch (startingPos) {
            // TODO: Make all start of yellow traj go to a point (may not be necessary)
            case wingRed:
                /*
                result.afterPurplePixel = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-36.00, -48.00, Math.toRadians(90.00)))
                        .build();
                 */

                /*
                result.yellowPixelDrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-57.50, -48.00))
                        .lineToConstantHeading(new Vector2d(-57.50, -9.50))
                        .lineToLinearHeading(new Pose2d(-28.00, -9.50, Math.toRadians(180.00)))
                        .lineToConstantHeading(new Vector2d(30.00, -9.50))
                        .lineToConstantHeading(new Vector2d(48.00, -44.00).plus(finalToCorrectBackdropPos))
                        .build();
                break;
                 */

                result.yellowPixelDrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(finalInitPose.plus(new Pose2d(goBack.getX(), goBack.getY(), 0)))
                        .lineToLinearHeading(new Pose2d(-36.00, -62.00, Math.toRadians(180.00)))
                        .lineToLinearHeading(new Pose2d(25.00, -62.00, Math.toRadians(180.00)))
                        .lineToLinearHeading(new Pose2d(48.00 + finalToCorrectBackdropPos.getX(), -44.00 + finalToCorrectBackdropPos.getY(), Math.toRadians(180.00)))
                        .build();
                break;
            case wingBlue:
                /*
                result.afterPurplePixel = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-36.00, 48.00, Math.toRadians(-90.00)))
                        .build();
                 */
                /*
                result.yellowPixelDrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(-57.50, 48.00))
                        .lineToConstantHeading(new Vector2d(-57.50, 8.00))
                        .lineToLinearHeading(new Pose2d(-28.00, 8.00, Math.toRadians(180.00)))
                        .lineToConstantHeading(new Vector2d(30.00, 8.00))
                        .lineToConstantHeading(new Vector2d(48.00, 44.00).plus(finalToCorrectBackdropPos))
                        .build();
                break;
                 */

                result.yellowPixelDrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(finalInitPose.plus(new Pose2d(goBack.getX(), goBack.getY(), 0)))
                        .lineToLinearHeading(new Pose2d(-36.00, 60.69, Math.toRadians(180.00)))
                        .lineToLinearHeading(new Pose2d(25.00, 60.69, Math.toRadians(180.00)))
                        .lineToLinearHeading(new Pose2d(48.00 + finalToCorrectBackdropPos.getX(), 26.00 + finalToCorrectBackdropPos.getY(), Math.toRadians(180.00)))
                        .build();
                break;
            case backdropRed:
                /*
                result.afterPurplePixel = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(12.00, -48.00, Math.toRadians(90.00)))
                        .build();
                 */
                result.yellowPixelDrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(finalInitPose.plus(new Pose2d(goBack.getX(), goBack.getY(), 0)))
                        .lineToLinearHeading(new Pose2d(48.00 + finalToCorrectBackdropPos.getX(), -44.00 + finalToCorrectBackdropPos.getY(), Math.toRadians(180.00)))
                        .build();
                break;
            case backdropBlue:
                /*
                result.afterPurplePixel = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(12.00, 48.00, Math.toRadians(-90.00)))
                        .build();
                 */
                result.yellowPixelDrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(finalInitPose.plus(new Pose2d(goBack.getX(), goBack.getY(), 0)))
                        .lineToLinearHeading(new Pose2d(48.00 + finalToCorrectBackdropPos.getX(), 29.00 + finalToCorrectBackdropPos.getY(), Math.toRadians(180.00)))
                        .build();
                break;
        }

        // Repeat trajectory
        if (startingPos == StartingPos.wingRed || startingPos == StartingPos.backdropRed) {
            result.repeatToStack = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(48.00, -11.00, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(-35.00, -11.00, Math.toRadians(180)))
                    .addDisplacementMarker(() -> {
                        autoFSM.state = AutoFSM.ArmState.prepare_intaking;
                        autoFSM.waitServoTimer.reset();
                        while (autoFSM.state != AutoFSM.ArmState.roadrunner) {
                            autoFSM.update();
                        }
                    })
                    .setVelConstraint(new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(-58.00, -11.00, Math.toRadians(180)))
                    .resetConstraints()
                    .build();
            result.repeatToBackdrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setConstraints(new MecanumVelocityConstraint(56, DriveConstants.TRACK_WIDTH), new ProfileAccelerationConstraint(150))
                    .lineToConstantHeading(new Vector2d(-35.00, -11.00))
                    .resetConstraints()
                    .addDisplacementMarker(() -> {
                        autoFSM.state = AutoFSM.ArmState.after_intake;
                        autoFSM.update();
                    })
                    .lineToLinearHeading(new Pose2d(-28.00, -11.00, Math.toRadians(0.00)))
                    .lineToConstantHeading(new Vector2d(39.00, -11.00))
                    .lineToLinearHeading(new Pose2d(48.00, -36.00, Math.toRadians(0.00)))
                    .build();
        } else {
            result.repeatToStack = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(48.00, 11.00, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(-35.00, 11.00, Math.toRadians(180)))
                    .addDisplacementMarker(() -> {
                        autoFSM.state = AutoFSM.ArmState.prepare_intaking;
                        autoFSM.waitServoTimer.reset();
                        while (autoFSM.state != AutoFSM.ArmState.roadrunner) {
                            autoFSM.update();
                        }
                    })
                    .setVelConstraint(new MecanumVelocityConstraint(35, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(-58.00, 11.00, Math.toRadians(180)))
                    .resetConstraints()
                    .build();
            result.repeatToBackdrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setConstraints(new MecanumVelocityConstraint(56, DriveConstants.TRACK_WIDTH), new ProfileAccelerationConstraint(150))
                    .lineToConstantHeading(new Vector2d(-35.00, 11.00))
                    .resetConstraints()
                    .addDisplacementMarker(() -> {
                        autoFSM.state = AutoFSM.ArmState.after_intake;
                        autoFSM.update();
                    })
                    .lineToLinearHeading(new Pose2d(-28.00, 11.00, Math.toRadians(0.00)))
                    .lineToConstantHeading(new Vector2d(39.00, 11.00))
                    .lineToLinearHeading(new Pose2d(48.00, 36.00, Math.toRadians(0.00)))
                    .build();
        }
        return result;
    }

    public void followTrajectory() {
        AutoTrajectory autoTrajectory = getAutoTrajectory();
        if (autoTrajectory == null) return; // Just in case
        drive.setPoseEstimate(autoTrajectory.purplePixelDrop.start());

        timer.reset();
        // For testing only
        if (startingPos == StartingPos.center) {
            drive.followTrajectorySequence(autoTrajectory.purplePixelDrop);
            return;
        }

        // Go to spike mark appropriately
        autoFSM.grabber.setPosition(CLOSE_CLAW_POSITION, CLOSE_CLAW_POSITION);
        drive.followTrajectorySequence(autoTrajectory.purplePixelDrop);

        // Drop purple Pixel
        autoFSM.timeoutTimer.reset();
        autoFSM.hasBorrowThread = false;
        autoFSM.borrowThread = autoTrajectory.yellowPixelDrop;
        autoFSM.state = AutoFSM.ArmState.purple_pixel;
        while (autoFSM.state != AutoFSM.ArmState.roadrunner) {
            autoFSM.update();
        }

        // Go to allocated position (may not be necessary but still keep here just in case)
        // drive.followTrajectorySequence(autoTrajectory.afterPurplePixel);

        // Go to backdrop for yellow pixel drop
        drive.followTrajectorySequence(autoFSM.result);

        // Drop yellow Pixel
        autoFSM.waitServoTimer.reset();
        autoFSM.borrowThread = autoTrajectory.repeatToStack;
        autoFSM.hasBorrowThread = false;
        autoFSM.state = AutoFSM.ArmState.yellow_pixel;
        while (autoFSM.state != AutoFSM.ArmState.roadrunner) {
            autoFSM.update();
        }
        while (opMode.opModeIsActive()) {
            // Go to stack
            TrajectorySequence repeatToStack = autoFSM.result;
            drive.followTrajectorySequence(repeatToStack);

            // Intaking
            autoFSM.waitServoTimer.reset();
            autoFSM.state = AutoFSM.ArmState.intaking;
            autoFSM.hasBorrowThread = false;
            autoFSM.borrowThread = autoTrajectory.repeatToBackdrop;
            while (autoFSM.state != AutoFSM.ArmState.roadrunner) {
                autoFSM.update();
            }

            // Go to backdrop
            TrajectorySequence repeatToBackdrop = autoFSM.result;
            drive.followTrajectorySequence(repeatToBackdrop);

            // Outtake Pixels
            autoFSM.timeoutTimer.reset();
            autoFSM.arm.setTargetAngle(ARM_BACKDROP_PARALLEL_ANGLE_AUTO);
            autoFSM.state = AutoFSM.ArmState.arm_moving_up;
            while (autoFSM.state != AutoFSM.ArmState.roadrunner) {
                autoFSM.update();
            }
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(4)
                    .build());

            autoFSM.waitServoTimer.reset();
            autoFSM.state = AutoFSM.ArmState.outaking;
            while (autoFSM.state != AutoFSM.ArmState.roadrunner) {
                autoFSM.update();
            }
            // TODO: Go back to backstage
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setVelConstraint(new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH))
                    .back(9).resetConstraints().build());
            autoFSM.timeoutTimer.reset();
            autoFSM.state = AutoFSM.ArmState.arm_moving_down;
            while (autoFSM.state != AutoFSM.ArmState.roadrunner) {
                autoFSM.update();
            }
            if (30 - timer.seconds() <= repeatToStack.duration() + repeatToBackdrop.duration() + 5)
                break;
        }
    }


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

    public static class AutoTrajectory {
        public TrajectorySequence purplePixelDrop = null;
        public TrajectorySequence afterPurplePixel = null;
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
}
