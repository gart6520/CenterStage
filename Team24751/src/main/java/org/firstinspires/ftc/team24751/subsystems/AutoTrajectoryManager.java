package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.AUTONOMOUS.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.ARM_BACKDROP_PARALLEL_ANGLE_AUTO;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.*;
import static org.firstinspires.ftc.team24751.Constants.startingPos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.Constants;
import org.firstinspires.ftc.team24751.Utility;
import org.firstinspires.ftc.team24751.commands.AutoFSM;
import org.firstinspires.ftc.team24751.subsystems.drivebase.DriveConstants;
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

    Constants.VISION.CV.TeamPropPosition teamPropPosition;
    Drivebase drive;
    LinearOpMode opMode;
    ElapsedTime timer = new ElapsedTime();
    AutoFSM autoFSM;

    public AutoTrajectoryManager(StartingPos startingPos, Constants.VISION.CV.TeamPropPosition teamPropPos,
                                 Drivebase drivebase, AutoFSM autoArm, LinearOpMode _opMode) {
        Constants.startingPos = startingPos;
        teamPropPosition = teamPropPos;
        drive = drivebase;
        opMode = _opMode;
        autoFSM = autoArm;
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


    private AutoTrajectory getAutoTrajectory() {
        // TODO Refactor all these to new framework
        if (startingPos == StartingPos.center) {
            return new AutoTrajectory(
                    drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                            .addDisplacementMarker(() ->
                            {
                                autoFSM.wrist.setAngle(WRIST_GROUND_PARALLEL_DEG);
                                autoFSM.grabber.setPosition(OPEN_CLAW_POSITION, OPEN_CLAW_POSITION);
                            })
                            .lineTo(new Vector2d(5, 0))
                            .addDisplacementMarker(() ->
                            {
                                autoFSM.grabber.setPosition(CLOSE_CLAW_POSITION, CLOSE_CLAW_POSITION);
                            })
                            .lineTo(new Vector2d(0, 0))
                            .build()
            );
        }
        AutoTrajectory result = new AutoTrajectory();
        Pose2d initPose = null;
        boolean isCloseLeft = startingPos == StartingPos.wingRed || startingPos == StartingPos.backdropBlue;

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
        switch (teamPropPosition) {
            case CLOSE:
                initPoseToPurplePose = new Pose2d(3.90625, 25.46875, Math.toRadians(30));
                break;
            case FAR:
                initPoseToPurplePose = new Pose2d(3.90625, 25.46875, Math.toRadians(-30));
                break;
            case CENTER:
                initPoseToPurplePose = new Pose2d(3.90625, 25.46875, Math.toRadians(0));
                break;
            case NONE:
        }
        if (startingPos == StartingPos.backdropRed) {
            initPoseToPurplePose = new Pose2d(-initPoseToPurplePose.getX(), initPoseToPurplePose.getY(), initPoseToPurplePose.getHeading());
        } else if (startingPos == StartingPos.wingBlue) {
            initPoseToPurplePose = new Pose2d(initPoseToPurplePose.getX(), -initPoseToPurplePose.getY(), initPoseToPurplePose.getHeading());
        } else if (startingPos == StartingPos.backdropBlue) {
            initPoseToPurplePose = new Pose2d(-initPoseToPurplePose.getX(), -initPoseToPurplePose.getY(), initPoseToPurplePose.getHeading());
        }
        Vector2d goBack = Utility.rotateVector(new Vector2d(0, 10), initPose.getHeading());
        result.purplePixelDrop = drive.trajectorySequenceBuilder(initPose)
                .forward(2)
                .lineToLinearHeading(initPose.plus(initPoseToPurplePose))
                .lineToLinearHeading(initPose.plus(new Pose2d(goBack.getX(), goBack.getY(), 0)))
                .build();

        Vector2d toCorrectBackdropPos = new Vector2d();
        // Yellow Pixel Offset for each team prop case
        switch (teamPropPosition) {
            case CLOSE:
                toCorrectBackdropPos = isCloseLeft ? LEFT_BACKDROP : RIGHT_BACKDROP;
                break;
            case FAR:
                toCorrectBackdropPos = isCloseLeft ? RIGHT_BACKDROP : LEFT_BACKDROP;
                break;
            case CENTER:
                toCorrectBackdropPos = CENTER_BACKDROP;
                break;
        }
        // Yellow pixel drop trajectory
        Vector2d finalToCorrectBackdropPos = toCorrectBackdropPos;
        switch (startingPos) {
            // TODO: Make all start of yellow traj go to a point (may not be necessary)
            case wingRed:
                /*
                result.afterPurplePixel = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-36.00, -48.00, Math.toRadians(90.00)))
                        .build();
                 */
                result.yellowPixelDrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-36.00, -62.00, Math.toRadians(180.00)))
                        .lineToConstantHeading(new Vector2d(25.00, -62.00))
                        .lineToConstantHeading(new Vector2d(50.65, -44.00).plus(finalToCorrectBackdropPos))
                        .build();
                break;
            case wingBlue:
                /*
                result.afterPurplePixel = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-36.00, 48.00, Math.toRadians(-90.00)))
                        .build();
                 */
                result.yellowPixelDrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-36.00, 62.00, Math.toRadians(180.00)))
                        .lineToConstantHeading(new Vector2d(25.00, 62.00))
                        .lineToConstantHeading(new Vector2d(50.65, 28.00).plus(finalToCorrectBackdropPos))
                        .build();
                break;
            case backdropRed:
                /*
                result.afterPurplePixel = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(12.00, -48.00, Math.toRadians(90.00)))
                        .build();
                 */
                result.yellowPixelDrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(50.65 + finalToCorrectBackdropPos.getX(), -44.00 + finalToCorrectBackdropPos.getY(), Math.toRadians(180.00)))
                        .build();
                break;
            case backdropBlue:
                /*
                result.afterPurplePixel = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(12.00, 48.00, Math.toRadians(-90.00)))
                        .build();
                 */
                result.yellowPixelDrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(50.65 + finalToCorrectBackdropPos.getX(), 28.00 + finalToCorrectBackdropPos.getY(), Math.toRadians(180.00)))
                        .build();
                break;
        }

        // Repeat trajectory
        if (startingPos == StartingPos.wingRed || startingPos == StartingPos.backdropRed) {
            result.repeatToStack = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(30.00, -9.50))
                    .lineToConstantHeading(new Vector2d(-55.00, -9.50))
                    .addDisplacementMarker(() ->
                    {
                        autoFSM.state = AutoFSM.ArmState.prepare_intaking;
                        autoFSM.update();
                    })
                    .lineToConstantHeading(new Vector2d(-55.00, -9.50))
                    .build();
            result.repeatToBackdrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setConstraints(
                            new MecanumVelocityConstraint(56, DriveConstants.TRACK_WIDTH),
                            new ProfileAccelerationConstraint(100))
                    .lineToConstantHeading(new Vector2d(-50.00, -9.50))
                    .resetConstraints()
                    .lineToConstantHeading(new Vector2d(30.00, -9.50))
                    .lineToConstantHeading(new Vector2d(50.65, -36.00))
                    .build();
        } else {
            result.repeatToStack = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(30.00, 9.50))
                    .lineToConstantHeading(new Vector2d(-50.00, 9.50))
                    .addDisplacementMarker(() ->
                    {
                        autoFSM.state = AutoFSM.ArmState.prepare_intaking;
                        autoFSM.update();
                    })
                    .lineToConstantHeading(new Vector2d(-55.00, 9.50))
                    .build();
            result.repeatToBackdrop = () -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setConstraints(
                            new MecanumVelocityConstraint(56, DriveConstants.TRACK_WIDTH),
                            new ProfileAccelerationConstraint(100))
                    .lineToConstantHeading(new Vector2d(-50.00, 9.50))
                    .resetConstraints()
                    .lineToConstantHeading(new Vector2d(30.00, 9.50))
                    .lineToConstantHeading(new Vector2d(50.65, 36.00))
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
        autoFSM.state = AutoFSM.ArmState.purple_pixel;
        while (autoFSM.state != AutoFSM.ArmState.roadrunner) {
            autoFSM.update();
        }

        // Go to allocated position (may not be necessary but still keep here just in case)
        // drive.followTrajectorySequence(autoTrajectory.afterPurplePixel);

        // Go to backdrop for yellow pixel drop
        drive.followTrajectorySequence(autoTrajectory.yellowPixelDrop.get());

        // Drop yellow Pixel
        autoFSM.waitServoTimer.reset();
        autoFSM.state = AutoFSM.ArmState.yellow_pixel;
        while (autoFSM.state != AutoFSM.ArmState.roadrunner) {
            autoFSM.update();
        }
        while (opMode.opModeIsActive()) {
            // Go to stack
            TrajectorySequence repeatToStack = autoTrajectory.repeatToStack.get();
            drive.followTrajectorySequence(repeatToStack);

            // Intaking
            autoFSM.waitServoTimer.reset();
            autoFSM.state = AutoFSM.ArmState.intaking;
            while (autoFSM.state != AutoFSM.ArmState.roadrunner) {
                autoFSM.update();
            }

            // Go back FAST
            Pose2d pose2 = drive.getPoseEstimate();
            drive.followTrajectorySequenceAsync(
                    drive.trajectorySequenceBuilder(pose2)
//                            .setConstraints(new MecanumVelocityConstraint(), new Meca)
                            .lineTo(new Vector2d(pose2.getX() + 5, pose2.getY()))
                            .build());

            // Go to backdrop
            autoFSM.state = AutoFSM.ArmState.after_intake;
            autoFSM.update();
            TrajectorySequence repeatToBackdrop = autoTrajectory.repeatToBackdrop.get();
            drive.followTrajectorySequence(repeatToBackdrop);

            // Outtake Pixels
            autoFSM.timeoutTimer.reset();
            autoFSM.arm.setTargetAngle(ARM_BACKDROP_PARALLEL_ANGLE_AUTO);
            autoFSM.state = AutoFSM.ArmState.arm_moving_up;
            while (autoFSM.state != AutoFSM.ArmState.roadrunner) {
                autoFSM.update();
            }
            if (30 - timer.seconds() <= repeatToStack.duration() + repeatToBackdrop.duration() + 5)
                break;
        }
    }
}
