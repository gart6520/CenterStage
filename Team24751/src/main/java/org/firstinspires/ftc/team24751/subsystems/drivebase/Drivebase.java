package org.firstinspires.ftc.team24751.subsystems.drivebase;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.drivebase.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team24751.subsystems.drivebase.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.team24751.subsystems.drivebase.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.team24751.subsystems.drivebase.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.team24751.Constants.GAMEPAD_SENSITIVITY.*;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;
import static org.firstinspires.ftc.team24751.Constants.SPEED.*;
import static org.firstinspires.ftc.team24751.subsystems.drivebase.DriveConstants.*;
import static org.firstinspires.ftc.team24751.Utility.sense;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class Drivebase extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(2, 0, 1.5);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(2, 0, 0.2);

    public static double LATERAL_MULTIPLIER = 1.49;

    // Drive speed
    // Should be 1, and leave speed control for opMode
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private List<DcMotorEx> motors;

    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();
    private Localizer localizer;
    private Pose2d deltaPose;
    private double driverAngle = 0;
    private LinearOpMode opMode;

    public Drivebase(LinearOpMode _opMode) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        opMode = _opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT);
        leftBack = hardwareMap.get(DcMotorEx.class, LEFT_BACK);
        rightBack = hardwareMap.get(DcMotorEx.class, RIGHT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT);

        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // TODO: if desired, use setLocalizer() to change the localization method
        localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
        setLocalizer(localizer);

        this.deltaPose = new Pose2d(0, 0, 0);

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        Pose2d currentPose = getPoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(currentPose, getPoseVelocity());
//        opMode.telemetry.addData("Current Pose", currentPose.toString());
        Pose2d targetPose = trajectorySequenceRunner.getLastPoseError();
//        opMode.telemetry.addData("Last Pose Error", targetPose.toString());
//        opMode.telemetry.update();
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftBack.setPower(v1);
        rightBack.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return 0.0;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    /**
     * Get current pose
     *
     * @return current fused pose (= estimated pose2d + delta pose2d)
     */
    public Pose2d getPoseFuse() {
        return this.getPoseEstimate().plus(deltaPose);
    }

    /**
     * Set new pose for drivebase localizer
     *
     * @param newPose new Pose2d to set
     * @return delta pose (from old pose2d to new pose2d)
     */
    public Pose2d setPoseFuse(Pose2d newPose) {
        return this.deltaPose = newPose.minus(this.getPoseEstimate());
    }

    /**
     * Set driver angle
     * Field-oriented drive will depend on this angle
     */
    public void setDriverAngle(double angle) {
        this.driverAngle = angle;
    }

    /**
     * Drive method for Drivebase class - Mecanum drive
     * Note: this method already include pose update, so you don't have to manually call update()
     *
     * @param xSpeed horizontal speed. Negative is to the left
     * @param ySpeed vertical speed. Positive is forward
     * @param zSpeed rotate speed. Negative is rotate counterclockwise
     */
    public void drive(double xSpeed, double ySpeed, double zSpeed) {
        // Drive
        this.setWeightedDrivePower(new Pose2d(ySpeed, -xSpeed, -zSpeed));

        // Update pose
        this.update();
    }

    /**
     * Field-oriented drive method for Drivebase class - Mecanum drive
     * Note: this method already include pose update, so you don't have to manually call update()
     *
     * @param xSpeed horizontal speed. Negative is to the left
     * @param ySpeed vertical speed. Positive is forward
     * @param zSpeed rotate speed. Negative is rotate counterclockwise
     */
    public void driveFieldOriented(double xSpeed, double ySpeed, double zSpeed) {
        // Get bot heading (from odometry pods)
        update(); // Update before getting pose
        double botHeading = this.getPoseEstimate().getHeading() - this.driverAngle;

        // Get the rotated velocity
        double rotX = xSpeed * Math.cos(-botHeading) - ySpeed * Math.sin(-botHeading);
        double rotY = xSpeed * Math.sin(-botHeading) + ySpeed * Math.cos(-botHeading);

        // Drive
        // Call update() twice, could cause problem
        this.drive(rotX, rotY, zSpeed);
    }

    /**
     * Call this method in the while loop in your opMode to enable drive using joystick
     * Note: this method already includes pose update from drive/driveFieldOriented, so
     * you don't need to call the update() function.
     *
     * @param fieldOriented whether the drive is field oriented (false is bot oriented)
     */
    public void manualControl(boolean fieldOriented) {
        // Control drivebase manually, using gamepad1's joystick
        // Check for boost button: if boost enabled -> run at max speed, otherwise run at half max speed
        double speed = opMode.gamepad1.right_trigger > SENSE_TRIGGER ? 1 : 0.5;

        // Get joystick axis values
        // Left joystick is used for driving bot in up/down/left/right direction, while right joystick is used for rotating the bot
        double left_y = -sense(opMode.gamepad1.left_stick_y, SENSE_Y); // Y axis is inverted
        double left_x = sense(opMode.gamepad1.left_stick_x, SENSE_X);
        double right_x;

        if (Math.abs(opMode.gamepad1.right_stick_x) >= SENSE_Z)
            right_x = sense(opMode.gamepad1.right_stick_x, SENSE_Z);
        else {
            right_x = sense(opMode.gamepad2.right_stick_x, SENSE_Z);
        }

        if (opMode.gamepad1.right_trigger <= SENSE_TRIGGER) {
            left_y = Math.pow(left_y, 5);
            left_x = Math.pow(left_x, 5);
            right_x = Math.pow(right_x, 5);
        }
        double linearY = left_y * speed * DRIVEBASE_SPEED_Y;
        double linearX = left_x * speed * DRIVEBASE_SPEED_X;
        double angular = right_x * speed * DRIVEBASE_SPEED_Z;


        // Drive
        // Hopefully we will never have to switch back to drive bot-oriented
        if (fieldOriented) {
            // Dpad buttons state
            boolean dpad_up = opMode.gamepad1.dpad_up || opMode.gamepad2.dpad_up;
            boolean dpad_down = opMode.gamepad1.dpad_down || opMode.gamepad2.dpad_down;
            boolean dpad_left = opMode.gamepad1.dpad_left || opMode.gamepad2.dpad_left;
            boolean dpad_right = opMode.gamepad1.dpad_right || opMode.gamepad2.dpad_right;

            if (!dpad_up && !dpad_down && !dpad_left && !dpad_right) {
                // No dpad is pressed -> normal field-oriented driving using joystick
                this.driveFieldOriented(linearX, linearY, angular);
            } else {
                // If dpad is pressed -> stop driving field-oriented, temporarily follow bot-oriented using dpad

                // No turn in this mode
                angular = 0;

                if (dpad_up) {
                    // Move up
                    linearX = 0;
                    linearY = DRIVEBASE_SPEED_X * speed;
                } else if (dpad_down) {
                    // Move down
                    linearX = 0;
                    linearY = -DRIVEBASE_SPEED_X * speed;
                } else if (dpad_left) {
                    // Move to the left
                    linearX = -DRIVEBASE_SPEED_X * speed;
                    linearY = 0;
                } else {
                    // Move to the right
                    linearX = DRIVEBASE_SPEED_X * speed;
                    linearY = 0;
                }

                // Drive bot oriented
                this.drive(linearX, linearY, angular);
            }
        } else {
            this.drive(linearX, linearY, angular); // Drive bot-oriented
        }
    }

    // TODO Oudated, dont use if havent update

    /**
     * Call this method in the while loop in your opMode to enable drive using joystick
     * Note: this method already includes pose update from drive/driveFieldOriented, so
     * you don't need to call the update() function. This method will also limit the
     * drivebase's max speed, which is useful when driving close to the backdrop
     *
     * @param fieldOriented whether the drive is field oriented (false is bot oriented)
     */
    @Deprecated
    public void manualControlLimitSpeed(boolean fieldOriented) {
        // Control drivebase manually, using gamepad1's joystick
        // Check for boost button: if boost enabled -> run at max speed, otherwise run at half max speed
        double speed = opMode.gamepad1.right_trigger > SENSE_TRIGGER ? 0.3 : 0.2;

        // Get joystick axis values
        // Left joystick is used for driving bot in up/down/left/right direction, while right joystick is used for rotating the bot
        double left_y = -sense(opMode.gamepad1.left_stick_y, SENSE_Y); // Y axis is inverted
        double left_x = sense(opMode.gamepad1.left_stick_x, SENSE_X);
        double right_x = sense(opMode.gamepad1.right_stick_x, SENSE_Z);
        if (opMode.gamepad1.right_trigger <= SENSE_TRIGGER) {
            left_y = Math.pow(left_y, 5);
            left_x = Math.pow(left_x, 5);
            right_x = Math.pow(right_x, 5);
        }
        double linearY = left_y * speed * DRIVEBASE_SPEED_Y;
        double linearX = left_x * speed * DRIVEBASE_SPEED_X;
        double angular = right_x * speed * DRIVEBASE_SPEED_Z;


        // Drive
        // Hopefully we will never have to switch back to drive bot-oriented
        if (fieldOriented) {
            // Dpad buttons state
            boolean dpad_up = opMode.gamepad2.dpad_up;
            boolean dpad_down = opMode.gamepad2.dpad_down;
            boolean dpad_left = opMode.gamepad2.dpad_left;
            boolean dpad_right = opMode.gamepad2.dpad_right;

            if (!dpad_up && !dpad_down && !dpad_left && !dpad_right) {
                // No dpad is pressed -> normal field-oriented driving using joystick
                this.driveFieldOriented(linearX, linearY, angular);
            } else {
                // If dpad is pressed -> stop driving field-oriented, temporarily follow bot-oriented using dpad

                // No turn in this mode
                angular = 0;

                if (dpad_up) {
                    // Move up
                    linearX = 0;
                    linearY = DRIVEBASE_SPEED_X * speed;
                } else if (dpad_down) {
                    // Move down
                    linearX = 0;
                    linearY = -DRIVEBASE_SPEED_X * speed;
                } else if (dpad_left) {
                    // Move to the left
                    linearX = -DRIVEBASE_SPEED_X * speed;
                    linearY = 0;
                } else {
                    // Move to the right
                    linearX = DRIVEBASE_SPEED_X * speed;
                    linearY = 0;
                }

                // Drive bot oriented
                this.drive(linearX, linearY, angular);
            }
        } else {
            this.drive(linearX, linearY, angular); // Drive bot-oriented
        }
    }
}
