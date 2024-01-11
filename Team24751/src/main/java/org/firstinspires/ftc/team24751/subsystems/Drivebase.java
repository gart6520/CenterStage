package org.firstinspires.ftc.team24751.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;
import static org.firstinspires.ftc.team24751.Constants.DRIVEBASE.*;

import androidx.annotation.NonNull;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/**
 * Drivebase class for driving mecanum drivebase
 */
public class Drivebase {
    // Useful instances
    private HardwareMap hardwareMap = null;
    private Telemetry telemetry = null;
    private Gyro gyro = null;

    // Drivebase motors
    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;

    // Voltage sensor
    private VoltageSensor voltageSensor = null;

    // Kinematics
    private final MecanumKinematics kinematics = new MecanumKinematics(TRACK_WIDTH, WHEELBASE_DISTANCE, LATERAL_MULTIPLER);

    // Localizer
    private OdometryPod localizer = null;

    // Pose
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();
    public Pose2d pose;

    // Default constraints
    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            MAX_ANG_VEL, -MAX_ANG_ACCEL, MAX_ANG_ACCEL);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(MAX_WHEEL_VEL),
                    new AngularVelConstraint(MAX_ANG_VEL)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(MIN_PROFILE_ACCEL, MAX_PROFILE_ACCEL);

    /**
     * Drivebase class for driving mecanum drivebase
     */
    public Drivebase() {}

    /**
     * Init method for Drivebase class
     * <p>
     * This method:
     * - gets motor objects from provided hardwareMap variable
     * - set drivebase motor direction
     * - set brake mode for all motors
     * - reset motor encoders
     * - stop all motors on init
     * - init localizer
     * </p>
     * @param opMode opMode instance. If you are init this from linearOpMode, just pass `this`
     * @param gyro gyro instance
     * @param localizer OdometryPod localizer
     */
    public void init(LinearOpMode opMode, Gyro gyro, OdometryPod localizer) {
        // Get hardware map
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.gyro = gyro;

        // Get motor objects
        leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT);
        leftBack = hardwareMap.get(DcMotorEx.class, LEFT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT);
        rightBack = hardwareMap.get(DcMotorEx.class, RIGHT_BACK);

        // Set drivebase motor direction
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set brake mode
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set localizer object
        this.localizer = localizer;

        // Stop all motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // Init voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    /**
     * Init method for Drivebase class
     * <p>
     * This method:
     * - gets motor objects from provided hardwareMap variable
     * - set drivebase motor direction
     * - set brake mode for all motors
     * - reset motor encoders
     * - stop all motors on init
     * - init localizer
     * </p>
     * @param opMode opMode instance. If you are init this from linearOpMode, just pass `this`
     * @param gyro gyro instance
     */
    public void init(LinearOpMode opMode, Gyro gyro) {
        // Get hardware map
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.gyro = gyro;

        // Get motor objects
        leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT);
        leftBack = hardwareMap.get(DcMotorEx.class, LEFT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT);
        rightBack = hardwareMap.get(DcMotorEx.class, RIGHT_BACK);

        // Set drivebase motor direction
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set brake mode
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Stop all motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // Init voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    /**
     * Reset encoder, and enable run USING encoder
     * This mode is used for auto and semi-auto
     */
    public void resetRunUsingEncoder() {
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Reset encoder, and enable run WITHOUT encoder
     * This mode is used for manual control, so the driver have full control of the drivebase
     */
    public void resetRunWithoutEncoder() {
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Drive method for Drivebase class - Mecanum drive
     * @param xSpeed horizontal speed. Negative is to the left
     * @param ySpeed vertical speed. Positive is forward
     * @param zSpeed rotate speed. Negative is rotate counterclockwise
     */
    public void drive(double xSpeed, double ySpeed, double zSpeed) {
        // Combine the requested power for each axis to determine each wheel's power.
        double leftFrontPower  = ySpeed + xSpeed + zSpeed;
        double rightFrontPower = ySpeed - xSpeed - zSpeed;
        double leftBackPower   = ySpeed - xSpeed + zSpeed;
        double rightBackPower  = ySpeed + xSpeed - zSpeed;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        // Find max power value
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        // If max power value > 1.0 -> need to trim that down to factor, not to overflow the max motor power value
        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Set motor power
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);

        // Update pose estimation
        updatePoseEstimate();

        // Show realtime info
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        //telemetry.update();
    }

    /**
     * Drive field-oriented - Mecanum drive
     * @param xSpeed horizontal speed. Negative is to the left
     * @param ySpeed vertical speed. Positive is forward
     * @param zSpeed rotate speed. Negative is rotate counterclockwise
     */
    public void driveFieldOriented(double xSpeed, double ySpeed, double zSpeed) {
        // Rotate the movement direction counter to the bot's rotation
        double botHeading = gyro.getYawRad();
        double rotX = xSpeed * Math.cos(-botHeading) - ySpeed * Math.sin(-botHeading);
        double rotY = xSpeed * Math.sin(-botHeading) + ySpeed * Math.cos(-botHeading);

        // Drive
        drive(rotX, rotY, zSpeed);
    }

    /**
     * Update pose estimation
     * This should be called frequently whenever the drivebase is moving
     * @return 2D velocity
     */
    public PoseVelocity2d updatePoseEstimate() {
        // If localizer is enabled
        if (localizer != null) {
            Twist2dDual<Time> twist = localizer.update();
            pose = pose.plus(twist.value());

            poseHistory.add(pose);
            while (poseHistory.size() > 100) {
                poseHistory.removeFirst();
            }

            //FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));

            telemetry.addData("Pose2d", "X: %5.2f (in); Y: %5.2f (in); Z: %4.2f (degree)", pose.position.x, pose.position.y, pose.heading.toDouble()*180/Math.PI);
            //telemetry.update();

            return twist.velocity().value();
        } else {
            // No localizer
            return new PoseVelocity2d(new Vector2d(0, 0), 0);
        }
    }

    /**
     * FollowTrajectoryAction: following a pre-calculated trajectory
     */
    public class FollowTrajectoryAction implements Action {
        private final TimeTrajectory timeTrajectory;
        private final double[] xPoints, yPoints;
        private double beginTs = -1;

        /**
         * FollowTrajectoryAction: following a pre-calculated trajectory
         * @param t TimeTrajectory object
         */
        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            // [Optional] Calculate the planned points to (later) draw on the motion graph
            // You can remove it if it makes the robot laggy
            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        /**
         * Follow the specified path
         * This method should be called by Action, and not by you
         * @param p TelemetryPacket
         * @return true if still running, false if stopped
         */
        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            // Init timer
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            // If run time exceeded -> stop the drivebase
            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            // Get target precomputed pose for the current time
            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);

            // Update current realworld pose
            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            // Calculate pose velocity for each time frame
            PoseVelocity2dDual<Time> command = new HolonomicController(
                    AXIAL_GAIN, LATERAL_GAIN, HEADING_GAIN,
                    AXIAL_VEL_GAIN, LATERAL_VEL_GAIN, HEADING_VEL_GAIN
            )
                    .compute(txWorldTarget, pose, robotVelRobot);

            // Compute wheel velocity
            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);

            // Get voltage
            double voltage = voltageSensor.getVoltage();

            // Create feedforward controller for motor speed
            final MotorFeedforward feedforward = new MotorFeedforward(kS, kV, kA);

            // Feed the computed velocity to each motors
            // We need to convert from inch/s to motor velocity (tick/s)
            // This can be done either by feedforward PID or using built-in PID/PIDF feedback controller
            // provided in the DcMotorEx class
            // Velocity MUST be something that is reachable regardless the battery is full or not
            /*leftFront.setVelocity(wheelVels.leftFront.value() / IN_PER_TICK);
            leftBack.setVelocity(wheelVels.leftBack.value() / IN_PER_TICK);
            rightFront.setVelocity(wheelVels.rightFront.value() / IN_PER_TICK);
            rightBack.setVelocity(wheelVels.rightBack.value() / IN_PER_TICK);*/

            leftFront.setVelocity(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setVelocity(feedforward.compute(wheelVels.leftBack) / voltage);
            rightFront.setVelocity(feedforward.compute(wheelVels.rightFront) / voltage);
            rightBack.setVelocity(feedforward.compute(wheelVels.rightBack) / voltage);

            // Draw path for visualization
            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.log()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.log()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    /**
     * TurnAction: follow a pre-calculated turn
     */
    public class TurnAction implements Action {
        private final TimeTurn timeTurn;
        private double beginTs = -1;

        /**
         * TurnAction: follow a pre-calculated turn
         * @param t TimeTurn object
         */
        public TurnAction(TimeTurn t) {
            timeTurn = t;
        }

        /**
         * Follow the specified turn action
         * This method should be called by Action, and not by you
         * @param p TelemetryPacket
         * @return true if still running, false if stopped
         */
        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            // Init timer
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            // If run time exceeded -> stop the drivebase
            if (t >= timeTurn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            // Get target precomputed pose for the current time
            Pose2dDual<Time> txWorldTarget = timeTurn.get(t);

            // Update current realworld pose
            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            // Calculate pose velocity for each time frame
            PoseVelocity2dDual<Time> command = new HolonomicController(
                    AXIAL_GAIN, LATERAL_GAIN, HEADING_GAIN,
                    AXIAL_VEL_GAIN, LATERAL_VEL_GAIN, HEADING_VEL_GAIN
            )
                    .compute(txWorldTarget, pose, robotVelRobot);

            // Compute wheel velocity
            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);

            // Get voltage
            double voltage = voltageSensor.getVoltage();

            // Create feedforward controller for motor speed
            final MotorFeedforward feedforward = new MotorFeedforward(kS, kV, kA);

            // Feed the computed velocity to each motors
            // We need to convert from inch/s to motor velocity (tick/s)
            // This can be done either by feedforward PID or using built-in PID/PIDF feedback controller
            // provided in the DcMotorEx class
            // Velocity MUST be something that is reachable regardless the battery is full or not
            /*leftFront.setVelocity(wheelVels.leftFront.value() / IN_PER_TICK);
            leftBack.setVelocity(wheelVels.leftBack.value() / IN_PER_TICK);
            rightFront.setVelocity(wheelVels.rightFront.value() / IN_PER_TICK);
            rightBack.setVelocity(wheelVels.rightBack.value() / IN_PER_TICK);*/

            leftFront.setVelocity(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setVelocity(feedforward.compute(wheelVels.leftBack) / voltage);
            rightFront.setVelocity(feedforward.compute(wheelVels.rightFront) / voltage);
            rightBack.setVelocity(feedforward.compute(wheelVels.rightBack) / voltage);

            // Draw turn for visualization
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(timeTurn.beginPose.position.x, timeTurn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(timeTurn.beginPose.position.x, timeTurn.beginPose.position.y, 2);
        }
    }

    /**
     * Draw pose history
     * For visualization purpose
     */
    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    /**
     * Draw robot (in the pose history map)
     * For visualization purpose
     */
    private static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    /**
     * Create TrajectoryActionBuilder object chaining run actions
     * @param beginPose pose before auto run
     * @return TrajectoryActionBuilder
     */
    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                beginPose, 1e-6, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint,
                0.25, 0.1
        );
    }

    /**
     * Set current drivetrain's pose to a specified pose
     * Useful for:
     * - Transfering pose state from auto to manual mode
     * - Update pose using pose from AprilTag
     * @param pose Pose2d object to set to
     */
    public void setCurrentPose(Pose2d pose) {
        this.pose = pose;
    }
}
