package org.firstinspires.ftc.team24751.subsystems;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Twist2dDual;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team24751.subsystems.rrhelper.Encoder;
import org.firstinspires.ftc.team24751.subsystems.rrhelper.OverflowEncoder;
import org.firstinspires.ftc.team24751.subsystems.rrhelper.PositionVelocityPair;
import org.firstinspires.ftc.team24751.subsystems.rrhelper.RawEncoder;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;
import static org.firstinspires.ftc.team24751.Constants.DRIVEBASE.*;

import java.util.LinkedList;

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

    // Kinematics
    private final MecanumKinematics kinematics = new MecanumKinematics(TRACK_WIDTH, WHEELBASE_DISTANCE, LATERAL_MULTIPLER);

    // Localizer
    private DriveLocalizer localizer = null;

    // Pose
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();
    private Pose2d pose;


    /**
     * Drivebase class for driving mecanum drivebase
     * @param opMode opMode instance. If you are init this from linearOpMode, just pass `this`
     */
    public Drivebase(LinearOpMode opMode, Gyro gyro) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.gyro = gyro;
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
     */
    public void init() {
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

        // Reset encoders
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Stop all motors
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // Init localizer
        localizer = new DriveLocalizer();
    }

    /**
     * Drive method for Drivebase class - Mecanum drive
     * @param xSpeed horizontal speed. Negative is to the left
     * @param ySpeed vertical speed. Positive is forward
     * @param zSpeed rotate speed. Negative is rotate counterclockwise
     */
    public void drive(double xSpeed, double ySpeed, double zSpeed) {
        // Combine the requested power for each axis to determine each wheel's power.
        double leftMasterPower  = ySpeed + xSpeed + zSpeed;
        double rightMasterPower = ySpeed - xSpeed - zSpeed;
        double leftFollowPower  = ySpeed - xSpeed + zSpeed;
        double rightFollowPower = ySpeed + xSpeed - zSpeed;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        // Find max power value
        double max = Math.max(Math.abs(leftMasterPower), Math.abs(rightMasterPower));
        max = Math.max(max, Math.abs(leftFollowPower));
        max = Math.max(max, Math.abs(rightFollowPower));

        // If max power value > 1.0 -> need to trim that down to factor, not to overflow the max motor power value
        if (max > 1.0) {
            leftMasterPower  /= max;
            rightMasterPower /= max;
            leftFollowPower  /= max;
            rightFollowPower /= max;
        }

        // Set motor power
        leftFront.setPower(leftMasterPower);
        leftBack.setPower(leftFollowPower);
        rightFront.setPower(rightMasterPower);
        rightBack.setPower(rightFollowPower);

        // Update pose estimation
        updatePoseEstimate();

        // Show realtime info
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftMasterPower, rightMasterPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftFollowPower, rightFollowPower);
        telemetry.addData("Pose2d", "X: %5.2f (in); Y: %5.2f (in); Z: %4.2f (rad)", pose.position.x, pose.position.y, pose.heading);
        telemetry.update();
    }

    /**
     * Drivebase's drive localizer class
     * This class provides methods for getting/updating/resetting the drivebase's pose, based on the drivebase's wheel encoders
     */
    public class DriveLocalizer {
        private Encoder leftFrontEncoder, leftBackEncoder, rightFrontEncoder, rightBackEncoder;
        private int lastLeftFrontPos, lastLeftBackPos, lastRightFrontPos, lastRightBackPos;
        private Rotation2d lastHeading;

        /**
         * PoseEstimator's constructor
         * This constructor:
         * - Init encoder objects
         * - Get last encoder position
         */
        public DriveLocalizer() {
            // Init encoders
            leftFrontEncoder = new OverflowEncoder(new RawEncoder(leftFront));
            leftBackEncoder = new OverflowEncoder(new RawEncoder(leftBack));
            rightFrontEncoder = new OverflowEncoder(new RawEncoder(rightFront));
            rightBackEncoder = new OverflowEncoder(new RawEncoder(rightBack));

            // Get last position
            lastLeftFrontPos = leftFrontEncoder.getPositionAndVelocity().position;
            lastLeftBackPos = leftBackEncoder.getPositionAndVelocity().position;
            lastRightFrontPos = leftFrontEncoder.getPositionAndVelocity().position;
            lastRightBackPos = leftBackEncoder.getPositionAndVelocity().position;

            // Get last heading
            lastHeading = Rotation2d.exp(gyro.getYaw());
        }

        /**
         * Update localizer
         * @return twist (delta pose/pose different)
         */
        public Twist2dDual<Time> update() {
            // Get position and velocity of each wheel
            PositionVelocityPair leftFrontPosVel = leftFrontEncoder.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBackEncoder.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFrontEncoder.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBackEncoder.getPositionAndVelocity();

            // Get heading delta
            Rotation2d heading = Rotation2d.exp(gyro.getYaw());
            double headingDelta = heading.minus(lastHeading);

            // Compute twist
            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(IN_PER_TICK),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(IN_PER_TICK),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(IN_PER_TICK),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(IN_PER_TICK)
            ));

            // Update last position and heading
            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;
            lastHeading = heading;

            // Return
            return new Twist2dDual<>(
                    twist.line,
                    DualNum.cons(headingDelta, twist.angle.drop(1))
            );
        }
    }

    /**
     * Update pose estimation
     * This should be called frequently whenever the drivebase is moving
     * @return 2D velocity
     */
    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        //FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));

        return twist.velocity().value();
    }
}
