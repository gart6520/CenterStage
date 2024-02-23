package org.firstinspires.ftc.team24751.subsystems.arm;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.*;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Utils.WPILibMotionProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.Encoder;
import org.firstinspires.ftc.team24751.subsystems.FuseSensor;

public class Arm {
    DcMotorEx leftArmMotor;
    DcMotorEx rightArmMotor;
    public Encoder leftArmEncoder;
    public Encoder rightArmEncoder;
    VoltageSensor batteryVoltageSensor;
    LinearOpMode opMode;
    PIDEx outakePID = new PIDEx(ARM_OUTAKE_PID_COEFFICIENTS);
    PIDEx distancePID = new PIDEx(ARM_DISTANCE_PID_COEFFICIENTS);
    PIDEx autoOutakePID = new PIDEx(ARM_AUTO_OUTAKE_PID_COEFFICIENTS);

    FeedforwardEx feedforward = new FeedforwardEx(ARM_VELOCITY_FEEDFORWARD_COEFFICIENTS);
    Double targetAngle = null;
    WPILibMotionProfile motionProfile = null;
    ElapsedTime timer = new ElapsedTime();
    public WPILibMotionProfile.State targetState;
    FuseSensor armAngleEstimator = new FuseSensor(0, 1,
            new FuseSensor.FuseSensorParameter(3, 3, 1));
    double currentAngle = Double.NaN;

    public enum ArmEncoderStatus {
        BOTH, LEFT_ONLY, RIGHT_ONLY
    }

    ArmEncoderStatus armEncoderStatus = ArmEncoderStatus.BOTH;

    public Arm(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public ArmEncoderStatus getArmEncoderStatus()
    {
        return armEncoderStatus;
    }
    public void update() {

        int leftArmEncoderPosition = leftArmEncoder.getPosition();
        int rightArmEncoderPosition = rightArmEncoder.getPosition();
        // Right side is faulty
        if (leftArmEncoderPosition - rightArmEncoderPosition >= FAULTY_ARM_ENCODER_THRESHOLD
                || armEncoderStatus == ArmEncoderStatus.LEFT_ONLY) {
            armEncoderStatus = ArmEncoderStatus.LEFT_ONLY;
            currentAngle = armAngleEstimator.update(leftArmEncoderPosition, null);
            return;
        }
        // Left side is faulty
        if (rightArmEncoderPosition - leftArmEncoderPosition >= FAULTY_ARM_ENCODER_THRESHOLD
                || armEncoderStatus == ArmEncoderStatus.RIGHT_ONLY) {
            armEncoderStatus = ArmEncoderStatus.RIGHT_ONLY;
            currentAngle = armAngleEstimator.update(rightArmEncoderPosition, null);
            return;
        }
        currentAngle = armAngleEstimator.update(
                tickToDeg(leftArmEncoderPosition),
                tickToDeg(rightArmEncoderPosition));
    }

    public double tickToDeg(int tick) {
        return tick * MOTOR_DEG_PER_TICK + MOTOR_DEG_AT_ZERO_TICK;
    }

    /**
     * Call update() before this
     *
     * @see #update()
     */
    public double getAngle() {
        return currentAngle;
    }

    /**
     * @param targetAngle targetAngle to be rotated to by outake PID, null to disable PID
     */
    public void setTargetAngle(Double targetAngle) {
        this.targetAngle = targetAngle;
        outakePID = new PIDEx(ARM_OUTAKE_PID_COEFFICIENTS);
        autoOutakePID = new PIDEx(ARM_AUTO_OUTAKE_PID_COEFFICIENTS);
        if (targetAngle != null)
            motionProfile = new WPILibMotionProfile(
                    ARM_VA_CONSTRAINT,
                    new WPILibMotionProfile.State(targetAngle, 0),
                    new WPILibMotionProfile.State(getAngle(), 0));
        timer.reset();
    }

    public void resetPID() {
        outakePID = new PIDEx(ARM_OUTAKE_PID_COEFFICIENTS);
        distancePID = new PIDEx(ARM_DISTANCE_PID_COEFFICIENTS);
    }

    public boolean outakePIDLoop() {
        if (targetAngle == null) {
            outakePID.calculate(0, getAngle());
            return true;
        }
        WPILibMotionProfile.State targetState = motionProfile.calculate(timer.seconds());
        this.targetState = targetState;
        double rawPIDPow = outakePID.calculate(targetState.position, getAngle());
        double pidPow = Math.max(Math.abs(rawPIDPow), ARM_ANGLE_MIN_PID_POW) * Math.signum(rawPIDPow);
        if (Math.abs(getAngle() - targetAngle) < ANGLE_TOLERANCE) {
            setPower(0);
            return true;
        }
        //Probably should switch to PositionVelocitySystem
        double ffPow = feedforward.calculate(
                Math.toRadians(targetState.position - getAngle()),
                targetState.velocity,
//                0,
                0);
        if (timer.seconds() > motionProfile.totalTime())
            ffPow = 0;
        setPower((ffPow + pidPow) * 12 / batteryVoltageSensor.getVoltage());
        opMode.telemetry.addData("Target State", targetState.position + " " + targetState.velocity);
        opMode.telemetry.addData("FF Pow", ffPow);
        opMode.telemetry.addData("PID Pow", pidPow);
        return false;
    }

    public void distancePIDLoop(double currentDistance, double targetDistance) {
        double vel = distancePID.calculate(currentDistance, targetDistance);
        if (Math.abs(currentDistance - targetDistance) < DISTANCE_TOLERANCE) {
            leftArmMotor.setPower(0);
            rightArmMotor.setPower(0);
            return;
        }
        opMode.telemetry.addLine("Distance PID output: " + vel);
        leftArmMotor.setPower(vel);
        rightArmMotor.setPower(vel);
    }

    public boolean autoOutakePIDLoop() {
        if (targetAngle == null) {
            autoOutakePID.calculate(0, getAngle());
            return true;
        }
        if (Math.abs(getAngle() - targetAngle) < ANGLE_TOLERANCE) {
            setPower(0);
            return true;
        }
        setPower(autoOutakePID.calculate(targetAngle, getAngle()));
        return false;
    }

    public void setPower(double power) {
        leftArmMotor.setPower(power);
        rightArmMotor.setPower(power);
    }

    public void init() {
        leftArmMotor = opMode.hardwareMap.get(DcMotorEx.class, LEFT_ARM_MOTOR);
        rightArmMotor = opMode.hardwareMap.get(DcMotorEx.class, RIGHT_ARM_MOTOR);
        leftArmMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftArmEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, LEFT_ARM_ENCODER));
        rightArmEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, RIGHT_ARM_ENCODER));
        leftArmEncoder.setDirection(Encoder.Direction.REVERSE);

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
    }

    public void resetEncoder() {
        leftArmEncoder.reset();
        rightArmEncoder.reset();
    }
}