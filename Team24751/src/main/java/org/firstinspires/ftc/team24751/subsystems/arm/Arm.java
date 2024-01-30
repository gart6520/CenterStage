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

public class Arm {
    public DcMotorEx leftArmMotor;
    public DcMotorEx rightArmMotor;
    VoltageSensor batteryVoltageSensor;
    LinearOpMode opMode;
    PIDEx anglePID = new PIDEx(ARM_ANGLE_PID_COEFFICIENTS);
    PIDEx distancePID = new PIDEx(ARM_DISTANCE_PID_COEFFICIENTS);
    FeedforwardEx feedforward = new FeedforwardEx(ARM_VELOCITY_FEEDFORWARD_COEFFICIENTS);
    Double targetAngle = null;
    WPILibMotionProfile motionProfile = null;
    ElapsedTime timer = new ElapsedTime();
    double prevAngle = 0;
    double prevTime = 0;
    double angularVelocity = 0;
    ElapsedTime derivativeTimer = new ElapsedTime();

    public Arm(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public void update() {
        angularVelocity = (getAngle() - prevAngle) / (derivativeTimer.seconds() - prevTime);
        prevAngle = getAngle();
        prevTime = derivativeTimer.seconds();
    }

    private double degToTick(double deg) {
        return (deg - MOTOR_DEG_AT_ZERO_TICK) / MOTOR_DEG_PER_TICK;
    }

    public double getAngle() {
        return leftArmMotor.getCurrentPosition() * MOTOR_DEG_PER_TICK + MOTOR_DEG_AT_ZERO_TICK;
    }

    /**
     * @param targetAngle targetAngle to be rotated to by PID, null to disable PID
     */
    public void setTargetAngle(Double targetAngle) {
        this.targetAngle = targetAngle;
        anglePID = new PIDEx(ARM_ANGLE_PID_COEFFICIENTS);
        if (targetAngle != null)
            motionProfile = new WPILibMotionProfile(
                    ARM_VA_CONSTRAINT,
                    new WPILibMotionProfile.State(targetAngle, 0),
                    new WPILibMotionProfile.State(getAngle(), 0));
        timer.reset();
    }

    public void resetPID() {
        anglePID = new PIDEx(ARM_ANGLE_PID_COEFFICIENTS);
        distancePID = new PIDEx(ARM_DISTANCE_PID_COEFFICIENTS);
    }

    public boolean anglePIDLoop() {
        if (targetAngle == null) {
            anglePID.calculate(0, getAngle());
            return true;
        }
        WPILibMotionProfile.State targetState = motionProfile.calculate(timer.seconds());
        double rawPIDPow = anglePID.calculate(targetState.position, getAngle());
        double pidPow = Math.max(Math.abs(rawPIDPow), 0.15) * Math.signum(rawPIDPow);
        if (Math.abs(getAngle() - targetAngle) < ANGLE_THRESHOLD) {
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
        if (Math.abs(currentDistance - targetDistance) < DISTANCE_THRESHOLD) {
            leftArmMotor.setPower(0);
            rightArmMotor.setPower(0);
            return;
        }
        opMode.telemetry.addLine("Distance PID output: " + vel);
        leftArmMotor.setPower(vel);
        rightArmMotor.setPower(vel);
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

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
    }

    public void resetEncoder() {
        leftArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftArmMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        prevAngle = 0;
    }
}