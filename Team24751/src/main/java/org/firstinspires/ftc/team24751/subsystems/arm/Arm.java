package org.firstinspires.ftc.team24751.subsystems.arm;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.*;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Utils.Timer;
import com.ThermalEquilibrium.homeostasis.Utils.WPILibMotionProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm {
    public DcMotorEx leftArmMotor;
    public DcMotorEx rightArmMotor;
    LinearOpMode opMode;
    PIDEx positionPID = new PIDEx(ARM_POSITION_PID_COEFFICIENTS);
    PIDEx distancePID = new PIDEx(ARM_DISTANCE_PID_COEFFICIENTS);
    FeedforwardEx feedforward = new FeedforwardEx(ARM_VELOCITY_FEEDFORWARD_COEFFICIENTS);
    Double targetAngle = null;
    WPILibMotionProfile motionProfile = null;
    Timer timer = new Timer();

    public Arm(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public DcMotorEx getArm() {
        return rightArmMotor;
    }


    private double degToTick(double deg) {
        return (deg - MOTOR_DEG_AT_ZERO_TICK) / MOTOR_DEG_PER_TICK;
    }

    public double getAngle() {
        return rightArmMotor.getCurrentPosition() * MOTOR_DEG_PER_TICK + MOTOR_DEG_AT_ZERO_TICK;
    }

    /**
     * @param targetAngle targetAngle to be rotated to by PID, null to disable PID
     */
    public void setTargetAngle(Double targetAngle) {
        this.targetAngle = targetAngle;
        positionPID = new PIDEx(ARM_POSITION_PID_COEFFICIENTS);
        if (targetAngle != null)
            motionProfile = new WPILibMotionProfile(
                    ARM_VA_CONSTRAINT,
                    new WPILibMotionProfile.State(targetAngle, 0),
                    new WPILibMotionProfile.State(getAngle(), 0));
        timer.reset();
    }

    public void resetPID() {
        positionPID = new PIDEx(ARM_POSITION_PID_COEFFICIENTS);
        distancePID = new PIDEx(ARM_DISTANCE_PID_COEFFICIENTS);
    }

    public boolean anglePIDLoop() {
        if (targetAngle == null) {
            positionPID.calculate(0, getAngle());
            return true;
        }
        WPILibMotionProfile.State targetState = motionProfile.calculate(timer.currentTime());
        double pidPow = positionPID.calculate(targetState.position, getAngle());
        if (Math.abs(getAngle() - targetAngle) < POSITION_THRESHOLD) {
            leftArmMotor.setPower(0);
            rightArmMotor.setPower(0);
            return true;
        }
        //Probably should switch to PositionVelocitySystem
        double ffPow = feedforward.calculate(
                targetState.position,
                targetState.velocity, 0);
        leftArmMotor.setPower(ffPow);
        rightArmMotor.setPower(ffPow);
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
        rightArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void resetEncoder() {
        leftArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftArmMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}