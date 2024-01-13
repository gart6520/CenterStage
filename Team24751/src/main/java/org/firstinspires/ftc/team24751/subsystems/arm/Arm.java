package org.firstinspires.ftc.team24751.subsystems.arm;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.*;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm {
    public DcMotorEx leftArmMotor;
    public DcMotorEx rightArmMotor;
    LinearOpMode opMode;
    PIDEx pid = new PIDEx(ARM_POSITION_PID_COEFFICIENTS);
    FeedforwardEx feedforward = new FeedforwardEx(ARM_VELOCITY_FEEDFORWARD_COEFFICIENTS);
    Double targetAngle = null;

    public Arm(LinearOpMode _opMode) {
        opMode = _opMode;
    }


    private double degToTick(double deg) {
        return (deg - MOTOR_DEG_AT_ZERO_TICK) / MOTOR_DEG_PER_TICK;
    }

    public double getAngle() {
        return rightArmMotor.getCurrentPosition() * MOTOR_DEG_PER_TICK + MOTOR_DEG_AT_ZERO_TICK;
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    public void loop() {
        if (targetAngle == null) {
            return;
        }
        if (Math.abs(getAngle() - targetAngle) < POSITION_THRESHOLD) {
            leftArmMotor.setPower(0);
            rightArmMotor.setPower(0);
            return;
        }
        //Probably should switch to PositionVelocitySystem
        double vel = pid.calculate(degToTick(targetAngle), rightArmMotor.getCurrentPosition());
        double power = feedforward.calculate(getAngle(), vel, 0);
        leftArmMotor.setPower(power);
        rightArmMotor.setPower(power);
    }

    public void setPower(double power) {
        leftArmMotor.setPower(power);
        rightArmMotor.setPower(power);
    }

    public void init() {
        leftArmMotor = opMode.hardwareMap.get(DcMotorEx.class, LEFT_ARM_MOTOR);
        rightArmMotor = opMode.hardwareMap.get(DcMotorEx.class, RIGHT_ARM_MOTOR);
        rightArmMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
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