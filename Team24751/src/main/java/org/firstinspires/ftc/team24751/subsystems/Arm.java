package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.ARM_POSITION_PID_COEFFICIENTS;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.ARM_VELOCITY_FEEDFORWARD_COEFFICIENTS;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.MOTOR_DEG_AT_ZERO_TICK;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.MOTOR_DEG_PER_TICK;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.POSITION_THRESHOLD;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm {
    DcMotorEx armMotor;
    LinearOpMode opMode;
    BasicPID pid = new BasicPID(ARM_POSITION_PID_COEFFICIENTS);
    Double targetAngle = null;

    public Arm(LinearOpMode _opMode) {
        opMode = _opMode;
    }


    private double degToTick(double deg) {
        return (deg - MOTOR_DEG_AT_ZERO_TICK) / MOTOR_DEG_PER_TICK;
    }

    public double getAngle() {
        return armMotor.getCurrentPosition() * MOTOR_DEG_PER_TICK + MOTOR_DEG_AT_ZERO_TICK;
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    public void loop() {
        if (targetAngle == null) {
            return;
        }
        if (Math.abs(getAngle() - targetAngle) < POSITION_THRESHOLD) {
            armMotor.setVelocity(0);
            return;
        }
        double vel = pid.calculate(degToTick(targetAngle), armMotor.getCurrentPosition());
        armMotor.setVelocity(vel);
    }

    public void setPower(double power) {
        armMotor.setPower(power);
    }

    public void init() {
        armMotor = opMode.hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setVelocityPIDFCoefficients(
                ARM_VELOCITY_FEEDFORWARD_COEFFICIENTS.p,
                ARM_VELOCITY_FEEDFORWARD_COEFFICIENTS.i,
                ARM_VELOCITY_FEEDFORWARD_COEFFICIENTS.d,
                ARM_VELOCITY_FEEDFORWARD_COEFFICIENTS.f);
    }

    public void resetEncoder() {
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}
