package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.MOTOR_POSITION_AT_PERPENDICULAR;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.MOTOR_POSITION_AT_ZERO;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.SERVO_POSITION_AT_PERPENDICULAR;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.SERVO_POSITION_AT_ZERO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Autonomous(name = "AutoHitlerArm", group = "Test")
public class TestAutoHitlerArm extends LinearOpMode {
    private DcMotor leftArmMotor = null;
    private DcMotor rightArmMotor = null;
    private DcMotor elevatorMotor = null;
    private ServoImplEx wristServo = null;

    private double getMotorAngle(DcMotor motor) {
        double a = (MOTOR_POSITION_AT_PERPENDICULAR - MOTOR_POSITION_AT_ZERO) / 90.0;
        double b = MOTOR_POSITION_AT_ZERO;
        double currentPosition = motor.getCurrentPosition();
        return a * currentPosition + b;
    }
    private void setWristAngle() {
        double a = (SERVO_POSITION_AT_PERPENDICULAR - SERVO_POSITION_AT_ZERO) / 90.0;
        double b = SERVO_POSITION_AT_ZERO;
        double targetAngle = 240.0 - getMotorAngle(leftArmMotor);
        double ticks = (targetAngle - b) / a;
        wristServo.setPosition(ticks);
    }

    @Override
    public void runOpMode() {
        leftArmMotor = hardwareMap.get(DcMotor.class, "leftArmMotor");
        rightArmMotor = hardwareMap.get(DcMotor.class, "rightArmMotor");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        wristServo = hardwareMap.get(ServoImplEx.class, "wristServo");

        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO: Init servo position
        wristServo.setPwmRange(new PwmControl.PwmRange(550, 2450));

        waitForStart();

        while (opModeIsActive()) {
            setWristAngle();
            if (gamepad1.triangle) {
                leftArmMotor.setPower(0.8);
                rightArmMotor.setPower(0.8);
            } else if (gamepad1.cross) {
                leftArmMotor.setPower(-0.5);
                rightArmMotor.setPower(-0.5);
            } else {
                leftArmMotor.setPower(0);
                rightArmMotor.setPower(0);
            }

            if (gamepad1.circle) {
                elevatorMotor.setPower(0.8);
            } else if (gamepad1.square) {
                elevatorMotor.setPower(-0.8);
            } else {
                elevatorMotor.setPower(0);
            }
            telemetry.addData("Current Position", leftArmMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
