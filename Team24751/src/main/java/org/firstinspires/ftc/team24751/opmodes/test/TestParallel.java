package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.INITAL_WRIST_SERVO_ANGLE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.MOTOR_POSITION_AT_PERPENDICULAR;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.MOTOR_POSITION_AT_ZERO;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.REV_SERVO_ANGLE_RANGE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.REV_SERVO_PWM_RANGE;
//import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.SERVO_POSITION_AT_PERPENDICULAR;
//import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.SERVO_POSITION_AT_ZERO;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.team24751.subsystems.AngleServo;

@TeleOp(name = "Test Parallel", group = "Test")
public class TestParallel extends LinearOpMode {
    private DcMotor leftArmMotor = null;
    private DcMotor rightArmMotor = null;
    private DcMotor elevatorMotor = null;
    private AngleServo wristServo = null;

    private double getMotorAngleDeg() {
        double a = (MOTOR_POSITION_AT_PERPENDICULAR - MOTOR_POSITION_AT_ZERO) / 90.0;
        double b = MOTOR_POSITION_AT_ZERO;
        double currentPosition = leftArmMotor.getCurrentPosition();
        return a * currentPosition + b;
    }

    @Override
    public void runOpMode() {
        leftArmMotor = hardwareMap.get(DcMotor.class, "leftArmMotor");
        rightArmMotor = hardwareMap.get(DcMotor.class, "rightArmMotor");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");

        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wristServo = new AngleServo("wristServo", INITAL_WRIST_SERVO_ANGLE, REV_SERVO_ANGLE_RANGE, this);
        wristServo.init(REV_SERVO_PWM_RANGE);
        wristServo.getServo().setDirection(Servo.Direction.REVERSE);
        wristServo.getServo().scaleRange(0.1, 0.9);

        waitForStart();

        while (opModeIsActive()) {
            wristServo.setAngle(120.0 + getMotorAngleDeg());
            if (gamepad1.triangle) {
                leftArmMotor.setPower(0.3);
                rightArmMotor.setPower(0.3);
            } else if (gamepad1.cross) {
                leftArmMotor.setPower(-0.1);
                rightArmMotor.setPower(-0.1);
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
