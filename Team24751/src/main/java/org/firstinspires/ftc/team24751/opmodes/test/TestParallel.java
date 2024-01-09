package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.MOTOR_POSITION_AT_PERPENDICULAR;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.MOTOR_POSITION_AT_ZERO;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.REV_SERVO_PWM_RANGE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.SERVO_POSITION_AT_HORIZONTAL_BACK;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.SERVO_POSITION_AT_ZERO;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "Test Parallel", group = "Test")
public class TestParallel extends LinearOpMode {
    private DcMotor leftArmMotor = null;
    private DcMotor rightArmMotor = null;
    private DcMotor elevatorMotor = null;
    private ServoImplEx wristServo = null;

    private double getMotorAngleDeg() {
        return 90.0 / (MOTOR_POSITION_AT_PERPENDICULAR - MOTOR_POSITION_AT_ZERO) * leftArmMotor.getCurrentPosition() - 90.0 / (MOTOR_POSITION_AT_PERPENDICULAR - MOTOR_POSITION_AT_ZERO) * MOTOR_POSITION_AT_ZERO;
    }

    private void setWristAngleDeg(double deg) {
        wristServo.setPosition(deg * (180.0 / SERVO_POSITION_AT_HORIZONTAL_BACK) / 180.0);
    }

    @Override
    public void runOpMode() {
        leftArmMotor = hardwareMap.get(DcMotor.class, "leftArmMotor");
        rightArmMotor = hardwareMap.get(DcMotor.class, "rightArmMotor");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        wristServo = (ServoImplEx) hardwareMap.get(ServoImplEx.class, "wristServo");

        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wristServo.setPwmRange(REV_SERVO_PWM_RANGE);

        waitForStart();

        while (opModeIsActive()) {
            if (getMotorAngleDeg() >= 45){
                //setWristAngleDeg(60.0 + getMotorAngleDeg());
                //setWristAngleDeg(300.0 - getMotorAngleDeg());
            }
            if (gamepad1.triangle) {
                leftArmMotor.setPower(0.5);
                rightArmMotor.setPower(0.5);
            } else if (gamepad1.cross) {
                leftArmMotor.setPower(-0.5);
                rightArmMotor.setPower(-0.5);
            } else {
                leftArmMotor.setPower(0);
                rightArmMotor.setPower(0);
            }

            if (gamepad1.circle) {
                elevatorMotor.setPower(0.6);
            } else if (gamepad1.square) {
                elevatorMotor.setPower(-0.6);
            } else {
                elevatorMotor.setPower(0);
            }
            telemetry.addData("Current Motor Degree", getMotorAngleDeg());
            //telemetry.addData("Current Servo Degree", wristServo.getAngle());
            telemetry.update();
        }
    }
}
