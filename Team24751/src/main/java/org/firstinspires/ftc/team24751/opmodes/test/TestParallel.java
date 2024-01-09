package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.MOTOR_POSITION_AT_FRONT_HORIZONTAL;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.MOTOR_POSITION_AT_UPWARD_VERTICAL;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.REV_SERVO_PWM_RANGE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.team24751.subsystems.Claw;
import org.firstinspires.ftc.team24751.subsystems.Wrist;

@TeleOp(name = "Test Parallel", group = "Test")
public class TestParallel extends LinearOpMode {
    private DcMotor leftArmMotor = null;
    private DcMotor rightArmMotor = null;
    private DcMotor elevatorMotor = null;
    private ServoImplEx wristServo = null;
    private ServoImplEx leftClawServo = null;
    private ServoImplEx rightClawServo = null;
    private Wrist wrist;
    private Claw claw;

    private double getMotorAngleDeg() {
        return 90.0 / (MOTOR_POSITION_AT_UPWARD_VERTICAL - MOTOR_POSITION_AT_FRONT_HORIZONTAL) * leftArmMotor.getCurrentPosition() - 90.0 / (MOTOR_POSITION_AT_UPWARD_VERTICAL - MOTOR_POSITION_AT_FRONT_HORIZONTAL) * MOTOR_POSITION_AT_FRONT_HORIZONTAL;
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

        wristServo = (ServoImplEx) hardwareMap.get(ServoImplEx.class, "wristServo");
        wristServo.setPwmRange(REV_SERVO_PWM_RANGE);
        wrist = new Wrist(wristServo, this);

        leftClawServo = (ServoImplEx) hardwareMap.get(ServoImplEx.class, "leftClawServo");
        rightClawServo = (ServoImplEx) hardwareMap.get(ServoImplEx.class, "rightClawServo");
        claw = new Claw(leftClawServo, rightClawServo, this);

        waitForStart();

        while (opModeIsActive()) {
            // Update Claw Position
            if (getMotorAngleDeg() <= 30) {
                wrist.setWristPosition(Wrist.WristState.STORED);
            } else if (getMotorAngleDeg() >= 90) {
                wrist.setWristPosition(Wrist.WristState.SCORING);
            } else {
                wrist.setWristPosition(Wrist.WristState.FLAT);
            }

            // Control Arm
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

            // Control Claw
            if (gamepad1.dpad_left) {
                if (!gamepad1.left_bumper) {
                    claw.updateClawState(Claw.ClawState.OPEN, Claw.ClawSide.LEFT);
                } else {
                    claw.updateClawState(Claw.ClawState.CLOSED, Claw.ClawSide.LEFT);
                }
            } else if (gamepad1.dpad_right) {
                if (!gamepad1.left_bumper) {
                    claw.updateClawState(Claw.ClawState.OPEN, Claw.ClawSide.RIGHT);
                } else {
                    claw.updateClawState(Claw.ClawState.CLOSED, Claw.ClawSide.RIGHT);
                }
            }

            telemetry.addData("Current Motor Degree", getMotorAngleDeg());
            //telemetry.addData("Current Servo Degree", wristServo.getAngle());
            telemetry.update();
        }
    }
}
