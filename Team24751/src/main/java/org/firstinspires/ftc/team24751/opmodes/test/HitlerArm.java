package org.firstinspires.ftc.team24751.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "HitlerArm", group = "Test")
public class HitlerArm extends LinearOpMode {
    private DcMotor motor0 = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;

    @Override
    public void runOpMode() {
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.triangle) {
                motor0.setPower(0.8);
                motor1.setPower(0.8);
            } else if (gamepad1.cross) {
                motor0.setPower(-0.5);
                motor1.setPower(-0.5);
            } else {
                motor0.setPower(0);
                motor1.setPower(0);
            }

            if (gamepad1.circle) {
                motor2.setPower(0.8);
            } else if (gamepad1.square) {
                motor2.setPower(-0.8);
            } else {
                motor2.setPower(0);
            }
            telemetry.addData("Current Position", motor0.getCurrentPosition());
            telemetry.update();
        }
    }
}
