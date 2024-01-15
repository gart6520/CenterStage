package org.firstinspires.ftc.team24751.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test 2 motor Ele", group = "Test")
public class TestTwoMotorEle extends LinearOpMode {
    DcMotorEx motor1;
    DcMotorEx motor2;

    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive())
        {
            if (gamepad1.dpad_up) {
                motor1.setPower(1);
                motor2.setPower(1);
            } else if (gamepad1.dpad_down) {
                motor1.setPower(-1);
                motor2.setPower(-1);
            } else {
                motor1.setPower(0);
                motor2.setPower(0);
            }
        }
    }
}
