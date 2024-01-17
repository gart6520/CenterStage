package org.firstinspires.ftc.team24751.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team24751.subsystems.Lift;

@TeleOp(name = "Test Lift", group = "Test")
public class TestLift extends LinearOpMode {
    Lift lift = new Lift(this);

    @Override
    public void runOpMode() throws InterruptedException {
        lift.init();
        waitForStart();
        while (opModeIsActive()) {
            double left = 0, right = 0;
            if (gamepad1.dpad_left) left = -0.6;
            if (gamepad1.dpad_right) left = 0.6;
            if (gamepad1.dpad_down) right = -0.6;
            if (gamepad1.dpad_up) right = 0.6;
            lift.setPower(left, right);
        }
    }
}
