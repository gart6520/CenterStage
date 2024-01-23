package org.firstinspires.ftc.team24751.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team24751.subsystems.Lift;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;

@TeleOp(name = "Test Lift", group = "Test")
public class TestLift extends LinearOpMode {
    Lift lift = new Lift(this);

    @Override
    public void runOpMode() throws InterruptedException {
        lift.init();
        Drivebase drive = new Drivebase(this);
        waitForStart();
        while (opModeIsActive()) {
            double left = 0;
            if (gamepad1.dpad_left) left = -1;
            if (gamepad1.dpad_right) left = 1;

            lift.setPower(left);
            drive.manualControl(true);
        }
    }
}
