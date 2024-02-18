package org.firstinspires.ftc.team24751.opmodes.test.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Disabled
@Autonomous(name = "TurnTest", group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        Drivebase drive = new Drivebase(this);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}
