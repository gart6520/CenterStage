package org.firstinspires.ftc.team24751.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;

@Autonomous(name = "Drive Back", group = "!")
public class DriveBack extends LinearOpMode {
    ElapsedTime timing = new ElapsedTime();
    Drivebase drivebase = null;

    @Override
    public void runOpMode() throws InterruptedException {
        timing.reset();
        drivebase = new Drivebase(hardwareMap);

        waitForStart();
        timing.reset();
        drivebase.drive(0, -0.5, 0);
        while (opModeIsActive()) {
            if (timing.seconds() > 4) {
                drivebase.drive(0, 0, 0);
            }
        }
    }
}

