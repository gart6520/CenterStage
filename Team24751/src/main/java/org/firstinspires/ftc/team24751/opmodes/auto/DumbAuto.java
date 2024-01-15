package org.firstinspires.ftc.team24751.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;

public abstract class DumbAuto extends LinearOpMode {

    protected ElapsedTime timing = new ElapsedTime();
    Drivebase drivebase = null;

    //right = true, left = false
    protected void baseRun(double sec, boolean right) {
        timing.reset();
        drivebase = new Drivebase(hardwareMap);

        waitForStart();
        timing.reset();
        int sign = right ? 1 : -1;
        drivebase.drive(sign * 0.5, 0, 0);
        while (opModeIsActive()) {
            if (timing.seconds() > sec)
            {
                drivebase.drive(0,0,0);
            }
        }
    }
}
