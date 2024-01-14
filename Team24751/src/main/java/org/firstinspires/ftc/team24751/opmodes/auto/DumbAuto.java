package org.firstinspires.ftc.team24751.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.Gyro;

public abstract class DumbAuto extends LinearOpMode {

    protected ElapsedTime timing = new ElapsedTime();
    Drivebase drivebase = new Drivebase();
    Gyro gyro = new Gyro();

    //right = true, left = false
    protected void baseRun(double sec, boolean right) {
        timing.reset();
        gyro.init(this);
        drivebase.init(this, gyro);

        waitForStart();
        gyro.reset();
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
