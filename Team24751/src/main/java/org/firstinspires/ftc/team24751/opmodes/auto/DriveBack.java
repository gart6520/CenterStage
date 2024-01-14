package org.firstinspires.ftc.team24751.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.Gyro;

@Autonomous(name = "Drive Back", group = "!")
public class DriveBack extends LinearOpMode {
    ElapsedTime timing = new ElapsedTime();
    Drivebase drivebase = new Drivebase();
    Gyro gyro = new Gyro();

    @Override
    public void runOpMode() throws InterruptedException {
        timing.reset();
        gyro.init(this);
        drivebase.init(this, gyro);

        waitForStart();
        gyro.reset();
        timing.reset();
        drivebase.drive(0, -0.5, 0);
        while (opModeIsActive()) {
            if (timing.seconds() > 4) {
                drivebase.drive(0, 0, 0);
            }
        }
    }
}

