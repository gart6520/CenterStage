package org.firstinspires.ftc.team24751.opmodes.auto;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.Gyro;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;

import java.util.List;

public class BaseAuto extends LinearOpMode {
    // Total run time
    private ElapsedTime runtime = new ElapsedTime();

    // Subsystem objects
    private Gyro gyro = new Gyro(this);
    private Drivebase drivebase = new Drivebase(this, gyro);

    public void base_runOpMode() {
        // Update status
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Enable bulk reads in auto mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Init subsystems
        gyro.init();
        drivebase.init();

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();
        runtime.reset();

        // TODO: add real auto actions here

        /*Actions.runBlocking(drivebase.actionBuilder(PoseStorage.getPose())
                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                .splineTo(new Vector2d(60, 0), Math.PI)
                .build());*/

        // Save the last Pose2d estimated in auto mode, for using in manual mode
        PoseStorage.setPose(drivebase.pose);
    }

    @Override
    public void runOpMode() {
        // Empty
        // This function should be overrode by the function in the class that further extends this class
    }
}
