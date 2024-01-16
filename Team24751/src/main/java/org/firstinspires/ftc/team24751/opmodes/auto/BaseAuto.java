package org.firstinspires.ftc.team24751.opmodes.auto;

import static org.firstinspires.ftc.team24751.Constants.AllianceColor.TEST;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;

import java.util.List;

public abstract class BaseAuto extends LinearOpMode {
    // Total run time
    private ElapsedTime runtime = new ElapsedTime();

    // Subsystem objects
    private Drivebase drivebase = null;

    /**
     * Extends this function and set the allianceColor to appropriate color
     * */
    protected abstract void setAllianceColor();
    protected void base_runOpMode() {
        // Update status
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Set alliance color
        setAllianceColor();

        // Enable bulk reads in auto mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Init subsystems
        drivebase = new Drivebase(this);

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
        PoseStorage.setPose(drivebase.getPoseEstimate());
    }
}
