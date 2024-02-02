package org.firstinspires.ftc.team24751.opmodes.auto;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.AutoTrajectoryManager;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;

import java.util.List;

public abstract class BaseAuto extends LinearOpMode {
    // Total run time
    private ElapsedTime runtime = new ElapsedTime();

    // Subsystem objects
    protected Drivebase drivebase = null;
    AutoTrajectoryManager autoTrajectoryManager;
    protected AutoTrajectoryManager.StartingPos startingPos;

    /**
     * Extends this function and set the allianceColor to appropriate color
     * */
    protected abstract void initStartingCondition();
    protected void base_runOpMode() {
        // Update status
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Set starting pos
        initStartingCondition();

        // Enable bulk reads in auto mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Init subsystems
        drivebase = new Drivebase(this);
        autoTrajectoryManager = new AutoTrajectoryManager(startingPos, drivebase, this);

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();
        runtime.reset();

        autoTrajectoryManager.followTrajectory();

        // Save the last Pose2d estimated in auto mode, for using in manual mode
        PoseStorage.setPose(drivebase.getPoseEstimate());
    }
    @Override
    public void runOpMode() {
        base_runOpMode();
    }
}
