package org.firstinspires.ftc.team24751.opmodes.auto;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.FRONT_CAMERA_NAME;
import static org.firstinspires.ftc.team24751.Constants.VISION.FRONT_CAMERA_RESOLUTION;
import static org.firstinspires.ftc.team24751.Utility.enableBulkRead;

import android.util.Size;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.Constants;
import org.firstinspires.ftc.team24751.subsystems.AutoTrajectoryManager;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.vision.Camera;
import org.firstinspires.ftc.team24751.subsystems.vision.TeamPropProcessor;

import java.util.List;

public abstract class BaseAuto extends LinearOpMode {
    // Total run time

    // Subsystem objects
    protected Drivebase drivebase = null;
    AutoTrajectoryManager autoTrajectoryManager;
    Camera frontCam = new Camera(FRONT_CAMERA_NAME, this);
    TeamPropProcessor teamPropProcessor = new TeamPropProcessor();
    protected AutoTrajectoryManager.StartingPos startingPos;
    ElapsedTime timer = new ElapsedTime();

    /**
     * Extends this function and set the allianceColor to appropriate color
     */
    protected abstract void initStartingCondition();

    protected void base_runOpMode() {
        // Update status
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        frontCam.addProcessorToQueue(teamPropProcessor);
        frontCam.buildCamera(FRONT_CAMERA_RESOLUTION);
        // Set starting pos
        initStartingCondition();

        // Enable bulk reads in auto mode
        enableBulkRead(hardwareMap);
        // Init subsystems
        drivebase = new Drivebase(this);
        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Give time for the Team Prop Processor to process the image
        sleep(3000);

        autoTrajectoryManager = new AutoTrajectoryManager(startingPos, teamPropProcessor.getPos(), drivebase, this);

        // Follow trajectory
        autoTrajectoryManager.followTrajectory();

        // Save the last Pose2d estimated in auto mode, for using in manual mode
        PoseStorage.setPose(drivebase.getPoseEstimate());
    }

    @Override
    public void runOpMode() {
        base_runOpMode();
    }
}
