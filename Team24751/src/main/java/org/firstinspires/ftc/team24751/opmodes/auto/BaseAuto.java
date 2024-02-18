package org.firstinspires.ftc.team24751.opmodes.auto;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.FRONT_CAMERA_NAME;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.CLOSE_CLAW_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.YELLOW_PIXEL_YEETER.LOAD_YELLOW_PIXEL_YEETER_POSITION;
import static org.firstinspires.ftc.team24751.Constants.VISION.FRONT_CAMERA_RESOLUTION;
import static org.firstinspires.ftc.team24751.Utility.enableBulkRead;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.Constants;
import org.firstinspires.ftc.team24751.commands.AutoArmFSM;
import org.firstinspires.ftc.team24751.subsystems.AutoTrajectoryManager;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.arm.Arm;
import org.firstinspires.ftc.team24751.subsystems.arm.Extender;
import org.firstinspires.ftc.team24751.subsystems.arm.Grabber;
import org.firstinspires.ftc.team24751.subsystems.arm.Wrist;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.vision.Camera;
import org.firstinspires.ftc.team24751.subsystems.vision.TeamPropProcessor;

public abstract class BaseAuto extends LinearOpMode {
    // Total run time

    // Subsystem objects
    protected Drivebase drivebase = null;
    AutoTrajectoryManager autoTrajectoryManager;
    Camera frontCam = new Camera(FRONT_CAMERA_NAME, this);
    TeamPropProcessor teamPropProcessor = new TeamPropProcessor();
    protected AutoTrajectoryManager.StartingPos startingPos;
    ElapsedTime timer = new ElapsedTime();
    protected AutoArmFSM autoArmFSM = new AutoArmFSM(this);

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
        autoArmFSM.init();

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        autoArmFSM.grabber.setPosition(CLOSE_CLAW_POSITION, CLOSE_CLAW_POSITION);
        autoArmFSM.yellowPixelYeeter.setPosition(LOAD_YELLOW_PIXEL_YEETER_POSITION);
        // Give time for the Team Prop Processor to process the image
        sleep(3000);

//        autoTrajectoryManager = new AutoTrajectoryManager(startingPos, teamPropProcessor.getPos(), drivebase, autoArmFSM, this);
        autoTrajectoryManager = new AutoTrajectoryManager(startingPos, Constants.VISION.CV.TeamPropPosition.LEFT, drivebase, autoArmFSM, this);

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
