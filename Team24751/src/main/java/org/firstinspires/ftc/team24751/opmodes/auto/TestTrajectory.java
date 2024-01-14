package org.firstinspires.ftc.team24751.opmodes.auto;

import static org.firstinspires.ftc.team24751.Constants.OdometryPod.inPerTick;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_X;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Y;
import static org.firstinspires.ftc.team24751.Constants.SPEED.DRIVEBASE_SPEED_Z;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.Drivebase;
import org.firstinspires.ftc.team24751.subsystems.Gyro;
import org.firstinspires.ftc.team24751.subsystems.OdometryPod;
import org.firstinspires.ftc.team24751.subsystems.PoseStorage;
import org.firstinspires.ftc.team24751.subsystems.arm.Arm;
import org.firstinspires.ftc.team24751.subsystems.arm.Elevator;
import org.firstinspires.ftc.team24751.subsystems.arm.Grabber;
import org.firstinspires.ftc.team24751.subsystems.arm.Wrist;

import java.util.List;

@TeleOp(name = "Test Trajectory", group = "Test")
public class TestTrajectory extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    Gyro gyro = new Gyro();
    OdometryPod localizer = new OdometryPod();
    Drivebase drivebase = new Drivebase();
    Arm arm = new Arm(this);
    Wrist wrist = new Wrist(this);
    Grabber grabber = new Grabber(this);
    Elevator elevator = new Elevator(this);

    // Gamepad
    Gamepad prev = null;
    Gamepad curr = null;
    boolean grablt = false;
    boolean grabrt = false;

    @Override
    public void runOpMode() {
        // Update status
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        //Init all subsystems
        arm.init();
        arm.resetEncoder();

        wrist.init();
        grabber.init();
        elevator.init();

        // Enable bulk reads in auto mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        prev = new Gamepad();
        prev.copy(gamepad2);
        curr = new Gamepad();

        // Init gyro
        gyro.init(this);

        // Init localizer
        localizer.init(this, inPerTick, gyro);

        // Init drivebase
        drivebase.init(this, gyro);

        // Set pose
        drivebase.setCurrentPose(new Pose2d(new Vector2d(34.4, 58.5), Rotation2d.fromDouble(0)));

        // Update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();

        // Reset runtime
        runtime.reset();

        // Start auto
        Actions.runBlocking(
                drivebase.actionBuilder(drivebase.pose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(60, 0), Math.PI)
                        .build());
    }
}
