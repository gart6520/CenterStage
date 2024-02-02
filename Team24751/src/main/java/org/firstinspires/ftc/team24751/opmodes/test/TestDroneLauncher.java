package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.DroneLauncher.LOAD_DRONE_LAUNCHER_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.DroneLauncher.SHOOT_DRONE_LAUNCHER_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.team24751.subsystems.DroneLauncher;

@TeleOp(name = "Test Drone Launcher", group = "Test")
public class TestDroneLauncher extends LinearOpMode {
    DroneLauncher droneLauncher = new DroneLauncher(this);
    boolean load = true;
    Gamepad curr, prev;

    @Override
    public void runOpMode() throws InterruptedException {
        droneLauncher.init();
        waitForStart();
        curr = new Gamepad();
        curr.copy(gamepad1);
        prev = new Gamepad();

        // Loop, run until driver presses STOP
        while (opModeIsActive()) {
            curr.copy(gamepad1);
            if (curr.circle && !prev.circle) {
                if (load) {
                    load = false;
                    droneLauncher.setPosition(SHOOT_DRONE_LAUNCHER_POSITION);
                } else {
                    load = true;
                    droneLauncher.setPosition(LOAD_DRONE_LAUNCHER_POSITION);
                }
            }
            telemetry.addData("Drone launcher state", load? "Load" : "Shot");
            telemetry.update();
            prev.copy(curr);
        }
    }
}
