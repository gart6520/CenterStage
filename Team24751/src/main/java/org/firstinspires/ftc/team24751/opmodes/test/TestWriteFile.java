package org.firstinspires.ftc.team24751.opmodes.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team24751.subsystems.PoseStorage;

@TeleOp(name = "Test Write File", group = "Test")
public class TestWriteFile extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PoseStorage.init();
        PoseStorage.setPose(new Pose2d(2, -3.14, 3.14));
        telemetry.addData("Pose", PoseStorage.getPose());
        telemetry.update();
        waitForStart();
    }
}
