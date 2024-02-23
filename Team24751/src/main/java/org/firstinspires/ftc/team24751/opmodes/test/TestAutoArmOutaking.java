package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.ARM_AUTO_INTAKING_ANGLE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.ARM_BACKDROP_PARALLEL_ANGLE_AUTO;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.CLOSE_CLAW_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.OPEN_CLAW_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.WRIST_AUTO_OUTAKING_DEG;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.WRIST_GROUND_PARALLEL_DEG;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.arm.Arm;
import org.firstinspires.ftc.team24751.subsystems.arm.Extender;
import org.firstinspires.ftc.team24751.subsystems.arm.Grabber;
import org.firstinspires.ftc.team24751.subsystems.arm.Wrist;
import org.firstinspires.ftc.team24751.subsystems.drivebase.Drivebase;

@Autonomous(name = "Test Auto Arm Outaking", group = "Test")
public class TestAutoArmOutaking extends LinearOpMode {
    Arm arm = new Arm(this);
    Extender extender = new Extender(this);
    Wrist wrist = new Wrist(this);
    Grabber grabber = new Grabber(this);
    ElapsedTime timeoutTimer = new ElapsedTime();
    ElapsedTime waitServoTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        arm.init();
        extender.init();
        wrist.init();
        grabber.init();
        waitForStart();
        arm.setTargetAngle(ARM_AUTO_INTAKING_ANGLE);
        Drivebase drive = new Drivebase(this);
        while (opModeIsActive()) {
            grabber.setPosition(CLOSE_CLAW_POSITION, CLOSE_CLAW_POSITION);
            arm.setTargetAngle(ARM_BACKDROP_PARALLEL_ANGLE_AUTO);
            wrist.setAngle(WRIST_GROUND_PARALLEL_DEG);
            timeoutTimer.reset();
            while (true) {
                arm.update();
                if (arm.autoOutakePIDLoop() || timeoutTimer.seconds() >= 10000) {
                    waitServoTimer.reset();
                    break;
                }
            }
            while (true)
            {
                arm.autoOutakePIDLoop();
                wrist.setAngle(WRIST_AUTO_OUTAKING_DEG);
                if (waitServoTimer.seconds() >= 1) {
                    break;
                } else if (waitServoTimer.seconds() >= 0.5) {
                    extender.setPower(0);
                    grabber.setPosition(OPEN_CLAW_POSITION, OPEN_CLAW_POSITION);
                } else if (extender.getPosition() < 50) {
                    extender.setPower(-0.5);
                } else {
                    extender.setPower(0);
                    break;
                }
            }
            /*
             * Must setTarget arm and reset timeoutTimer
             * */
            telemetry.addData("Current Arm Position (L)", arm.leftArmEncoder.getPosition());
            telemetry.addData("Current Arm Position (R)", arm.rightArmEncoder.getPosition());
            telemetry.addData("Current Arm Angle (L + R)", arm.getAngle());
            telemetry.update();
        }
        drive.drive(-0.5, 0, 0);
        sleep(1000);
        drive.drive(0, 0, 0);
    }
}
