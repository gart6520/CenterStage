package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.ARM_AUTO_INTAKING_ANGLE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.ARM_BACKDROP_PARALLEL_ANGLE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.AUTO_INTAKING_DEG;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.CLOSE_CLAW_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.FULL_BACKWARD_DEG;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.arm.Arm;
import org.firstinspires.ftc.team24751.subsystems.arm.Grabber;
import org.firstinspires.ftc.team24751.subsystems.arm.Wrist;

@Autonomous(name = "Test Auto Arm Intaking", group = "Test")
public class TestAutoArmIntaking extends LinearOpMode {
    Arm arm = new Arm(this);
    Wrist wrist = new Wrist(this);
    Grabber grabber = new Grabber(this);
    ElapsedTime timer = new ElapsedTime();
    boolean gotPixels = false;
    public enum ArmState
    {
        intaking, arm_moving_up, outaking, base_moving
    }

    @Override
    public void runOpMode() throws InterruptedException {
        arm.init();
        wrist.init();
        grabber.init();
        waitForStart();
        arm.setTargetAngle(ARM_AUTO_INTAKING_ANGLE);
        while (opModeIsActive()) {
            //TODO Turn this into FSM
            wrist.setAngle(AUTO_INTAKING_DEG);
            if (timer.seconds() >= 1 && gotPixels)
            {
                wrist.setAngle(FULL_BACKWARD_DEG);
                if (arm.outakePIDLoop())
                {

                }
                continue;
            }
            if (arm.autoIntakePIDLoop()) {
                if (!gotPixels)
                    timer.reset();
                gotPixels = true;
                arm.setTargetAngle(ARM_BACKDROP_PARALLEL_ANGLE);
                grabber.setPosition(CLOSE_CLAW_POSITION, CLOSE_CLAW_POSITION);
            }
        }
    }
}
