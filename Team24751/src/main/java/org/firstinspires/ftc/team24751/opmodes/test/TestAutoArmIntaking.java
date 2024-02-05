package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.ARM_AUTO_INTAKING_ANGLE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.ARM_BACKDROP_PARALLEL_ANGLE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.CLOSE_CLAW_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.OPEN_CLAW_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.WRIST_AUTO_INTAKING_DEG;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.WRIST_FULL_BACKWARD_DEG;

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
    ElapsedTime clawCloseTimer = new ElapsedTime();
    public enum ArmState
    {
        none, intaking, arm_moving_up, outaking, base_moving, to_intaking
    }
    ArmState state = ArmState.none;

    @Override
    public void runOpMode() throws InterruptedException {
        arm.init();
        wrist.init();
        grabber.init();
        waitForStart();
        arm.setTargetAngle(ARM_AUTO_INTAKING_ANGLE);
        arm.update();
        state = ArmState.to_intaking;
        while (opModeIsActive()) {
            //TODO Turn this into FSM
            switch (state)
            {
                case none:
                    //Impossible state
                    break;
                case to_intaking:
                    wrist.setAngle(WRIST_AUTO_INTAKING_DEG);
                    if (arm.autoIntakePIDLoop())
                    {
                        clawCloseTimer.reset();
                        state = ArmState.intaking;
                    }
                    break;
                case intaking:
                    grabber.setPosition(CLOSE_CLAW_POSITION, CLOSE_CLAW_POSITION);
                    if (clawCloseTimer.seconds() > 0.5)
                    {
                        arm.setTargetAngle(ARM_BACKDROP_PARALLEL_ANGLE);
                        state = ArmState.arm_moving_up;
                    }
                    break;
                case arm_moving_up:
                    wrist.setAngle(WRIST_FULL_BACKWARD_DEG);
                    if (arm.outakePIDLoop())
                    {
                        state = ArmState.outaking;
                    }
                    break;
                case outaking:
                    grabber.setPosition(OPEN_CLAW_POSITION, OPEN_CLAW_POSITION);
                    break;
                case base_moving:
                    wrist.setAngle(WRIST_FULL_BACKWARD_DEG);
                    break;
            }
            telemetry.addData("Current Arm Position (L)", arm.leftArmEncoder.getPosition());
            telemetry.addData("Current Arm Position (R)", arm.rightArmEncoder.getPosition());
            telemetry.addData("Current Arm Angle (L + R)", arm.getAngle());
            telemetry.addData("Arm State", state.toString());
            telemetry.update();
        }
    }
}
