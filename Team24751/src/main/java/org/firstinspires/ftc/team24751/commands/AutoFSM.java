package org.firstinspires.ftc.team24751.commands;

import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.DISTANCE_TO_GROUND_THRESHOLD;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.CLOSE_CLAW_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.OPEN_CLAW_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.WRIST_FULL_BACKWARD_DEG;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.WRIST_GROUND_PARALLEL_DEG;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.YellowPixelYeeter.LOAD_YELLOW_PIXEL_YEETER_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.YellowPixelYeeter.YEET_YELLOW_PIXEL_YEETER_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team24751.subsystems.YellowPixelYeeter;
import org.firstinspires.ftc.team24751.subsystems.arm.Arm;
import org.firstinspires.ftc.team24751.subsystems.arm.Extender;
import org.firstinspires.ftc.team24751.subsystems.arm.Grabber;
import org.firstinspires.ftc.team24751.subsystems.arm.Wrist;
import org.firstinspires.ftc.team24751.subsystems.sensor.Distance;

public class AutoFSM {
    public Extender extender;
    public Arm arm;
    public Wrist wrist;
    public Grabber grabber;
    public YellowPixelYeeter yellowPixelYeeter;
    public Distance distance;
    LinearOpMode opMode;
    public ElapsedTime waitServoTimer = new ElapsedTime();
    public ElapsedTime timeoutTimer = new ElapsedTime();

    public void dropArmAndReset() {
        ElapsedTime timer = new ElapsedTime();

        wrist.setAngle(WRIST_GROUND_PARALLEL_DEG);

        timer.reset();
        opMode.telemetry.addLine("Resetting arm and extender");
        opMode.telemetry.update();

        while (timer.seconds() <= 2 && distance.getDistanceCM() > DISTANCE_TO_GROUND_THRESHOLD) {
            if (timer.seconds() >= 1) {
                arm.setPower(-0.05);
            }
        }


        arm.resetEncoder();
        arm.setPower(0);
        extender.resetPosition();

        // Set initial state to intaking
        // After drop arm and reset, the arm now should be at intake position
        grabber.setPosition(CLOSE_CLAW_POSITION, CLOSE_CLAW_POSITION);
        yellowPixelYeeter.setPosition(LOAD_YELLOW_PIXEL_YEETER_POSITION);
        wrist.setAngle(WRIST_GROUND_PARALLEL_DEG);
    }

    public enum ArmState {
        none, roadrunner, prepare_intaking, yellow_pixel, purple_pixel, outaking,
        arm_moving_up, arm_moving_down, intaking, after_intake
    }

    public ArmState state;

    public AutoFSM(LinearOpMode _opMode) {
        opMode = _opMode;
        extender = new Extender(opMode);
        arm = new Arm(opMode);
        wrist = new Wrist(opMode);
        grabber = new Grabber(opMode);
        yellowPixelYeeter = new YellowPixelYeeter(opMode);
        distance = new Distance(opMode);
    }

    public void init() {
        extender.init();
        arm.init();
        wrist.init();
        grabber.init();
        yellowPixelYeeter.init();
        distance.init();

        grabber.setPosition(CLOSE_CLAW_POSITION, CLOSE_CLAW_POSITION);
    }

    public void update() {
        arm.update();

        switch (state) {
            case none: // Impossible state
            case roadrunner:
                break;
            /*
             * Must reset timeoutTimer
             * */
            case purple_pixel:
                wrist.setAngle(WRIST_GROUND_PARALLEL_DEG);
                if (distance.getDistanceCM() <= DISTANCE_TO_GROUND_THRESHOLD || timeoutTimer.seconds() >= 2) {
                    grabber.setPosition(OPEN_CLAW_POSITION, OPEN_CLAW_POSITION);
                    wrist.setAngle(WRIST_FULL_BACKWARD_DEG);
                    state = ArmState.roadrunner;
                }
                break;
            /*
             * Must reset waitServoTimer
             * */
            case yellow_pixel:
                wrist.setAngle(WRIST_FULL_BACKWARD_DEG);
                yellowPixelYeeter.setPosition(YEET_YELLOW_PIXEL_YEETER_POSITION);
                if (waitServoTimer.seconds() >= 0.8) {
                    yellowPixelYeeter.setPosition(LOAD_YELLOW_PIXEL_YEETER_POSITION);
                }
                if (waitServoTimer.seconds() >= 1.5) {
                    state = ArmState.roadrunner;
                }
                break;
            case prepare_intaking:
                wrist.setAngle(WRIST_GROUND_PARALLEL_DEG);
                grabber.setPosition(OPEN_CLAW_POSITION, OPEN_CLAW_POSITION);
                state = ArmState.roadrunner;
                break;
            /*
             * Must reset waitServoTimer
             * */
            case intaking:
                wrist.setAngle(WRIST_GROUND_PARALLEL_DEG);
                grabber.setPosition(CLOSE_CLAW_POSITION, CLOSE_CLAW_POSITION);
                if (waitServoTimer.seconds() >= 1) {
                    state = ArmState.roadrunner;
                }
                break;
            case after_intake:
                grabber.setPosition(CLOSE_CLAW_POSITION, CLOSE_CLAW_POSITION);
                wrist.setAngle(WRIST_FULL_BACKWARD_DEG);
                state = ArmState.roadrunner;
                break;
            /*
             * Must reset waitServoTimer
             * */
            case outaking:
                wrist.autoParallel(arm.getAngle());
                if (waitServoTimer.seconds() >= 1) {
                    timeoutTimer.reset();
                    state = ArmState.arm_moving_down;
                } else if (waitServoTimer.seconds() >= 0.5) {
                    grabber.setPosition(OPEN_CLAW_POSITION, OPEN_CLAW_POSITION);
                }
                break;
            /*
             * Must setTarget arm and reset timeoutTimer
             * */
            case arm_moving_up:
                wrist.setAngle(WRIST_FULL_BACKWARD_DEG);
                if (arm.outakePIDLoop() || timeoutTimer.seconds() >= 1.75) {
                    waitServoTimer.reset();
                    state = ArmState.outaking;
                }
                break;
            /*
             * Must reset timeoutTimer
             * */
            case arm_moving_down:
                // Get current arm angle
                double angle = arm.getAngle();

                // If arm angle is < 20
                if (angle < 20) {
                    arm.setPower(-0.05);
                }

                // If arm angle is < 90
                else if (angle < 90) {
                    // If just enter 90 degree state

                    // Move wrist up to allow base moving
                    wrist.setAngle(WRIST_GROUND_PARALLEL_DEG);

                    // Continue to move arm down
                    arm.setPower(-0.1);

                }
                // If angle > 90 -> move arm down
                else {
                    arm.setPower(-0.4);
                }

                // If distance sensor reported touching ground or if arm is timeout
                if (timeoutTimer.seconds() > 3 || distance.getDistanceCM() <= DISTANCE_TO_GROUND_THRESHOLD) {
                    // Stop arm
                    arm.setPower(0);

                    // Reset arm encoder
                    arm.resetEncoder();

                    // Move wrist up
                    wrist.setAngle(WRIST_FULL_BACKWARD_DEG);

                    // Switch to roadrunner traj
                    state = ArmState.roadrunner;
                }
                break;
        }
    }
}
