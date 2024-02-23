package org.firstinspires.ftc.team24751.commands;

import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Arm.DISTANCE_TO_GROUND_THRESHOLD;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Extender.EXTENDER_FULLY_IN_THRESHOLD;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.CLOSE_CLAW_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.OPEN_CLAW_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.WRIST_AUTO_INTAKING_DEG;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.Hand.WRIST_AUTO_OUTAKING_DEG;
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
import org.firstinspires.ftc.team24751.subsystems.drivebase.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team24751.subsystems.sensor.Distance;

import java.util.function.Supplier;

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
        extender.resetEncoder();

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

    boolean hasPlacedPurplePixel = false;
    public ArmState state;
    public TrajectorySequence result;
    public Supplier<TrajectorySequence> borrowThread;
    public boolean hasBorrowThread;

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
             * Must reset timeoutTimer and hasBorrowThread
             * */
            case purple_pixel:
                if (!hasPlacedPurplePixel) {
                    wrist.setAngle(WRIST_GROUND_PARALLEL_DEG);
                }
                if ((distance.getDistanceCM() <= DISTANCE_TO_GROUND_THRESHOLD || timeoutTimer.seconds() >= 2) && !hasPlacedPurplePixel) {
                    grabber.setPosition(OPEN_CLAW_POSITION, OPEN_CLAW_POSITION);
                    wrist.setAngle(WRIST_FULL_BACKWARD_DEG);
                    waitServoTimer.reset();
                    hasPlacedPurplePixel = true;
                }
                if (!hasBorrowThread) {
                    result = borrowThread.get();
                    hasBorrowThread = true;
                }
                if (waitServoTimer.seconds() >= 0.75 && hasPlacedPurplePixel) {
                    state = ArmState.roadrunner;
                }

                break;
            /*
             * Must reset waitServoTimer and hasBorrowThread
             * */
            case yellow_pixel:
                wrist.setAngle(WRIST_FULL_BACKWARD_DEG);
                yellowPixelYeeter.setPosition(YEET_YELLOW_PIXEL_YEETER_POSITION);
                if (waitServoTimer.seconds() >= 0.7) {
                    yellowPixelYeeter.setPosition(LOAD_YELLOW_PIXEL_YEETER_POSITION);
                    if (!hasBorrowThread) {
                        result = borrowThread.get();
                        hasBorrowThread = true;
                    }
                }
                if (waitServoTimer.seconds() >= 1.2) {
                    state = ArmState.roadrunner;
                }
                break;
            /*
             * Must reset waitServoTimer
             * */
            case prepare_intaking:
                wrist.setAngle(WRIST_AUTO_INTAKING_DEG);
                grabber.setPosition(OPEN_CLAW_POSITION, OPEN_CLAW_POSITION);
                if (waitServoTimer.seconds() > 0.75) {
                    state = ArmState.roadrunner;
                }
                break;
            /*
             * Must reset waitServoTimer and hasBorrowThread
             * */
            case intaking:
                wrist.setAngle(WRIST_AUTO_INTAKING_DEG);
                grabber.setPosition(CLOSE_CLAW_POSITION, CLOSE_CLAW_POSITION);
                if (!hasBorrowThread) {
                    result = borrowThread.get();
                    hasBorrowThread = true;
                }
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
                wrist.setAngle(WRIST_AUTO_OUTAKING_DEG);
                if (waitServoTimer.seconds() >= 1) {
                    state = ArmState.roadrunner;
                } else if (waitServoTimer.seconds() >= 0.5) {
                    extender.setPower(0);
                    grabber.setPosition(OPEN_CLAW_POSITION, OPEN_CLAW_POSITION);
                } else if (extender.getPosition() < 680) {
                    extender.setPower(-1);
                } else {
                    extender.setPower(0);
                }
                break;
            /*
             * Must setTarget arm and reset timeoutTimer
             * */
            case arm_moving_up:
                wrist.setAngle(WRIST_FULL_BACKWARD_DEG);
                if (arm.outakePIDLoop() || timeoutTimer.seconds() >= 1.75) {
                    state = ArmState.roadrunner;
                }
                break;
            /*
             * Must reset timeoutTimer
             * */
            case arm_moving_down:

                if (timeoutTimer.seconds() < 2 && extender.getPosition() > EXTENDER_FULLY_IN_THRESHOLD) {
                    extender.setPower(1);
                } else {
                    extender.setPower(0);
                    arm.setPower(-0.1);
                }

                // If distance sensor reported touching ground or if arm is timeout
                if (timeoutTimer.seconds() > 4 || distance.getDistanceCM() <= DISTANCE_TO_GROUND_THRESHOLD) {
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
