package org.firstinspires.ftc.team24751.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.jetbrains.annotations.NotNull;

public class Claw {
    private ServoImplEx leftClaw;
    private ServoImplEx rightClaw;
    private LinearOpMode opMode;

    public ClawState leftClawState;
    public ClawState rightClawState;


    public enum ClawState {
        CLOSED,
        INTERMEDIATE,
        OPEN
    }

    public enum ClawSide {
        LEFT,
        RIGHT,
        BOTH
    }

    public Claw(ServoImplEx _leftClaw, ServoImplEx _rightClaw, LinearOpMode _opMode) {
        leftClaw = _leftClaw;
        rightClaw = _rightClaw;
        opMode = _opMode;
        updateClawState(ClawState.CLOSED, ClawSide.BOTH);
    }
    private double getClawStatePosition(ClawState state, ClawSide side) {
        switch (side) {
            case LEFT:
                switch (state) {
                    case CLOSED:
                        return 0.07;
                    case INTERMEDIATE:
                        return 0.15;
                    case OPEN:
                        return 0.38;
                    default:
                        return 0.0;
                }
            case RIGHT:
                switch (state) {
                    case CLOSED:
                        return 0.52;
                    case INTERMEDIATE:
                        return 0.58;
                    case OPEN:
                        return 0.89;
                    default:
                        return 0.0;
                }
            default:
                return 0.0;
        }
    }

    public ClawState getClawState(ClawSide clawSide) {
        if (clawSide == ClawSide.BOTH)
            return (rightClawState == ClawState.CLOSED || leftClawState == ClawState.CLOSED) ? ClawState.CLOSED : ClawState.OPEN;
        return (clawSide == ClawSide.LEFT) ? leftClawState : rightClawState;
    }

    public void updateClawState(@NotNull ClawState clawState, @NotNull ClawSide clawSide) {
        double position = getClawStatePosition(clawState, clawSide);
        switch (clawSide) {
            case LEFT:
                leftClaw.setPosition(position);
                leftClawState = clawState;
                break;
            case RIGHT:
                rightClaw.setPosition(position);
                rightClawState = clawState;
                break;
            case BOTH:
                leftClaw.setPosition(position);
                leftClawState = clawState;
                rightClaw.setPosition(position);
                rightClawState = clawState;
                break;
        }
    }
}
