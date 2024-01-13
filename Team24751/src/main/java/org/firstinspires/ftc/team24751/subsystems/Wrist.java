package org.firstinspires.ftc.team24751.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Wrist {
    private ServoImplEx wrist;
    private LinearOpMode linearOpMode;
    private WristState wristState;
    private double overallTargetPosition = 0.0;
    private double targetPosition = 0.0;

    public enum WristState {
        SCORING,
        FLAT,
        STORED
    }

    public Wrist(ServoImplEx _wrist, LinearOpMode _linearOpMode) {
        wrist = _wrist;
        linearOpMode = _linearOpMode;
        wristState = WristState.STORED;
    }

    private static double clamp(double num, double min, double max) {
        return Math.max(min, Math.min(num, max));
    }

    private static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    private void setTargetPosition(double _targetPosition) {
        targetPosition = _targetPosition;
        overallTargetPosition = _targetPosition;
        wrist.setPosition(targetPosition);
    }

    public void setWristPosition(WristState _wristState) {
        double pos = overallTargetPosition;
        //double pos = wrist.getPosition();
        if (_wristState == WristState.SCORING) {
            double targetAngle = ((pos) - ((((pos < Math.PI / 2) ? 5 : 13) * Math.PI) / 18));
            setTargetPosition(clamp(map(targetAngle, 0, Math.PI / 2 - 0.35, 0.5, 0.93), 0.03, 0.97));
        } else if (_wristState == WristState.FLAT) {
            double targetAngle = ((pos) - ((((pos < Math.PI / 2) ? 0 : 1) * Math.PI)));
            setTargetPosition(clamp(map(targetAngle, 0, Math.PI / 2 - 0.35, 0.5, 0.93) + 0.03, 0.03, 0.97));
        } else if (_wristState == WristState.STORED) {
            setTargetPosition(0.03);
        }
    }
}
