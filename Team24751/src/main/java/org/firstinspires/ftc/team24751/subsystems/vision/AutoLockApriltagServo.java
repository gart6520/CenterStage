package org.firstinspires.ftc.team24751.subsystems.vision;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.team24751.Constants.INIT_VALUE.*;
import static org.firstinspires.ftc.team24751.Constants.FIELD_PARAMETER.*;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.team24751.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Optional;
import java.util.function.Supplier;

public class AutoLockApriltagServo {
    String servoName;
    LinearOpMode linearOpMode;
    Servo servo;
    ArrayList<Vector2d> aprilTagPos;

    public AutoLockApriltagServo(String servoName, LinearOpMode linearOpMode) {
        this.servoName = servoName;
        this.linearOpMode = linearOpMode;
        for (int id : BIG_APRIL_TAG_ID) {
            VectorF _pos = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(id).fieldPosition;
            aprilTagPos.add(new Vector2d(_pos.get(0), _pos.get(1)));
        }
    }

    //Must call
    public void initServo() {
        servo = linearOpMode.hardwareMap.get(Servo.class, servoName);
    }

    private double angleToPWMServo(double angle) {
        angle = normaliseAngle(angle);
        return angle / 300;
    }

    /**
     * Wrap angle to be from 0-360
     */
    private double normaliseAngle(double angle) {
        while (angle >= 360) angle -= 360;
        while (angle < 0) angle += 360;
        return angle;
    }

    private double PWMServoToAngle(double PWM) {
        return PWM * 300;
    }

    public void loop(Vector2d cameraPos, double botAngle) {
        //Raw angle from positive Ox
        ArrayList<Double> fieldAngles = new ArrayList<>();
        botAngle = normaliseAngle(botAngle);
        double cameraAngle = PWMServoToAngle(servo.getPosition());
        double globalCameraAngle = normaliseAngle(cameraAngle + botAngle + INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE);
        for (Vector2d apPos : aprilTagPos) {
            fieldAngles.add(angleToTurn(apPos, cameraPos));
        }

        //There's no fucking chance this ArrayList is empty, if it is somehow in testing I will cut my dick off
        double targetFieldAngle = fieldAngles.stream().min(Comparator.comparingDouble(a -> Math.abs(a - globalCameraAngle))).get();
        double servoAngle = normaliseAngle(targetFieldAngle - botAngle - INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE);
        if (servoAngle > 300) {
            //Decide if 0 deg (360 deg) or 300 deg is closer
            servoAngle = 360 - servoAngle < servoAngle - 300 ? 0 : 300;
        }
        servo.setPosition(angleToPWMServo(servoAngle));
    }

    private double angleToTurn(Vector2d apriltagPos, Vector2d cameraPos) {
        Vector2d cameraToApriltag = apriltagPos.minus(cameraPos);
        return normaliseAngle(Math.toDegrees(Math.atan2(cameraToApriltag.y, cameraToApriltag.x)));
    }
}
