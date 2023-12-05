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
        ArrayList<Double> globalTargetAngles = new ArrayList<>();
        botAngle = normaliseAngle(botAngle);
        //Supposed camera angle from last set command (should be close to actual
        //camera angle if the the servo is fast enough) from the perspective of the robot
        double cameraAngle = PWMServoToAngle(servo.getPosition());
        //Same as above but now field-wise
        double globalCameraAngle = normaliseAngle(cameraAngle + botAngle + INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE);
        //Get all potential target angles to turn to
        for (Vector2d apPos : aprilTagPos) {
            globalTargetAngles.add(angleToTurn(apPos, cameraPos));
        }

        //There's no fucking chance this ArrayList is empty, if it is somehow in testing I will cut my dick off
        double globalTargetAngle = selectAngle(globalTargetAngles, globalCameraAngle);
        //Actually turn the servo
        turnToAngle(globalTargetAngle, botAngle);
    }
    /**
     * @param globalTargetAngles list of potential target angle
     * @param globalCameraAngle Other param are for additional information that the compare algo might need
     * */
    private double selectAngle (ArrayList<Double> globalTargetAngles, double globalCameraAngle)
    {
        return globalTargetAngles.stream().min(Comparator.comparingDouble(a -> Math.abs(a - globalCameraAngle))).get();
    }
    private void turnToAngle(double globalTargetAngle, double botAngle) {
        //Angle to set position
        double servoAngle = normaliseAngle(globalTargetAngle - botAngle - INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE);
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
