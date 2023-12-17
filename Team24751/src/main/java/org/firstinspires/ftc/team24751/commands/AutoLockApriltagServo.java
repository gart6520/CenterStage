package org.firstinspires.ftc.team24751.commands;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.team24751.Constants.INIT_VALUE.*;
import static org.firstinspires.ftc.team24751.Constants.FIELD_PARAMETER.*;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.team24751.subsystems.AngleServo;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.ArrayList;
import java.util.Comparator;

import static org.firstinspires.ftc.team24751.Utility.*;

public class AutoLockApriltagServo {
    LinearOpMode linearOpMode;
    AngleServo servo;
    ArrayList<Vector2d> aprilTagPos;

    public AutoLockApriltagServo(String servoName, LinearOpMode linearOpMode) {
        servo = new AngleServo(servoName, INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE_DEG, 300, linearOpMode);
        this.linearOpMode = linearOpMode;
        for (int id : BIG_APRIL_TAG_ID) {
            VectorF _pos = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(id).fieldPosition;
            aprilTagPos.add(new Vector2d(_pos.get(0), _pos.get(1)));
        }
    }

    //Must call
    public void initServo() {
        servo.init();
    }

    public void loop(Vector2d cameraPos, double botAngle) {
        //Raw angle from positive Ox
        ArrayList<Double> globalTargetAngles = new ArrayList<>();
        botAngle = wrapAngle(botAngle, WRAP_ANGLE_TYPE.zeroTo360);
        //Supposed camera angle from last set command (should be close to actual
        //camera angle if the the servo is fast enough) from the perspective of the robot
        double cameraAngle = servo.getAngle();
        //Same as above but now field-wise
        double globalCameraAngle = wrapAngle(cameraAngle + botAngle + INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE_DEG, WRAP_ANGLE_TYPE.zeroTo360);
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
     * @param globalCameraAngle  Other param are for additional information that the compare algo might need
     */
    private double selectAngle(ArrayList<Double> globalTargetAngles, double globalCameraAngle) {
        return globalTargetAngles.stream().min(Comparator.comparingDouble(a -> Math.abs(a - globalCameraAngle))).get();
    }

    private void turnToAngle(double globalTargetAngle, double botAngle) {
        //Angle to set position
        double servoAngle = wrapAngle(globalTargetAngle - botAngle - INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE_DEG, WRAP_ANGLE_TYPE.zeroTo360);
        if (servoAngle > 300) {
            //Decide if 0 deg (360 deg) or 300 deg is closer
            servoAngle = 360 - servoAngle < servoAngle - 300 ? 0 : 300;
        }
        servo.setAngle(servoAngle);
    }

    private double angleToTurn(Vector2d apriltagPos, Vector2d cameraPos) {
        Vector2d cameraToApriltag = apriltagPos.minus(cameraPos);
        return wrapAngle(Math.toDegrees(Math.atan2(cameraToApriltag.y, cameraToApriltag.x)), WRAP_ANGLE_TYPE.zeroTo360);
    }
}
