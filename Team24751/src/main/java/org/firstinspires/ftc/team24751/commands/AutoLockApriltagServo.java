package org.firstinspires.ftc.team24751.commands;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.team24751.Constants.INIT_VALUE.*;
import static org.firstinspires.ftc.team24751.Constants.FIELD_PARAMETER.*;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.team24751.subsystems.AngleServo;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.ArrayList;
import java.util.Comparator;

import static org.firstinspires.ftc.team24751.Utility.*;

import android.util.Pair;

public class AutoLockApriltagServo {
    LinearOpMode linearOpMode;
    AngleServo servo;
    ArrayList<Vector2d> aprilTagPos = new ArrayList<>();

    public AngleServo getAngleServo() {
        return servo;
    }

    private final double ANGLE_WEIGHT = 1;
    private final double DISTANCE_WEIGHT = 0;


    public AutoLockApriltagServo(String servoName, LinearOpMode linearOpMode) {
        servo = new AngleServo(servoName, INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE_DEG,
                270, linearOpMode);
        this.linearOpMode = linearOpMode;
        INIT_FIELD_PARAMETER();
        for (int id : BIG_APRIL_TAG_ID) {
            VectorF _pos = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(id).fieldPosition;
            aprilTagPos.add(new Vector2d(_pos.get(0), _pos.get(1)));
        }
    }

    //Must call
    public void initServo() {
        servo.init();
        servo.getServo().setDirection(Servo.Direction.REVERSE);
    }

    public void loop(Vector2d cameraPos, double botAngle) {
        //Raw angle from positive Ox
        ArrayList<Pair<Double, Double>> globalTargetAngles_Distances = new ArrayList<>();
        botAngle = wrapAngle(botAngle, WRAP_ANGLE_TYPE.zeroTo360);
        //Supposed camera angle from last set command (should be close to actual
        //camera angle if the the servo is fast enough) from the perspective of the robot
        double cameraAngle = servo.getAngle();
        //Same as above but now field-wise
        double globalCameraAngle = wrapAngle(cameraAngle + botAngle, WRAP_ANGLE_TYPE.zeroTo360);
        //Get all potential target angles to turn to
        for (Vector2d apPos : aprilTagPos) {
            Pair<Double, Double> t = new Pair<>(angleToTurn(apPos, cameraPos), distanceToTarget(apPos, cameraPos));
            globalTargetAngles_Distances.add(t);
        }

        double globalTargetAngle = selectAngle(globalTargetAngles_Distances, globalCameraAngle);
        //Actually turn the servo
        linearOpMode.telemetry.addData("Global target angle", globalTargetAngle);
        turnToAngle(globalTargetAngle, botAngle);
    }

    /**
     * @param globalTargetAngles_Distances list of potential target angle and distance
     * @param globalCameraAngle            Other param are for additional information that the compare algo might need
     */
    private double selectAngle(ArrayList<Pair<Double, Double>> globalTargetAngles_Distances, double globalCameraAngle) {
        //There's no fucking chance this ArrayList is empty, if it is somehow in testing I will cut my dick off
        Double targetAngle = globalTargetAngles_Distances.stream().min(Comparator.comparingDouble(a ->
                ANGLE_WEIGHT * Math.abs(a.first - globalCameraAngle) +
                        DISTANCE_WEIGHT * a.second
        )).get().first;
        return targetAngle;
    }

    private void turnToAngle(double globalTargetAngle, double botAngle) {
        //Angle to set position
        double servoAngle = wrapAngle(globalTargetAngle - botAngle, WRAP_ANGLE_TYPE.zeroTo360);
        linearOpMode.telemetry.addData("Servo Angle", servoAngle);
        servo.setAngle(servoAngle);
        linearOpMode.telemetry.addData("Servo PWM", servo.getServo().getPosition());
    }

    private double angleToTurn(Vector2d apriltagPos, Vector2d cameraPos) {
        Vector2d cameraToApriltag = apriltagPos.minus(cameraPos);
        return wrapAngle(Math.toDegrees(Math.atan2(cameraToApriltag.y, cameraToApriltag.x)), WRAP_ANGLE_TYPE.zeroTo360);
    }

    private double distanceToTarget(Vector2d apriltagPos, Vector2d cameraPos) {
        Vector2d cameraToApriltag = apriltagPos.minus(cameraPos);
        return cameraToApriltag.norm();
    }
}
