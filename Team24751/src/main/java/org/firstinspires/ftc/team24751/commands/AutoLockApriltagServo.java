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

public class AutoLockApriltagServo {
    LinearOpMode linearOpMode;
    AngleServo servo;
    ArrayList<Vector2d> aprilTagPos = new ArrayList<>();

    public AutoLockApriltagServo(String servoName, LinearOpMode linearOpMode) {
        servo = new AngleServo(servoName, INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE_DEG, 300, linearOpMode);
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
        ArrayList<Double> globalTargetAngles = new ArrayList<>();
        botAngle = wrapAngle(botAngle, WRAP_ANGLE_TYPE.zeroTo360);
        //Supposed camera angle from last set command (should be close to actual
        //camera angle if the the servo is fast enough) from the perspective of the robot
        double cameraAngle = servo.getAngle();
        //Same as above but now field-wise
        double globalCameraAngle = wrapAngle(cameraAngle + botAngle, WRAP_ANGLE_TYPE.zeroTo360);
        //Get all potential target angles to turn to
        for (Vector2d apPos : aprilTagPos) {
            globalTargetAngles.add(angleToTurn(apPos, cameraPos));
        }

        //There's no fucking chance this ArrayList is empty, if it is somehow in testing I will cut my dick off
        double globalTargetAngle = selectAngle(globalTargetAngles, globalCameraAngle);
        //Actually turn the servo
        linearOpMode.telemetry.addData("Global target angle", globalTargetAngle);
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
        double servoAngle = wrapAngle(globalTargetAngle - botAngle, WRAP_ANGLE_TYPE.zeroTo360);
        linearOpMode.telemetry.addData("Servo Angle", servoAngle);
        linearOpMode.telemetry.addData("Bot Angle", botAngle);

        servo.setAngle(servoAngle);
        linearOpMode.telemetry.addData("Servo PWM", servo.getServo().getPosition());
    }

    private double angleToTurn(Vector2d apriltagPos, Vector2d cameraPos) {
        Vector2d cameraToApriltag = apriltagPos.minus(cameraPos);
        return wrapAngle(Math.toDegrees(Math.atan2(cameraToApriltag.y, cameraToApriltag.x)), WRAP_ANGLE_TYPE.zeroTo360);
    }
}
