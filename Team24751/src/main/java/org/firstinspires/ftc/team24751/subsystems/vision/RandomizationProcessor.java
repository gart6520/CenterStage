package org.firstinspires.ftc.team24751.subsystems.vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.team24751.Constants;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Comparator;
import java.util.List;

public class RandomizationProcessor {
    TfodProcessor tfod;
    Camera camera;
    LinearOpMode linearOpMode;
    String modelFileName;
    String[] labels;

    public enum TeamPropPosition {
        NONE, LEFT, CENTER, RIGHT
    }

    public RandomizationProcessor(Camera camera, String modelFileName, String[] labels, LinearOpMode linearOpMode) {
        this.camera = camera;
        this.linearOpMode = linearOpMode;
        this.modelFileName = modelFileName;
        this.labels = labels;
    }
    /**
     * Return TeamPropPosition.NONE if no team prop is detected
     * otherwise return a TeamPropPosition appropriate to the team prop
     * position on the spike mark indicator
     * */
    public TeamPropPosition getTeamPropPositionFromTfod() {
        List<Recognition> recogs = tfod.getRecognitions();
        if (recogs.size() < 1) return TeamPropPosition.NONE;
        //Get the recognition with largest confidence
        Recognition recog = recogs.stream().max(Comparator.comparingDouble(Recognition::getConfidence)).get();
        //Get the approx center of the team prop
        double center = (recog.getRight() + recog.getLeft()) / 2;
        telemetry.addData("Team prop center coord X", center);
        telemetry.update();
        //Disable processor to save on resource
        camera.disableProcessor(tfod);
        if (center <= Constants.VISION.TFOD.TEAM_PROP_LEFT_CENTER) return TeamPropPosition.LEFT;
        if (center <= Constants.VISION.TFOD.TEAM_PROP_CENTER_RIGHT) return TeamPropPosition.CENTER;
        return TeamPropPosition.RIGHT;
    }

    public void initTfodProcessor() {
        tfod = new TfodProcessor.Builder()
                .setModelFileName(modelFileName)
                .setModelLabels(labels)
                .build();

        camera.addProcessorToQueue(tfod);
    }
}
