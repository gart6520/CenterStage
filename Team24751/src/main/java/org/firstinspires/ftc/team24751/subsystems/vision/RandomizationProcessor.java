package org.firstinspires.ftc.team24751.subsystems.vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.team24751.Constants.VISION.TFOD.TEAM_PROP_BLUE_LABEL;
import static org.firstinspires.ftc.team24751.Constants.VISION.TFOD.TEAM_PROP_RED_LABEL;
import static org.firstinspires.ftc.team24751.Constants.VISION.TFOD.TFOD_PIXEL_MODEL_FILE;
import static org.firstinspires.ftc.team24751.Constants.VISION.TFOD.TFOD_TEAM_PROP_MODEL_FILE;
import static org.firstinspires.ftc.team24751.Constants.allianceColor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.team24751.Constants;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public class RandomizationProcessor {
    TfodProcessor tfod;
    Camera camera;
    LinearOpMode linearOpMode;


    public enum TeamPropPosition {
        NONE, LEFT, CENTER, RIGHT
    }

    public RandomizationProcessor(Camera camera, LinearOpMode linearOpMode) {
        this.camera = camera;
        this.linearOpMode = linearOpMode;
    }

    /**
     * Return TeamPropPosition.NONE if no team prop is detected
     * otherwise return a TeamPropPosition appropriate to the team prop
     * position on the spike mark indicator
     */
    public TeamPropPosition getTeamPropPositionFromTfod() {
        List<Recognition> recogs = tfod.getRecognitions();
        if (recogs.size() < 1) return TeamPropPosition.NONE;
        //Get the recognition with largest confidence and has the same color as alliance's
        Optional<Recognition> _recog = recogs.stream().filter((recognition -> {
            if (allianceColor == Constants.AllianceColor.BLUE)
                return recognition.getLabel().equals(TEAM_PROP_BLUE_LABEL);
            if (allianceColor == Constants.AllianceColor.RED)
                return recognition.getLabel().equals(TEAM_PROP_RED_LABEL);
            return true;
        })).max(Comparator.comparingDouble(Recognition::getConfidence));
        if (!_recog.isPresent()) {
            telemetry.addLine("No team prop detected");
            return TeamPropPosition.NONE;
        }
        Recognition recog = _recog.get();
        //Get the approx center of the team prop
        double center = (recog.getRight() + recog.getLeft()) / 2;
        telemetry.addData("Team prop center coord X", center);
        //Disable processor to save on resource
        camera.disableProcessor(tfod);
        //Decide position of team prop based on region
        if (center <= Constants.VISION.TFOD.TEAM_PROP_LEFT_CENTER) return TeamPropPosition.LEFT;
        if (center <= Constants.VISION.TFOD.TEAM_PROP_CENTER_RIGHT) return TeamPropPosition.CENTER;
        return TeamPropPosition.RIGHT;
    }

    public void initTfodProcessor() {
        tfod = new TfodProcessor.Builder()
                .setIsModelQuantized(true)
                .setIsModelTensorFlow2(true)
                .setModelInputSize(256)
                .setNumExecutorThreads(4)
                .setNumDetectorThreads(4)
                .setModelFileName(TFOD_TEAM_PROP_MODEL_FILE)
                .build();
        tfod.setMinResultConfidence(0.9f);
        camera.addProcessorToQueue(tfod);

    }
    public List<Recognition> getRecognitions()
    {
        return tfod.getRecognitions();
    }
}
