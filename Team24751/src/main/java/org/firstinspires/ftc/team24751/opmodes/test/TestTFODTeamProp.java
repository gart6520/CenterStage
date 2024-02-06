package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.FRONT_CAMERA_NAME;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.team24751.subsystems.vision.Camera;
import org.firstinspires.ftc.team24751.subsystems.vision.RandomizationProcessor;

import java.util.List;
@TeleOp(name = "Test TFOD Team Prop", group = "Test")
public class TestTFODTeamProp extends LinearOpMode {


    /**
     * The variable to store our instance of the vision portal.
     */
    private final Camera frontCam = new Camera(FRONT_CAMERA_NAME, this);
    private RandomizationProcessor tfod = new RandomizationProcessor(frontCam, this);

    @Override
    public void runOpMode() {

        tfod.initProcessor();
        frontCam.buildCamera();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry.addData("Team prop position", tfod.getTeamPropPositionFromTfod().toString());
                telemetryTfod();

                telemetry.update();

                // Share the CPU.
                sleep(20);
            }
        }
    }

    private void telemetryTfod() {

//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2;
//            double y = (recognition.getTop() + recognition.getBottom()) / 2;
//
//            telemetry.addData("", " ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//            telemetry.addData("- Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));
//
//        }   // end for() loop

    }   // end method telemetryTfod()

}   // end class
