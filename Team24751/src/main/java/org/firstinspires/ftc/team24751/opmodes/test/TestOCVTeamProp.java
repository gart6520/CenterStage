package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.FRONT_CAMERA_NAME;
import static org.firstinspires.ftc.team24751.Constants.VISION.FRONT_CAMERA_RESOLUTION;
import static org.firstinspires.ftc.team24751.Constants.allianceColor;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team24751.Constants;
import org.firstinspires.ftc.team24751.subsystems.vision.Camera;
import org.firstinspires.ftc.team24751.subsystems.vision.TeamPropProcessor;

@TeleOp(name = "Test OCV team prop", group = "Test")
public class TestOCVTeamProp extends LinearOpMode {

    /**
     * The variable to store our instance of the vision portal.
     */
    private final Camera frontCam = new Camera(FRONT_CAMERA_NAME, this);
    private final TeamPropProcessor teamPropProcessor = new TeamPropProcessor();

    @Override
    public void runOpMode() {

        allianceColor = Constants.AllianceColor.RED;

        frontCam.addProcessorToQueue(teamPropProcessor);
        frontCam.buildCamera(FRONT_CAMERA_RESOLUTION);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            //sleep(1500);
            while(opModeIsActive())
            {
                telemetry.addData("Team prop position", teamPropProcessor.getPos().toString());
                telemetry.addData("Contour area", teamPropProcessor.getArea());
                telemetry.addData("Center (X)", teamPropProcessor.getCenter());
                telemetry.update();
            }

            // Share the CPU.
            frontCam.close();
        }
    }
}
