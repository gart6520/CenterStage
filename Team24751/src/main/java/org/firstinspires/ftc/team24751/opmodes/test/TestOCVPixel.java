package org.firstinspires.ftc.team24751.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.team24751.subsystems.vision.sim.PixelAlignProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Rect;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;
import static org.firstinspires.ftc.team24751.Constants.VISION.FRONT_CAMERA_RESOLUTION;

import java.util.List;

@TeleOp(name = "Test OCV pixel", group = "Test")
public class TestOCVPixel extends LinearOpMode {
    private VisionPortal visionPortal;
    private PixelAlignProcessor pixelProcessor;

    @Override
    public void runOpMode() throws InterruptedException {
        pixelProcessor = new PixelAlignProcessor();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(CameraName.class, FRONT_CAMERA_NAME))
                .setCameraResolution(FRONT_CAMERA_RESOLUTION)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .addProcessor(pixelProcessor)
                .build();

        waitForStart();
        while (opModeIsActive()) {
            List<Rect> recognitions = pixelProcessor.getRecognition();

            for (Rect obj : recognitions) {
                if (obj == null) continue;
                telemetry.addData("", " ");
                telemetry.addData("- Position", "%d / %d", obj.x, obj.y);
                telemetry.addData("- Size", "%d x %d", obj.width, obj.height);
                telemetry.addLine();
            }

            List<Double> angs = pixelProcessor.getObjectAngles();
            for (Double a : angs) {
                telemetry.addData("- Ang", a);
            }

            telemetry.update();
        }
    }
}
