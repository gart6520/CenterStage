package org.firstinspires.ftc.team24751.opmodes.test;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.team24751.subsystems.vision.sim.PixelProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Rect;

import java.util.List;

@TeleOp(name = "Test Pixel Recognition", group = "Test")
public class TestPixelRecog extends LinearOpMode {
    private VisionPortal visionPortal;
    private PixelProcessor pixelProcessor;

    @Override
    public void runOpMode() throws InterruptedException {
        pixelProcessor = new PixelProcessor();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(CameraName.class, "fieldCamera"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .addProcessor(pixelProcessor)
                .build();

        waitForStart();
        while (opModeIsActive()) {
            List<Rect> recognitions = pixelProcessor.getRecognitions();

            for (Rect obj : recognitions) {
                telemetry.addData(""," ");
                telemetry.addData("- Position", "%d / %d", obj.x, obj.y);
                telemetry.addData("- Size", "%d x %d", obj.width, obj.height);
            }

            telemetry.update();
        }
    }
}
