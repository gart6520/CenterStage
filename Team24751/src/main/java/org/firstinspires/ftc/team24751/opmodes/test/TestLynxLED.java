package org.firstinspires.ftc.team24751.opmodes.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.standard.LynxSetModuleLEDColorCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "Test Lynx LED", group = "Test")
public class TestLynxLED extends LinearOpMode {
    private double mod(double x, double y) {
        return x - Math.floor(x / y) * y;
    }

    /**
     * ! \brief Convert HSV to RGB color space
     * <p>
     * Converts a given set of HSV values `h', `s', `v' into RGB
     * coordinates. The output RGB values are in the range [0, 1], and
     * the input HSV values are in the ranges h = [0, 360], and s, v =
     * [0, 1], respectively.
     * <p>
     * \param fR Red component, used as output, range: [0, 1]
     * \param fG Green component, used as output, range: [0, 1]
     * \param fB Blue component, used as output, range: [0, 1]
     * \param fH Hue component, used as input, range: [0, 360]
     * \param fS Hue component, used as input, range: [0, 1]
     * \param fV Hue component, used as input, range: [0, 1]
     */
    Mat.Tuple3<Double> HSVtoRGB(double fH, double fS, double fV) {
        double fR, fG, fB;
        double fC = fV * fS; // Chroma
        double fHPrime = mod(fH / 60.0, 6);
        double fX = fC * (1 - Math.abs(mod(fHPrime, 2) - 1));
        double fM = fV - fC;

        if (0 <= fHPrime && fHPrime < 1) {
            fR = fC;
            fG = fX;
            fB = 0;
        } else if (1 <= fHPrime && fHPrime < 2) {
            fR = fX;
            fG = fC;
            fB = 0;
        } else if (2 <= fHPrime && fHPrime < 3) {
            fR = 0;
            fG = fC;
            fB = fX;
        } else if (3 <= fHPrime && fHPrime < 4) {
            fR = 0;
            fG = fX;
            fB = fC;
        } else if (4 <= fHPrime && fHPrime < 5) {
            fR = fX;
            fG = 0;
            fB = fC;
        } else if (5 <= fHPrime && fHPrime < 6) {
            fR = fC;
            fG = 0;
            fB = fX;
        } else {
            fR = 0;
            fG = 0;
            fB = 0;
        }

        fR += fM;
        fG += fM;
        fB += fM;
        return new Mat.Tuple3<>(fR, fG, fB);
    }

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // Get hub objects
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        // Enable bulk read
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();
        int i = 0;
        while (opModeIsActive()) {
            Mat.Tuple3<Double> rgb = HSVtoRGB(i, 1, 1);

            // Command for the first hub
            LynxSetModuleLEDColorCommand cmd0 = new LynxSetModuleLEDColorCommand(allHubs.get(0),
                    (byte) Math.round(rgb.get_0() * 255),
                    (byte) Math.round(rgb.get_1() * 255),
                    (byte) Math.round(rgb.get_2() * 255));

            // Command for the second hub
            LynxSetModuleLEDColorCommand cmd1 = new LynxSetModuleLEDColorCommand(allHubs.get(1),
                    (byte) Math.round(rgb.get_0() * 255),
                    (byte) Math.round(rgb.get_1() * 255),
                    (byte) Math.round(rgb.get_2() * 255));

            // Send command
            try {
                cmd0.send();
                cmd1.send();
                sleep(5);
            } catch (InterruptedException | LynxNackException e) {
                e.printStackTrace();
            }

            // List generated color
            /*telemetry.addLine(Math.round(rgb.get_0() * 255) + " " +
                    Math.round(rgb.get_1() * 255) + " " +
                    Math.round(rgb.get_2() * 255)
            );

            telemetry.update();*/

            // Start over if it's done
            if (i >= 360) i = 0;
            else i++;
        }
    }
}
