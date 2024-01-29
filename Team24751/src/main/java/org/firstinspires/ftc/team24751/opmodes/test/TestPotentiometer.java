package org.firstinspires.ftc.team24751.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "TestPotentiometer", group = "Test")
public class TestPotentiometer extends LinearOpMode {
    AnalogInput potentiometer;

    private double voltageToAngle(double voltage) {
        return (-0.002346137 + 0.006297726 * voltage + 0.00003397255 * voltage * voltage + -2.971331e-7 * voltage * voltage * voltage + 8.515154e-10 * voltage * voltage * voltage * voltage);
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
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        double voltage;

        waitForStart();
        while (opModeIsActive()) {
            voltage = potentiometer.getVoltage();
            telemetry.addData("Voltage", voltage);
            telemetry.addData("Angle", voltageToAngle(voltage));
            telemetry.update();
        }
    }
}
