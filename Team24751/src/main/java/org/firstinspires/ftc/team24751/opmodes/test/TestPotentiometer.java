package org.firstinspires.ftc.team24751.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.team24751.subsystems.sensor.Potentiometer;

import java.io.FileNotFoundException;

@TeleOp(name = "TestPotentiometer", group = "Test")
public class TestPotentiometer extends LinearOpMode {
    Potentiometer potentiometer;

    {
        try {
            potentiometer = new Potentiometer(this);
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
    }

    private double voltageToAngle(double voltage) {
        return potentiometer.getAngle();
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
        potentiometer.init();
        double voltage;
        potentiometer.printLUT();

        waitForStart();
        while (opModeIsActive()) {
            voltage = potentiometer.getVoltage();
            telemetry.addData("Voltage", voltage);
            telemetry.addData("Angle", voltageToAngle(voltage));
            telemetry.update();
        }
    }
}
