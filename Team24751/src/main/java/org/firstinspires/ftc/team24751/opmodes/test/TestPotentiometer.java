package org.firstinspires.ftc.team24751.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    private double getAngle() {
        return potentiometer.getAngle(true);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        potentiometer.init();
        potentiometer.printLUT();

        waitForStart();
        while (opModeIsActive()) {
            double voltage = potentiometer.getVoltage();
            double smoothVoltage = potentiometer.getSmoothVoltage();
            telemetry.addData("Voltage", voltage);
            telemetry.addData("Smooth Voltage", smoothVoltage);
            telemetry.addData("Angle", getAngle());
            telemetry.update();
        }
    }
}
