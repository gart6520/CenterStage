package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.*;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.POTENTIOMETER.LUT_DATA_FILE_NAME;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team24751.subsystems.AngleServo;
import org.firstinspires.ftc.team24751.subsystems.sensor.Potentiometer;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name = "Potentiometer LUT Generator", group = "Misc")
public class PotentiometerLUTGenerator extends LinearOpMode {


    AngleServo angleServo = new AngleServo("preciseAngleServo", 0, GENERAL_SERVO.GOBILDA_SERVO_ANGLE_RANGE, this);
    Potentiometer potentiometer = new Potentiometer(this);
    FileWriter fileWriter;

    public PotentiometerLUTGenerator() throws FileNotFoundException {
        try {
            fileWriter = new FileWriter(LUT_DATA_FILE_NAME);
        }
        catch (IOException e)
        {
            fileWriter = null;
            telemetry.addLine("Cannot open LUT data file");
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() {
        angleServo.init(GENERAL_SERVO.GOBILDA_SERVO_PWM_RANGE);
        potentiometer.init();
        waitForStart();
        angleServo.getServo().setPosition(0);
        while (opModeIsActive()) {
            if (!gamepad1.circle) continue;
            for (int i = 0; i <= 300; i++) {
                angleServo.setAngle(i);
                sleep(100);

                try {
                    telemetry.addLine(potentiometer.getVoltage() + " " + i + '\n');
                    telemetry.update();
                    fileWriter.write(potentiometer.getVoltage() + " " + i + '\n');
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }
            break;
        }
        try {
            fileWriter.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }
}
