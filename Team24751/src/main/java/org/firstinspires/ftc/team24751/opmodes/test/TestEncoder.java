package org.firstinspires.ftc.team24751.opmodes.test;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test Encoders", group = "Test")
public class TestEncoder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx encoder1 = hardwareMap.get(DcMotorEx.class, LEFT_ODO);
        DcMotorEx encoder2 = hardwareMap.get(DcMotorEx.class, RIGHT_ODO);
        DcMotorEx encoder3 = hardwareMap.get(DcMotorEx.class, FRONT_ODO);

        encoder1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encoder2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encoder3.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        encoder1.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder2.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Encoder: L - R - Fr",
                    encoder1.getCurrentPosition() + " " +
                            encoder2.getCurrentPosition() + " " +
                            encoder3.getCurrentPosition());
            telemetry.update();
        }
    }
}