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
        DcMotorEx encoder1 = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT);
        DcMotorEx encoder2 = hardwareMap.get(DcMotorEx.class, RIGHT_BACK);
        DcMotorEx encoder3 = hardwareMap.get(DcMotorEx.class, LEFT_FRONT);
        encoder1.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Encoder: 1, 2, 3",
                    encoder1.getCurrentPosition() + " " +
                            encoder2.getCurrentPosition() + " " +
                            encoder3.getCurrentPosition());
            telemetry.update();
        }
    }
}