package org.firstinspires.ftc.team24751.subsystems.arm;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.ELEVATOR_MOTOR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Elevator {
    DcMotorEx elevatorMotor;
    LinearOpMode opMode;
    public Elevator (LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        elevatorMotor = opMode.hardwareMap.get(DcMotorEx.class, ELEVATOR_MOTOR);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        elevatorMotor.setPower(power);
    }
}
