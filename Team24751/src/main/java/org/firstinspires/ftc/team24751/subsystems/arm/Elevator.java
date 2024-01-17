package org.firstinspires.ftc.team24751.subsystems.arm;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.LEFT_ELEVATOR_MOTOR;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.RIGHT_ELEVATOR_MOTOR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Elevator {
    DcMotorEx leftElevatorMotor;
    DcMotorEx rightElevatorMotor;
    LinearOpMode opMode;

    public Elevator(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        leftElevatorMotor = opMode.hardwareMap.get(DcMotorEx.class, LEFT_ELEVATOR_MOTOR);
        leftElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevatorMotor = opMode.hardwareMap.get(DcMotorEx.class, RIGHT_ELEVATOR_MOTOR);
        rightElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        leftElevatorMotor.setPower(power);
        rightElevatorMotor.setPower(power);
    }
}
