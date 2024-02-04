package org.firstinspires.ftc.team24751.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoder {
    public enum Direction {
        FORWARD, REVERSE
    }

    DcMotorEx motor;
    Direction direction = Direction.FORWARD;

    public Encoder(DcMotorEx motor) {
        this.motor = motor;
    }

    public int getPosition() {
        return motor.getCurrentPosition() * (direction == Direction.FORWARD ? 1 : -1);
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public Direction getDirection() {
        return direction;
    }

    public void reset() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
