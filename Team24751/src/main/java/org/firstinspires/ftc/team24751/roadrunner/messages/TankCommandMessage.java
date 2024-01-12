package org.firstinspires.ftc.team24751.roadrunner.messages;

public final class TankCommandMessage {
    public long timestamp;
    public double voltage;
    public double leftPower;
    public double rightPower;

    public TankCommandMessage(double voltage, double leftPower, double rightPower) {
        this.timestamp = System.nanoTime();
        this.voltage = voltage;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
    }
}
