package org.firstinspires.ftc.team24751.rrtuning.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

public final class TwoDeadWheelEncodersMessage {
    public long timestamp;
    public PositionVelocityPair par;
    public PositionVelocityPair perp;

    public TwoDeadWheelEncodersMessage(PositionVelocityPair par, PositionVelocityPair perp) {
        this.timestamp = System.nanoTime();
        this.par = par;
        this.perp = perp;
    }
}
