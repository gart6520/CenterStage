package org.firstinspires.ftc.team24751;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class FTCObject {
    protected LinearOpMode opMode = null;
    public boolean isDebugMode = true;

    private void addData(String caption, String value) {
        if (isDebugMode) {
            opMode.telemetry.addData(caption, value);
        }
    }

    private void update() {
        if (isDebugMode) {
            opMode.telemetry.update();
        }
    }
}
