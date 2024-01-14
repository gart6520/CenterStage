package org.firstinspires.ftc.team24751.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Left Short", group = "!")
public class DumbAutoLeftShort extends DumbAuto{
    @Override
    public void runOpMode() throws InterruptedException {
        baseRun(4, false);
    }
}
