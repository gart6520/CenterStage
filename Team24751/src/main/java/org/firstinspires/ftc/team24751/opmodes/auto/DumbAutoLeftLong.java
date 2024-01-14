package org.firstinspires.ftc.team24751.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Left Long", group = "!")
public class DumbAutoLeftLong extends DumbAuto{
    @Override
    public void runOpMode() throws InterruptedException {
        baseRun(10, false);
    }
}
