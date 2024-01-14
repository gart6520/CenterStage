package org.firstinspires.ftc.team24751.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Right Long", group = "!")
public class DumbAutoRightLong extends DumbAuto{
    @Override
    public void runOpMode() throws InterruptedException {
        baseRun(10, true);
    }
}
