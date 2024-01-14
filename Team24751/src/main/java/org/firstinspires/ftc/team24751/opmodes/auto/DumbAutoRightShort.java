package org.firstinspires.ftc.team24751.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Right Short", group = "!")
public class DumbAutoRightShort extends DumbAuto{
    @Override
    public void runOpMode() throws InterruptedException {
        baseRun(4, true);
    }
}
