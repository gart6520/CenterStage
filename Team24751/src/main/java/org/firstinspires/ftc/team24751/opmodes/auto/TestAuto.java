package org.firstinspires.ftc.team24751.opmodes.auto;

import static org.firstinspires.ftc.team24751.Constants.allianceColor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team24751.Constants;

@Autonomous(name = "Test Auto")
public class TestAuto extends BaseAuto {

    @Override
    protected void setAllianceColor() {
        allianceColor = Constants.AllianceColor.TEST;
    }

    @Override
    public void runOpMode() {
        base_runOpMode();
    }
}
