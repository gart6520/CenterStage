package org.firstinspires.ftc.team24751.opmodes.auto;

import static org.firstinspires.ftc.team24751.Constants.allianceColor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team24751.Constants;
import org.firstinspires.ftc.team24751.subsystems.AutoTrajectoryManager;

@Autonomous(name = "Wing Blue Auto", group = "Auto")
public class WingBlueAuto extends BaseAuto{
    @Override
    protected void initStartingCondition() {
        allianceColor = Constants.AllianceColor.BLUE;
        startingPos = AutoTrajectoryManager.StartingPos.wingBlue;
    }
}
