package org.firstinspires.ftc.team24751.opmodes.auto;

import static org.firstinspires.ftc.team24751.Constants.allianceColor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team24751.Constants;
import org.firstinspires.ftc.team24751.commands.AutoTrajectoryManager;

@Autonomous(name = "Backdrop Red Auto", group = "Auto")
public class BackdropRedAuto extends BaseAuto{
    @Override
    protected void initStartingCondition() {
        allianceColor = Constants.AllianceColor.RED;
        startingPos = AutoTrajectoryManager.StartingPos.backdropRed;
    }
}
