package org.firstinspires.ftc.team24751.opmodes.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.standard.LynxSetModuleLEDColorCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Test Lynx LED", group = "Test")
public class TestLynxLED extends LinearOpMode {

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // Get hub objects
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        // Enable bulk read
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Create lynx led command
        LynxSetModuleLEDColorCommand cmd = new LynxSetModuleLEDColorCommand(allHubs.get(1), (byte)0xFF, (byte)0xFF, (byte)0xFF);

        //ArrayList<LynxSetModuleLEDColorCommand> cmds = new ArrayList<>()

        waitForStart();

        // Send command
        try {
            cmd.send();
        } catch (InterruptedException | LynxNackException e) {
            e.printStackTrace();
        }

        while (opModeIsActive());
    }
}
