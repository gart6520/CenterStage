package org.firstinspires.ftc.team24751.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;

public class LedIndicator {
    // Hardware map
    private HardwareMap hardwareMap = null;
    // Digital channels
    private DigitalChannel red = null;
    private DigitalChannel green = null;

    /**
     * Led indicator object for handling REV digital led indicator
     * @param opMode
     */
    public LedIndicator(LinearOpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    /**
     * Init digital pins
     */
    public void init() {
        // Get DigitalChannel object
        red = hardwareMap.get(DigitalChannel.class, LED_RED);
        green = hardwareMap.get(DigitalChannel.class, LED_GREEN);

        // Set output mode
        red.setMode(DigitalChannel.Mode.OUTPUT);
        green.setMode(DigitalChannel.Mode.OUTPUT);

        // Turn off the led
        turnOff();
    }

    /**
     * Turn off leds
     */
    public void turnOff() {
        // Disable high
        red.setState(true);
        green.setState(true);
    }

    /**
     * Show green color
     */
    public void setGreen() {
        // Disable high
        red.setState(true);

        // Enable low
        green.setState(false);
    }

    /**
     * Show red color
     */
    public void setRed() {
        // Enable low
        red.setState(false);

        // Disable high
        green.setState(true);
    }

    /**
     * Show amber color
     * (= red + green)
     */
    public void setAmber() {
        // Enable low
        red.setState(false);
        green.setState(false);
    }

    /**
     *  Note: Opposite of the state of DIO
     * @param redOn true if on, false is off
     * @param greenOn true if on, false is off
     * */
    public void setColor (boolean redOn, boolean greenOn)
    {
        this.red.setState(!redOn);
        this.green.setState(!greenOn);
    }
}
