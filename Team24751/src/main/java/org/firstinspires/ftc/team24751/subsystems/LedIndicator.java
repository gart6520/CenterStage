package org.firstinspires.ftc.team24751.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;

public class LedIndicator {
    // OpMode
    private LinearOpMode opMode;

    // Led name
    String redName, greenName;

    // Digital channels
    private DigitalChannel red = null;
    private DigitalChannel green = null;

    /**
     * Led indicator object for handling REV digital led indicator
     * @param _opMode
     */
    public LedIndicator(LinearOpMode _opMode, String _redName, String _greenName) {
        this.opMode = _opMode;
        this.redName = _redName;
        this.greenName = _greenName;
    }

    /**
     * Init digital pins
     */
    public void init() {
        // Get DigitalChannel object
        red = this.opMode.hardwareMap.get(DigitalChannel.class, this.redName);
        green = this.opMode.hardwareMap.get(DigitalChannel.class, this.greenName);

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
