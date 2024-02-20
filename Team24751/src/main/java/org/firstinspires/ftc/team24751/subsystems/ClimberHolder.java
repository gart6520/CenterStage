package org.firstinspires.ftc.team24751.subsystems;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.CLIMBER_HOLDER;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.YELLOW_PIXEL_YEETER;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.ClimberHolder.HOLD_CLIMBER_HOLDER_POSITION;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.GENERAL_SERVO.GOBILDA_SERVO_PWM_RANGE;
import static org.firstinspires.ftc.team24751.Constants.HARDWARE_CONSTANT.YellowPixelYeeter.LOAD_YELLOW_PIXEL_YEETER_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ClimberHolder {
    ServoImplEx servo;
    LinearOpMode opMode;

    public ClimberHolder(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        servo = opMode.hardwareMap.get(ServoImplEx.class, CLIMBER_HOLDER);
        servo.setPwmRange(GOBILDA_SERVO_PWM_RANGE);
        servo.setPosition(HOLD_CLIMBER_HOLDER_POSITION);
    }
    public void setPosition (double position)
    {
        servo.setPosition(position);
    }
    public ServoImplEx getServo ()
    {
        return servo;
    }
    public void reset() {servo.setPosition(HOLD_CLIMBER_HOLDER_POSITION);}
}
