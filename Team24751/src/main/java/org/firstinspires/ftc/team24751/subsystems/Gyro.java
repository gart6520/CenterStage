package org.firstinspires.ftc.team24751.subsystems;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import static org.firstinspires.ftc.team24751.Constants.BOT_PARAMETERS.INITIAL_BOT_ANGLE_DEG_BLUE;
import static org.firstinspires.ftc.team24751.Constants.BOT_PARAMETERS.INITIAL_BOT_ANGLE_DEG_RED;
import static org.firstinspires.ftc.team24751.Constants.BOT_PARAMETERS.INITIAL_BOT_ANGLE_DEG_TEST;
import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;
import static org.firstinspires.ftc.team24751.Constants.ORIENTATIONS.*;
import static org.firstinspires.ftc.team24751.Utility.wrapAngle;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team24751.Constants;
import org.firstinspires.ftc.team24751.Utility;

public class Gyro {
    // Useful instances
    private LinearOpMode opMode = null;

    // IMU instance
    private AHRS navx_device = null;
    private double initialBotAngleDeg;

    /**
     * Gyro class for getting data from IMU
     * (Sorry it's just my convention to call it gyro)
     *
     * @param opMode opMode instance. If you are init this from linearOpMode, just pass `this`
     */
    public Gyro(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * Init method for Gyro class
     * <p>
     * This method:
     * - Config control hub's orientation
     * - Init IMU
     * - Reset yaw
     * </p>
     */
    public void init() {

        //Set correct initial bot angle
        switch(Constants.allianceColor)
        {
            case BLUE:
                initialBotAngleDeg = INITIAL_BOT_ANGLE_DEG_BLUE;
                break;
            case RED:
                initialBotAngleDeg = INITIAL_BOT_ANGLE_DEG_RED;
                break;
            case TEST:
                initialBotAngleDeg = INITIAL_BOT_ANGLE_DEG_TEST;
        }

        navx_device = AHRS.getInstance(opMode.hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData);

        // Reset yaw
        navx_device.zeroYaw();
    }

    /**
     * Get yaw from gyro
     *
     * @return Yaw angle, in radian
     */
    public double getYawRad() {
        return wrapAngle(Math.toRadians(-navx_device.getYaw() + initialBotAngleDeg), Utility.WRAP_ANGLE_TYPE.minusPiToPi);
    }/**
     * Get yaw from gyro
     *
     * @return Yaw angle, in degree
     */
    public double getYawDeg() {
        return wrapAngle(-navx_device.getYaw() + initialBotAngleDeg, Utility.WRAP_ANGLE_TYPE.minus180To180);
    }

    /**
     * Reset the yaw value
     */
    public void reset() {
        navx_device.zeroYaw();
    }
}
