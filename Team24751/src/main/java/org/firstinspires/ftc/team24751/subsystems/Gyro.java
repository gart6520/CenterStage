package org.firstinspires.ftc.team24751.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;
import static org.firstinspires.ftc.team24751.Constants.INIT_VALUE.INITIAL_BOT_ANGLE_DEG;
import static org.firstinspires.ftc.team24751.Constants.ORIENTATIONS.*;
import static org.firstinspires.ftc.team24751.Utility.wrapAngle;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team24751.Utility;

public class Gyro {
    // Useful instances
    private LinearOpMode opMode = null;

    // IMU instance
    private IMU imu = null;

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
        // Get IMU from hardwareMap
        imu = opMode.hardwareMap.get(IMU.class, IMU_NAME);

        // Config IMU
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO_DIRECTION, HUB_USB_DIRECTION));
        imu.initialize(parameters);

        // Reset yaw
        imu.resetYaw();
    }

    /**
     * Get yaw from gyro
     *
     * @return Yaw angle, in radian
     */
    public double getYawRad() {
        return wrapAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.toRadians(INITIAL_BOT_ANGLE_DEG), Utility.WRAP_ANGLE_TYPE.minusPiToPi);
    }/**
     * Get yaw from gyro
     *
     * @return Yaw angle, in degree
     */
    public double getYawDeg() {
        return wrapAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + INITIAL_BOT_ANGLE_DEG, Utility.WRAP_ANGLE_TYPE.minus180To180);
    }

    /**
     * Reset the yaw value
     */
    public void reset() {
        imu.resetYaw();
    }
}
