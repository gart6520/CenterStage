package org.firstinspires.ftc.team24751.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import static org.firstinspires.ftc.team24751.Constants.DEVICES.*;
import static org.firstinspires.ftc.team24751.Constants.ORIENTATIONS.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Gyro {
    // Useful instances
    private HardwareMap hardwareMap = null;

    // IMU instance
    private IMU imu = null;

    /**
     * Gyro class for getting data from IMU
     * (Sorry it's just my convention to call it gyro)
     * @param opMode opMode instance. If you are init this from linearOpMode, just pass `this`
     */
    public Gyro(LinearOpMode opMode) {
        hardwareMap = opMode.hardwareMap;
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
        imu = hardwareMap.get(IMU.class, IMU_NAME);

        // Config IMU
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO_DIRECTION, HUB_USB_DIRECTION));
        imu.initialize(parameters);

        // Reset yaw
        imu.resetYaw();
    }

    /**
     * Get yaw from gyro
     * @return Yaw angle, in radian
     */
    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
