package org.firstinspires.ftc.team24751;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Constants {
    // Some general constants
    public static final double INCH_TO_MM = 25.4;
    public static final double MM_TO_INCH = 1/INCH_TO_MM;

    /**
     * Device names
     * They are kept here for convenient (eg to quickly change when needed)
     */
    public static class DEVICES {
        // Drivebase motors
        public static final String LEFT_FRONT = "leftFront";
        public static final String LEFT_BACK = "leftBack";
        public static final String RIGHT_FRONT = "rightFront";
        public static final String RIGHT_BACK = "rightBack";

        // Sensors
        public static final String IMU_NAME = "IMU";
    }

    /**
     * Orientation of some objects
     * It should be changed to match with the real bot
     */
    public static class ORIENTATIONS {
        // Control Hub orientation
        public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    }

    /**
     * Some speed constants
     * NOTE: drivebase speed constant should NOT be used with auto. Auto speed are stored separatedly
     */
    public static class SPEED {
        // Drivebase speed
        public static final double DRIVEBASE_SPEED_Y = 0.8; // Up/down speed
        public static final double DRIVEBASE_SPEED_X = 0.8; // Left/right speed
        public static final double DRIVEBASE_SPEED_Z = 0.8; // Rotate speed
    }

    /**
     * Some drivebase constants (for auto and semi-auto)
     */
    public static class DRIVEBASE {
        // You don't need to worry about a bunch of constants, a good Java compiler will optimize this

        // Pulse per revolution
        public static final double BASE_ENCODER_PPR = ((((1+(46/17))) * (1+(46/11))) * 28);

        // Counts per pulse
        public static final double BASE_ENCODER_CPP = 4;

        // Count per revolution
        public static final double BASE_CPR = BASE_ENCODER_PPR * BASE_ENCODER_CPP;

        // Wheel diameter (in inch)
        // For goBilda's mecanum wheel, the diameter is 96mm
        public static final double WHEEL_D_IN = 96*MM_TO_INCH;

        // Inch traveled per tick/count
        // This can either be calculated using the formula, or by measuring
        // The choice is yours
        public static final double IN_PER_TICK = BASE_CPR / WHEEL_D_IN;

        // Track width and wheelbase distance (in inch)
        // See https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.
        // For https://www.gobilda.com/strafer-chassis-kit-96mm-mecanum-wheels/ kit,
        // track width should be [PLACEHOLDER] and wheelbase distance should be 336mm
        public static final double TRACK_WIDTH = 420.8*MM_TO_INCH;
        public static final double WHEELBASE_DISTANCE = 336*MM_TO_INCH;

        // Lateral multipler
        // factor that multiplies strafe velocity to compensate for slip;
        // increase it to boost the distance traveled in the strafe direction
        public static final double LATERAL_MULTIPLER = 1;
    }

    /**
     * Joystick axis sensitivity
     */
    public static class SENSITIVITY {
        // Joystick sensitivity
        public static final double SENSE_X = 0.15;
        public static final double SENSE_Y = 0.15;
        public static final double SENSE_Z = 0.15;
    }
}
