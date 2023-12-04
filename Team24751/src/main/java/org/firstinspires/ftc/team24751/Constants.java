package org.firstinspires.ftc.team24751;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import java.util.ArrayList;

public class Constants {
    // Some general constants
    public static final double INCH_TO_MM = 25.4;
    public static final double MM_TO_INCH = 1 / INCH_TO_MM;
    public static final double M_TO_INCH = MM_TO_INCH * 1000;

    //Really important constants
    public enum AllianceColor
    {
        RED, BLUE
    }
    //TODO: All AutoOpMode should init this variable to the correct color
    public static AllianceColor color = AllianceColor.BLUE;


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

        // Camera
        // WIP

        // LEDs
        public static final String LED_RED = "led_red";
        public static final String LED_GREEN = "led_green";
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
        public static final double BASE_ENCODER_PPR = ((((1 + (46.0 / 17))) * (1 + (46.0 / 11))) * 28);

        // Counts per pulse
        public static final double BASE_ENCODER_CPP = 4;

        // Count per revolution
        public static final double BASE_CPR = BASE_ENCODER_PPR * BASE_ENCODER_CPP;

        // Wheel diameter (in inch)
        // For goBilda's mecanum wheel, the diameter is 96mm
        public static final double WHEEL_D_IN = 96 * MM_TO_INCH;

        // Inch traveled per tick/count
        // This can either be calculated using the formula, or by measuring
        // The choice is yours
        public static final double IN_PER_TICK = BASE_CPR / WHEEL_D_IN;

        // Track width and wheelbase distance (in inch)
        // See https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.
        // For https://www.gobilda.com/strafer-chassis-kit-96mm-mecanum-wheels/ kit,
        // track width should be [PLACEHOLDER] and wheelbase distance should be 336mm
        public static final double TRACK_WIDTH = 413.2 * MM_TO_INCH;
        public static final double WHEELBASE_DISTANCE = 336 * MM_TO_INCH;

        // Lateral multipler
        // factor that multiplies strafe velocity to compensate for slip;
        // increase it to boost the distance traveled in the strafe direction
        public static final double LATERAL_MULTIPLER = 1;

        // Gain constants
        public static final double AXIAL_GAIN = 300 * MM_TO_INCH;
        public static final double LATERAL_GAIN = 300 * MM_TO_INCH;
        public static final double HEADING_GAIN = Math.PI / 2;
        public static final double AXIAL_VEL_GAIN = 150 * MM_TO_INCH;
        public static final double LATERAL_VEL_GAIN = 150 * MM_TO_INCH;
        public static final double HEADING_VEL_GAIN = Math.PI / 2;

        // Constrain
        public static final double MAX_WHEEL_VEL = 300 * MM_TO_INCH;
        public static final double MIN_PROFILE_ACCEL = -150 * MM_TO_INCH;
        public static final double MAX_PROFILE_ACCEL = 150 * MM_TO_INCH;

        // Turn profile parameters (in radians)
        public static final double MAX_ANG_VEL = Math.PI; // shared with path
        public static final double MAX_ANG_ACCEL = Math.PI;

        // Drivebase motor feedforward constants
        // TODO: Tune this to work in real life
        public static final double kS = 0;
        public static final double kV = 1.395;
        public static final double kA = 0.002;
    }

    /**
     * Sensitivity / Threshold
     */
    public static class SENSITIVITY {
        // Joystick sensitivity
        public static final double SENSE_X = 0.15;
        public static final double SENSE_Y = 0.15;
        public static final double SENSE_Z = 0.15;

        // AprilTag marginDecision threshold
        public static final double MARGIN_DECISION_THRESHOLD = 0.1;
        //TFOD Team prop position threshold
        // |    Left     |     Center    |      Right      |
        // min       left-center     center-right         max
        public static final double TEAM_PROP_LEFT_CENTER = 100;
        public static final double TEAM_PROP_CENTER_RIGHT = 200;
    }

    /**
     * Field related parameters
     */
    public static class FIELD_PARAMETER {
        public FIELD_PARAMETER() {
            // TODO: Add Appropriate April Tag ID
            BIG_APRIL_TAG_ID.add(1);
            BIG_APRIL_TAG_ID.add(2);
        }

        public static final ArrayList<Integer> BIG_APRIL_TAG_ID = new ArrayList<>();
    }

    /**
     * Initial values which doesn't fit above categories
     */
    public static class INIT_VALUE {
        public static final double INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE = 0;
    }

    /**
     * Constant used for TFOD
     */
    public static class TFOD {
        public static final String TFOD_TEAM_PROP_MODEL_FILE = "TeamPropModelFile.tflite";
        public static final String[] TFOD_TEAM_PROP_LABELS = {
                "BlueTeamProp",
                "RedTeamProp",
        };
    }
}
