package org.firstinspires.ftc.team24751;

import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PwmControl;

import java.util.ArrayList;

public class Constants {
    // Some general constants
    public static final double INCH_TO_MM = 25.4;
    public static final double MM_TO_INCH = 1 / INCH_TO_MM;
    public static final double M_TO_INCH = MM_TO_INCH * 1000;

    //Really important constants
    public enum AllianceColor
    {
        //Test for code usage in testing
        RED, BLUE, TEST
    }
    //TODO: All AutoOpMode should init this variable to the correct color
    public static AllianceColor allianceColor = AllianceColor.TEST;


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

        // Arm motors
        public static final String LEFT_ARM_MOTOR = "leftArmMotor";
        public static final String RIGHT_ARM_MOTOR = "rightArmMotor";
        public static final String ELEVATOR_MOTOR = "elevatorMotor";

        // Hand servos
        public static final String LEFT_WRIST = "leftWristServo";
        public static final String RIGHT_WRIST = "rightWristServo";
        public static final String LEFT_CLAW = "leftClawServo";
        public static final String RIGHT_CLAW = "rightClawServo";

        // Active intake servo
        public static final String ACTIVE_INTAKE_SERVO = "activeIntakeServo";

        // Sensors
        public static final String IMU_NAME = "imu";

        // Camera
        public static final String FIELD_CAMERA_NAME = "fieldCamera";
        public static final String FOOT_CAMERA_NAME = "footCamera";
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
        public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
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
        //public static final double BASE_ENCODER_CPP = 4;

        // Count per revolution
        //public static final double BASE_CPR = BASE_ENCODER_PPR * BASE_ENCODER_CPP;

        // Wheel diameter (in inch)
        // For goBilda's mecanum wheel, the diameter is 96mm
        public static final double WHEEL_D_IN = 96 * MM_TO_INCH;

        // Inch traveled per tick/count
        // This can either be calculated using the formula, or by measuring
        // The choice is yours
        public static final double IN_PER_TICK = WHEEL_D_IN*Math.PI / BASE_ENCODER_PPR;
        public static final double TICK_PER_IN = 1 / IN_PER_TICK;

        // Track width and wheelbase distance (in inch)
        // See https://upload.wikimedia.org/wikipedia/commons/5/52/Wheelbase_and_Track.
        // For https://www.gobilda.com/strafer-chassis-kit-96mm-mecanum-wheels/ kit,
        // track width should be [PLACEHOLDER] and wheelbase distance should be 336mm
        public static final double TRACK_WIDTH = 380 * MM_TO_INCH;
        public static final double WHEELBASE_DISTANCE = 340 * MM_TO_INCH;

        // Lateral multipler
        // factor that multiplies strafe velocity to compensate for slip;
        // increase it to boost the distance traveled in the strafe direction
        public static final double LATERAL_MULTIPLER = 1;

        // Gain constants
        public static final double AXIAL_GAIN = 30;
        public static final double LATERAL_GAIN = 20;
        public static final double HEADING_GAIN = Math.PI / 2;
        public static final double AXIAL_VEL_GAIN = 20;
        public static final double LATERAL_VEL_GAIN = 50;
        public static final double HEADING_VEL_GAIN = Math.PI / 2;

        // Constrain
        public static final double MAX_WHEEL_VEL = 40;
        public static final double MIN_PROFILE_ACCEL = -40;
        public static final double MAX_PROFILE_ACCEL = 40;

        // Turn profile parameters (in radians)
        public static final double MAX_ANG_VEL = Math.PI; // shared with path
        public static final double MAX_ANG_ACCEL = Math.PI;

        // Drivebase motor feedforward constants
        // TODO: Tune this to work in real life
        public static final double kS = 0.02;
        public static final double kV = 0.0049;
        public static final double kA = 0.00003;
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
        //TODO: tune
        public static final double MARGIN_DECISION_THRESHOLD = 0.1;
        //TFOD Team prop position threshold
        // |    Left     |     Center    |      Right      |
        // min       left-center     center-right         max
        public static final double TEAM_PROP_LEFT_CENTER = 100;
        public static final double TEAM_PROP_CENTER_RIGHT = 200;

        public static final double SERVO_ANGLE_PWM_THRESHOLD = 1.0/30;
        public static final double SERVO_PWM_SPEED = 0.1;
    }

    /**
     * Field related parameters
     */
    public static class FIELD_PARAMETER {
        public static void INIT_FIELD_PARAMETER() {
            // TODO: Add Appropriate April Tag ID
            if (!init)
            {
                BIG_APRIL_TAG_ID.add(7);
                BIG_APRIL_TAG_ID.add(10);
                init = true;
            }
        }
        private static boolean init = false;
        public static final ArrayList<Integer> BIG_APRIL_TAG_ID = new ArrayList<>();
    }

    /**
     * Constant used for general hardware
     */
    public static class HARDWARE_CONSTANT {
        public static class Arm {


            /*
             *                      / <- arm
             *                    / arm angle
             *                  /-------- 0 deg
             *              |         |
             *  base -> O-------------------------O  <- front
             */
            //TODO: tune/calculate
            public static final double MOTOR_POSITION_AT_FRONT_HORIZONTAL = 0; //tick
            public static final double MOTOR_POSITION_AT_UPWARD_VERTICAL = 181; //tick
            public static final double MOTOR_DEG_PER_TICK = 90.0 / (MOTOR_POSITION_AT_UPWARD_VERTICAL - MOTOR_POSITION_AT_FRONT_HORIZONTAL);
            public static final double MOTOR_DEG_AT_ZERO_TICK = -MOTOR_POSITION_AT_FRONT_HORIZONTAL * MOTOR_DEG_PER_TICK;
            public static final PIDCoefficientsEx ARM_POSITION_PID_COEFFICIENTS = new PIDCoefficientsEx(0.1, 0, 0,
                    0.5, 30, 0.9);
            public static final FeedforwardCoefficientsEx ARM_VELOCITY_FEEDFORWARD_COEFFICIENTS = new FeedforwardCoefficientsEx(
                    0.1, 0.1, 0,
                    0, 0.1);
            public static final double POSITION_THRESHOLD = 1;

        }

        //Constant for wrist and grabber
        public static class Hand
        {
            public static final double INIT_WRIST_SERVO_ANGLE_DEG = 0;
        }
        //TODO: tune
        public static final double MOTOR_POSITION_AT_ZERO = 0;

        public static final double MOTOR_POSITION_AT_PERPENDICULAR = 90;

        public static final double SERVO_POSITION_AT_ZERO = 0;

        public static final double SERVO_POSITION_AT_PERPENDICULAR = 90;
        public static final double INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE_DEG = 60;
        public static final PwmControl.PwmRange REV_SERVO_PWM_RANGE = new PwmControl.PwmRange(550, 2450);
        //TODO: Tune for gobilda one
        public static final PwmControl.PwmRange GOBILDA_SERVO_PWM_RANGE = new PwmControl.PwmRange(550, 2450);
        public static final double REV_SERVO_ANGLE_RANGE = 270;
        public static final double GOBILDA_SERVO_ANGLE_RANGE = 300;
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
    /**
     * Bot physical parameters
     * */
    public static class BOT_PARAMETERS
    {
        public static final Vector2d robotToCamera = new Vector2d(-1,2);
        public static final double INITIAL_BOT_ANGLE_DEG_BLUE = -90;
        //TODO: Change based on starting location
        public static final double INITIAL_BOT_ANGLE_DEG_TEST = 0;
        public static final double INITIAL_BOT_ANGLE_DEG_RED = 90;
    }

    public static class OdometryPod
    {
        public static final double WHEEL_DIAMETER_IN = 48 * MM_TO_INCH;
        public static final double TICKS_PER_REV = 2000;

        public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

        public static double inPerTick = (WHEEL_DIAMETER_IN * Math.PI) / (TICKS_PER_REV * GEAR_RATIO);
    }
}
