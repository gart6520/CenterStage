package org.firstinspires.ftc.team24751;

import android.annotation.SuppressLint;

import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Utils.WPILibMotionProfile;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PwmControl;

import java.util.ArrayList;

public class Constants {
    // Some general constants
    public static final double INCH_TO_MM = 25.4;
    public static final double MM_TO_INCH = 1 / INCH_TO_MM;
    public static final double M_TO_INCH = MM_TO_INCH * 1000;

    //Really important constants
    public enum AllianceColor {
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
        public static final String EXTENDER_MOTOR = "extendMotor";
        public static final String LIFT_MOTOR = "liftMotor";


        //Odo pod encoder (could be the same name as other motors)
        public static final String LEFT_ODO = RIGHT_FRONT;
        public static final String RIGHT_ODO = RIGHT_ARM_MOTOR;
        public static final String FRONT_ODO = RIGHT_BACK;

        // Hand servos
        public static final String LEFT_WRIST = "leftWristServo";
        public static final String RIGHT_WRIST = "rightWristServo";
        public static final String LEFT_CLAW = "leftClawServo";
        public static final String RIGHT_CLAW = "rightClawServo";
        public static final String DRONE_LAUNCHER = "droneLauncherServo";

        // Active intake servo
        public static final String ACTIVE_INTAKE = "activeIntakeServo";
        // Misc servo
        public static final String CAMERA_SERVO = "cameraServo";

        // Sensors
        public static final String DISTANCE_SENSOR = "distanceSensor";
        public static final String POTENTIOMETER = "potentiometer";

        // Camera
        public static final String BACK_CAMERA_NAME = "backCamera"; // fieldCamera
        public static final String FRONT_CAMERA_NAME = "frontCamera"; // randomization camera
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
     * Sensitivity / Threshold
     */
    public static class GAMEPAD_SENSITIVITY {
        // Joystick sensitivity
        public static final double SENSE_X = 0.15;
        public static final double SENSE_Y = 0.15;
        public static final double SENSE_Z = 0.15;

        // Analog buttons
        public static final double SENSE_TRIGGER = 0.25;
    }

    /**
     * Field related parameters
     */
    public static class FIELD_PARAMETER {
        public static void initFieldParameters() {
            // TODO: Add Appropriate April Tag ID
            if (!init) {
                BIG_APRIL_TAG_ID.add(7);
                BIG_APRIL_TAG_ID.add(10);
                init = true;

                // Add low speed coors
                //LOW_SPEED_COORS.add(new ArrayList<Double>())
            }
        }

        private static boolean init = false;
        public static final ArrayList<Integer> BIG_APRIL_TAG_ID = new ArrayList<>();

        public static final ArrayList<ArrayList<Double>> LOW_SPEED_COORS = new ArrayList<>();
    }

    /**
     * Constant used for general hardware
     */
    public static class HARDWARE_CONSTANT {
        @Config
        public static class Arm {
            public static final double MOTOR_POSITION_AT_UPWARD_VERTICAL = 266.8; // tick
            public static final double MOTOR_POSITION_AT_FRONT_HORIZONTAL = 0; // tick

            /*
             *                      / <- arm
             *                    / arm angle
             *                  /-------- 0 deg
             *              |         |
             *  base -> O-------------------------O  <- front
             */
            //TODO: tune/calculate
            public static double ARM_PARALLEL_ANGLE = 130;
            public static final double MOTOR_DEG_PER_TICK = 90.0 / (MOTOR_POSITION_AT_UPWARD_VERTICAL - MOTOR_POSITION_AT_FRONT_HORIZONTAL);
            public static final double MOTOR_DEG_AT_ZERO_TICK = -MOTOR_POSITION_AT_FRONT_HORIZONTAL * MOTOR_DEG_PER_TICK;
            public static double ARM_ANGLE_MIN_PID_POW = 0;
            public static PIDCoefficientsEx ARM_ANGLE_PID_COEFFICIENTS = new PIDCoefficientsEx(
                    0.0975, 5, 0.01,
                    40, 100, 0.1);
            public static PIDCoefficientsEx ARM_DISTANCE_PID_COEFFICIENTS = new PIDCoefficientsEx(
                    0.05, .05, 0.02,
                    2, 5, 0.1);
            public static FeedforwardCoefficientsEx ARM_VELOCITY_FEEDFORWARD_COEFFICIENTS = new FeedforwardCoefficientsEx(
                    0.005, 0, 0,
                    0, 0.05);
            public static WPILibMotionProfile.Constraints ARM_VA_CONSTRAINT =
                    new WPILibMotionProfile.Constraints(
                            200, 150);
            public static final double ANGLE_TOLERANCE = 1;
            public static final double DISTANCE_TOLERANCE = 1;

            /**
             * Distance reported by distance sensor when arm is // ground
             */
            public static double DISTANCE_TO_GROUND_THRESHOLD = 2;

        }

        //Constant for wrist and grabber
        @Config
        public static class Hand {
            public static final double INIT_WRIST_SERVO_ANGLE_DEG = 10;

            /* Parallel to the ground
             * ========\
             *           \====
             * */
            public static double GROUND_PARALLEL_DEG = 192;

            /* Fully backward and touch arm (min/max angle)
             *      ====\
             * ===========\
             * */
            public static double FULL_EXTEND_DEG = 60;
            /*                 //||
             *               //  ||
             *             //beta||
             *           //
             *   alpha // (arm)
             *  =====================
             *  beta = BACKDROP_PARALLEL_DEG - alpha
             * */
            //TODO Tune backdrop (lesser mean more far from arm)
            public static double BACKDROP_PARALLEL_DEG = 117;
        }

        public static class Extender {
            public static final double EXTENDER_FULLY_IN_THRESHOLD = 50;
        }

        public static class DroneLauncher {
            public static final double LOAD_DRONE_LAUNCHER_POSITION = 0;
            public static final double SHOOT_DRONE_LAUNCHER_POSITION = 0.45;


        }

        public static class POTENTIOMETER {
            @SuppressLint("SdCardPath")
            public static final String LUT_DATA_FILE_NAME = "/sdcard/FIRST/Potentiometer_LUT_data.txt";
        }

        @Config
        public static class GENERAL_SERVO {
            public static final PwmControl.PwmRange REV_SERVO_PWM_RANGE = new PwmControl.PwmRange(500, 2500);
            //TODO: Tune for gobilda one
            public static PwmControl.PwmRange GOBILDA_SERVO_PWM_RANGE = new PwmControl.PwmRange(450, 2450);
            public static final double REV_SERVO_ANGLE_RANGE = 270;
            public static final double GOBILDA_SERVO_ANGLE_RANGE = 300;
            public static final double SERVO_ANGLE_PWM_THRESHOLD = 1.0 / 30;
            public static final double SERVO_PWM_SPEED = 0.1;
        }

    }

    /**
     * Constant used for Vision
     */
    public static class VISION {

        public static class APRIL_TAG {

            // AprilTag marginDecision threshold
            //TODO: tune
            public static final double MARGIN_DECISION_THRESHOLD = 0.1;
            public static final double INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE_DEG = 0;
        }

        public static class TFOD {
            public static final String TFOD_TEAM_PROP_MODEL_FILE = "TeamPropModelFile.tflite";
            public static final String[] TFOD_TEAM_PROP_LABELS = {
                    "BlueTeamProp",
                    "RedTeamProp",
            };
            //TFOD Team prop position threshold
            // |    Left     |     Center    |      Right      |
            // min       left-center     center-right         max
            public static final double TEAM_PROP_LEFT_CENTER = 100;
            public static final double TEAM_PROP_CENTER_RIGHT = 200;
        }
    }

    /**
     * Bot physical parameters
     */
    public static class BOT_PARAMETERS {
        public static final Vector2d robotToCamera = new Vector2d(-6, 5.5);
        public static final double INITIAL_BOT_ANGLE_DEG_BLUE = -90;
        //TODO: Change based on starting location
        public static final double INITIAL_BOT_ANGLE_DEG_TEST = 0;
        public static final double INITIAL_BOT_ANGLE_DEG_RED = 90;
    }
}
