package org.firstinspires.ftc.team24751;

import android.annotation.SuppressLint;

import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Utils.WPILibMotionProfile;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import android.util.Size;

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
        public static final String CLIMBER_MOTOR = "liftMotor";


        //Encoders (could be the same name as other motors)
        public static final String LEFT_ODO = RIGHT_FRONT;
        public static final String RIGHT_ODO = RIGHT_ARM_MOTOR;
        public static final String FRONT_ODO = RIGHT_BACK;
        public static final String LEFT_ARM_ENCODER = LEFT_ARM_MOTOR;
        public static final String RIGHT_ARM_ENCODER = LEFT_FRONT;
        public static final String EXTENDER_ENCODER = EXTENDER_MOTOR;

        // Hand servos
        public static final String LEFT_WRIST = "leftWristServo";
        public static final String RIGHT_WRIST = "rightWristServo";
        public static final String LEFT_CLAW = "leftClawServo";
        public static final String RIGHT_CLAW = "rightClawServo";
        public static final String DRONE_LAUNCHER = "droneLauncherServo";
        public static final String YELLOW_PIXEL_YEETER = "yellowPixelYeeterServo";
        public static final String CLIMBER_HOLDER = "climberHolderServo";

        // Misc servo
        public static final String CAMERA_SERVO = "cameraServo";

        // Sensors
        public static final String DISTANCE_SENSOR = "distanceSensor";

        // Camera
        public static final String BACK_CAMERA_NAME = "backCamera"; // fieldCamera
        public static final String FRONT_CAMERA_NAME = "frontCamera"; // randomization camera
        // WIP

        // LEDs
        public static final String LED_RED_LEFT = "leftRedLed";
        public static final String LED_GREEN_LEFT = "rightGreenLed";
        public static final String LED_RED_RIGHT = "rightRedLed";
        public static final String LED_GREEN_RIGHT = "rightGreenLed";
        public static final String LED_RED_WRIST = "wristRedLed";
        public static final String LED_GREEN_WRIST = "wristGreenLed";
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
            if (init) return;
            init = true;

            AUTO_AIM_APRIL_TAG_IDS.add(7);
            AUTO_AIM_APRIL_TAG_IDS.add(10);
            AUTO_AIM_APRIL_TAG_IDS.add(5);
            AUTO_AIM_APRIL_TAG_IDS.add(2);

            // Add low speed coors
            LOW_SPEED_COORDS.add(new Rect(24, -60, 72, -12.5));
            LOW_SPEED_COORDS.add(new Rect(24, 12.5, 72, 60));

        }

        private static boolean init = false;
        public static final ArrayList<Integer> AUTO_AIM_APRIL_TAG_IDS = new ArrayList<>();

        public static class Rect {
            public double x1 = 0, y1 = 0, x2 = 0, y2 = 0;

            public Rect() {
            }

            public Rect(double xMin, double yMin, double xMax, double yMax) {
                x1 = xMin;
                y1 = yMin;
                x2 = xMax;
                y2 = yMax;
            }

            public boolean isInside(Vector2d point) {
                double x = point.getX();
                double y = point.getY();
                return x >= x1 && x <= x2 && y >= y1 && y <= y2;
            }
        }

        public static final ArrayList<Rect> LOW_SPEED_COORDS = new ArrayList<>();
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
            public static final double MOTOR_DEG_PER_TICK = 90.0 / (MOTOR_POSITION_AT_UPWARD_VERTICAL - MOTOR_POSITION_AT_FRONT_HORIZONTAL);
            public static final double MOTOR_DEG_AT_ZERO_TICK = -MOTOR_POSITION_AT_FRONT_HORIZONTAL * MOTOR_DEG_PER_TICK;
            public static double ARM_BACKDROP_PARALLEL_ANGLE = 139;
            public static double ARM_BACKDROP_PARALLEL_ANGLE_AUTO = 130;
            /*
             *                       grabber
             *     arm        ======= || |++++++++|  grabber only
             *        ========        || |++++++++|  intake top two
             * =======                   |--------|  pixels while
             *                           |-pixels-|  leaving the other
             *                           |--------|  3 untouched
             * */
            public static double ARM_AUTO_INTAKING_ANGLE = 20;
            public static double ARM_ANGLE_MIN_PID_POW = 0;
            public static PIDCoefficientsEx ARM_OUTAKE_PID_COEFFICIENTS = new PIDCoefficientsEx(
                    0.0975, 5, 0.01,
                    40, 100, 0.1);
            public static PIDCoefficientsEx ARM_AUTO_INTAKE_PID_COEFFICIENTS = new PIDCoefficientsEx(
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
            public static final double INIT_WRIST_SERVO_ANGLE_DEG = 0;

            /* Parallel to the ground
             * ========\
             *           \====
             * */
            public static double WRIST_GROUND_PARALLEL_DEG = 180;

            /* Fully backward and touch arm (min/max angle)
             *      ====\
             * ===========\
             * */
            public static double WRIST_FULL_BACKWARD_DEG = 30;
            /*                 //||
             *               //  ||
             *             //beta||
             *           //
             *   alpha // (arm)
             *  =====================
             *  beta = BACKDROP_PARALLEL_DEG - alpha
             * */
            //TODO Tune backdrop (lesser mean more far from arm)
            public static double WRIST_BACKDROP_PARALLEL_DEG = 105;
            /*
             *                       grabber
             *     arm        ======= || |++++++++|  grabber only
             *        ========        || |++++++++|  intake top two
             * =======                   |--------|  pixels while
             *                           |-pixels-|  leaving the other
             *                           |--------|  3 untouched
             * */
            public static double WRIST_AUTO_INTAKING_DEG = 210;

            public static double OPEN_CLAW_POSITION = 0.25;
            public static double CLOSE_CLAW_POSITION = 0.011111111111111112;
        }

        public static class Extender {
            public static final double EXTENDER_FULLY_IN_THRESHOLD = 20;
        }

        public static class DroneLauncher {
            public static final double LOAD_DRONE_LAUNCHER_POSITION = 0.1;
            public static final double SHOOT_DRONE_LAUNCHER_POSITION = 0.45;

        }
        @Config
        public static class ClimberHolder
        {
            public static double HOLD_CLIMBER_HOLDER_POSITION = 0;
            public static double RELEASE_CLIMBER_HOLDER_POSITION = 1;
        }

        @Config
        public static class YellowPixelYeeter {
            public static double LOAD_YELLOW_PIXEL_YEETER_POSITION = 1;
            public static double YEET_YELLOW_PIXEL_YEETER_POSITION = 0.5; // 0.5

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
        public static final Size FRONT_CAMERA_RESOLUTION = new Size(320, 240);
        public static final Size BACK_CAMERA_RESOLUTION = new Size(640, 480);

        public static class APRIL_TAG {

            // AprilTag marginDecision threshold
            //TODO: tune
            public static final double MARGIN_DECISION_THRESHOLD = 0.1;
            public static final double INITIAL_AUTO_LOCK_APRIL_TAG_SERVO_ANGLE_DEG = 0;

        }

        @Config
        @SuppressLint("SdCardPath")
        public static class CV {
            // Team prop position
            public enum TeamPropPosition {
                NONE, LEFT, CENTER, RIGHT
            }

            // Color threshold

            //Area threshold
            public static double TEAM_PROP_AREA_THRESHOLD = 550; //px^2
            public static double TEAM_PROP_NAN_COUNT_THRESHOLD = 15; //px^2

            // Red team prop
            public static final Scalar TEAM_PROP_RED_MIN = new Scalar(0, 115, 133);
            public static final Scalar TEAM_PROP_RED_MAX = new Scalar(5, 228, 255);

            // Blue team prop
            public static final Scalar TEAM_PROP_BLUE_MIN = new Scalar(90, 68, 75);
            public static final Scalar TEAM_PROP_BLUE_MAX = new Scalar(108, 246, 255);

            // Team prop position threshold
            // |    Left     |     Center    |      Right      |
            // min       left-center     center-right         max
            public static double TEAM_PROP_LEFT_CENTER = 5;
            public static double TEAM_PROP_CENTER_RIGHT = 264;
        }
    }

    /**
     * Bot physical parameters
     */
    public static class BOT_PARAMETERS {
        public static final Vector2d ROBOT_TO_CAMERA = new Vector2d(-6, 5.5);
    }

    public static class AUTONOMOUS {
        // TODO tune/measure these number
        public static final Mat.Tuple3<Double> LEFT_SPIKE_MARK = new Mat.Tuple3<>(18.0, -3.0, Math.toRadians(52));
        public static final Mat.Tuple3<Double> CENTER_SPIKE_MARK = new Mat.Tuple3<>(22.0, 0.0, Math.toRadians(0));
        public static final Mat.Tuple3<Double> RIGHT_SPIKE_MARK = new Mat.Tuple3<>(18.0, 0.0, Math.toRadians(-47));
        // TODO: DON'T FUCKING CHANGE INIT POSE OR I WILL PLAY SKIBIDI TOILET FOR AN HOUR WHILE TRAPPING YOU IN EP MEETING WITH PROF DUNG
        public static final Pose2d WING_RED_START_POSE = new Pose2d(-32.09375, -63.46875, Math.toRadians(90));
        public static final Pose2d WING_BLUE_START_POSE = new Pose2d(-39.90625, 63.46875, Math.toRadians(-90));
        public static final Pose2d BACKDROP_RED_START_POSE = new Pose2d(15.90625, -63.46875, Math.toRadians(90));
        public static final Pose2d BACKDROP_BLUE_START_POSE = new Pose2d(8.09375, 63.46875, Math.toRadians(-90));
        public static final Vector2d LEFT_BACKDROP = new Vector2d(0, 6.5);
        public static final Vector2d CENTER_BACKDROP = new Vector2d(0, 0);
        public static final Vector2d RIGHT_BACKDROP = new Vector2d(0, -6.5);

    }
}
