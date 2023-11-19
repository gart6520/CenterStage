package com.example.meepmeeptesting;

public class Constants {
    // Some general constants
    public static final double INCH_TO_MM = 25.4;
    public static final double MM_TO_INCH = 1/INCH_TO_MM;

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
        public static final double TRACK_WIDTH = 413.2*MM_TO_INCH;
        public static final double WHEELBASE_DISTANCE = 336*MM_TO_INCH;

        // Lateral multipler
        // factor that multiplies strafe velocity to compensate for slip;
        // increase it to boost the distance traveled in the strafe direction
        public static final double LATERAL_MULTIPLER = 1;

        // Gain constants
        public static final double AXIAL_GAIN = 300*MM_TO_INCH;
        public static final double LATERAL_GAIN = 300*MM_TO_INCH;
        public static final double HEADING_GAIN = Math.PI/2;
        public static final double AXIAL_VEL_GAIN = 150*MM_TO_INCH;
        public static final double LATERAL_VEL_GAIN = 150*MM_TO_INCH;
        public static final double HEADING_VEL_GAIN = Math.PI/2;

        // Constrain
        public static final double MAX_WHEEL_VEL = 300*MM_TO_INCH;
        public static final double MIN_PROFILE_ACCEL = -150*MM_TO_INCH;
        public static final double MAX_PROFILE_ACCEL = 150*MM_TO_INCH;

        // turn profile parameters (in radians)
        public static final double MAX_ANG_VEL = Math.PI; // shared with path
        public static final double MAX_ANG_ACCEL = Math.PI;
    }
}
