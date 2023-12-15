package org.firstinspires.ftc.team24751;

/**
 * Utilities functions, wrapper, etc...
 */
public class Utility {
    public enum WRAP_ANGLE_TYPE {
        zeroTo360,
        minus180To180,
        zeroTo2Pi,
        minusPiToPi
    }

    /**
     * Wrap angle to be wrapped with different range
     */
    public static double wrapAngle(double angle, WRAP_ANGLE_TYPE type) {
        switch (type) {
            default:
            case zeroTo360:
                while (angle >= 360) angle -= 360;
                while (angle < 0) angle += 360;
                return angle;
            case minus180To180:
                while (angle >= 180) angle -= 360;
                while (angle < -180) angle += 360;
                return angle;
            case zeroTo2Pi:
                while (angle >= 2 * Math.PI) angle -= Math.PI;
                while (angle < 0) angle += Math.PI;
                return angle;
            case minusPiToPi:
                while (angle >= Math.PI) angle -= Math.PI;
                while (angle < -Math.PI) angle += Math.PI;
                return angle;
        }
    }
}
