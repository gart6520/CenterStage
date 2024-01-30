package org.firstinspires.ftc.team24751;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

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

    /**
     * Enable bulk read for all hubs
     * @param hardwareMap hardwareMap object from opMode
     */
    public static void enableBulkRead(HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /**
     * Apply sense to input
     * @param input input value
     * @param threshold sense threshold
     * @return input if abs(input) > threshold, else 0
     */
    public static double sense(double input, double threshold) {
        if (Math.abs(input) >= threshold) {
            return input;
        } else {
            return 0;
        }
    }
}
