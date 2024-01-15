package org.firstinspires.ftc.team24751.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class PoseStorage {
    // This variable should be preserved between opmodes
    // The init value is just 0. I tried to just use new Pose2d() but it causes error
    public static Pose2d pose = new Pose2d(new Vector2d(0, 0), 0);

    /**
     * Store the pose
     * You can directly access the pose variable, but this method is recommended
     * in case of changing pose saving mechanism.
     * @param pose Pose2d object to save
     */
    public static void setPose(Pose2d pose) {
        PoseStorage.pose = pose;
    }

    /**
     * Get the pose
     * You can directly access the pose variable, but this method is recommended
     * in case of chaning pose saving mechanism.
     * @return stored Pose2d object
     */
    public static Pose2d getPose() {
        return PoseStorage.pose;
    }
}
