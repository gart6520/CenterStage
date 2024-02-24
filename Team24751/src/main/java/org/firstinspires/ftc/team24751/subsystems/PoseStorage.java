package org.firstinspires.ftc.team24751.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Objects;
import java.util.Scanner;

public class PoseStorage {
    // This variable should be preserved between opmodes
    // The init value is just 0. I tried to just use new Pose2d() but it causes error
    public static Pose2d pose = new Pose2d(new Vector2d(0, 0), 0);
    @SuppressLint("SdCardPath")
    String fileName = "/sdcard/FIRST/pose.txt";
//    static FileWriter fileWriter;
//    static Scanner fileReader;

    PoseStorage() {
//        try {
//            File file = new File(fileName);
//            file.createNewFile();
//            fileWriter = new FileWriter(file);
//            fileReader = new Scanner(file);
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }
    }

    /**
     * Store the pose
     * You can directly access the pose variable, but this method is recommended
     * in case of changing pose saving mechanism.
     *
     * @param pose Pose2d object to save
     */
    public static void setPose(Pose2d pose) {
        PoseStorage.pose = pose;
//        try {
//            fileWriter.write(pose.getX() + " " + pose.getY() + " " + pose.getHeading());
//            fileWriter.close();
//        } catch (IOException ignored) {
//        }
    }

    /**
     * Get the pose
     * You can directly access the pose variable, but this method is recommended
     * in case of changing pose saving mechanism.
     *
     * @return stored Pose2d object
     */
    public static Pose2d getPose() {
        return pose;
    }
}
