package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import static com.example.meepmeeptesting.Constants.*;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import com.example.meepmeeptesting.Homeostasis.KalmanFilter;

public class MeepMeepTesting {


    public static void main(String[] args) throws IOException {
        KalmanFilter[] kalmanFilters = {
                new KalmanFilter(1, 0.4, 3),
                new KalmanFilter(1, 0.4, 3),
                new KalmanFilter(0.015, 0.4, 3),
        };
        for (int i = 0; i < kalmanFilters.length; i++) {
            KalmanFilter kalmanFilter = kalmanFilters[i];
            {
                try (
                        BufferedReader fileReader = new BufferedReader(new FileReader("C:\\Users\\DELL\\Downloads\\input_kalman.txt"));
                        BufferedWriter fileWriter = new BufferedWriter(new FileWriter("C:\\Users\\DELL\\Downloads\\output_kalman" + (i + 1) + ".txt"))
                ) {
                    String line;
                    boolean first = true;
                    while ((line = fileReader.readLine()) != null) {
                        // Split the line into unfiltered value and true value
                        String[] values = line.split("\\s+"); // Assuming values are separated by space
                        if (first && i != 0) {
                            kalmanFilter.setX(Double.parseDouble(values[0]));
                            first = false;
                        }
                        // Parse unfiltered and true values
                        double unfilteredValue = Double.parseDouble(values[0]);
                        double trueValue = Double.parseDouble(values[1]);

                        // Perform Kalman filtering
                        double filteredValue = kalmanFilter.estimate(unfilteredValue);

                        // Write to output file
                        double[] writeTos = {filteredValue, unfilteredValue, trueValue};
                        writeLine(fileWriter, writeTos);
                    }
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }
        }


        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(640);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.2, -62.21, Math.toRadians(90.00)))
                                .lineTo(new Vector2d(-36.2, -38.93))
                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(-57.00, -38.93))
                                .lineTo(new Vector2d(49.00, -38.93))
                                .setReversed(true)
                                .lineTo(new Vector2d(-57.00, -38.93))
                                .lineTo(new Vector2d(49.00, -38.93))
                                .setReversed(true)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    private static void writeLine(BufferedWriter writer, double[] values) throws IOException {
        for (double value : values)
            writer.write(value + " ");
        writer.newLine();
    }
}
