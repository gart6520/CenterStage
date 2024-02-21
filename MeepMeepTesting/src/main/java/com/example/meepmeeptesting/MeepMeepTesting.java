package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import static com.example.meepmeeptesting.Constants.*;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(640);

        Pose2d initPoseToPurplePose = new Pose2d(-3.90625, -25.46875, Math.toRadians(30));
        Pose2d initPose = new Pose2d(15.90625, 63.46875, Math.toRadians(-90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(initPose)
                                .forward(2)
                                .lineToLinearHeading(initPose.plus(initPoseToPurplePose))
                                .lineToLinearHeading(new Pose2d(50.65, 28.00, Math.toRadians(180.00)))
                                .lineToConstantHeading(new Vector2d(30.00, 9.50))
                                .lineToConstantHeading(new Vector2d(-50.00, 9.50))
                                .lineToConstantHeading(new Vector2d(-55.00, 9.50))
                                .setConstraints(
                                        new MecanumVelocityConstraint(56, Constants.TRACK_WIDTH),
                                        new ProfileAccelerationConstraint(100))
                                .lineToConstantHeading(new Vector2d(-50.00, 9.50))
                                .resetConstraints()
                                .lineToConstantHeading(new Vector2d(30.00, 9.50))
                                .lineToConstantHeading(new Vector2d(50.65, 36.00))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
