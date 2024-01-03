package org.firstinspires.ftc.team24751.subsystems.old_roadrunner

import org.firstinspires.ftc.team24751.subsystems.old_roadrunner.geometry.Pose2d

/**
 * Generic interface for estimating robot pose over time.
 */
interface Localizer {

    /**
     * Current robot pose estimate.
     */
    var poseEstimate: Pose2d

    /**
     * Current robot pose velocity (optional)
     */
    val poseVelocity: Pose2d?

    /**
     * Completes a single localization update.
     */
    fun update()
}