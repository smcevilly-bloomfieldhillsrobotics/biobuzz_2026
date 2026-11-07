package org.firstinspires.ftc.teamcode

import com.pedropathing.api.Paths
import com.pedropathing.api.PoseFactory
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path

/**
 * Defines clean, active 90° orthogonal (axis-aligned) paths for FTC BioBuzz Autonomous routines.
 * All unused legacy poses and curves have been purged.
 */
class PedroPaths {

    private val poseFactory = PoseFactory.degrees()

    val centerHiveStartPose: Pose = poseFactory.of(72.0, 9.0, 90.0)
    val centerHiveIntermediatePose: Pose = poseFactory.of(72.0, 120.0, 90.0)
    val centerHiveFinalPose: Pose = poseFactory.of(53.0, 120.0, 270.0)
    val centerHiveIntermediatePath: Path get() = Paths.line(centerHiveStartPose, centerHiveIntermediatePose).constant(Math.toRadians(90.0))
    val centerHiveFinalPath: Path get() = Paths.line(centerHiveIntermediatePose, centerHiveFinalPose).linear(Math.toRadians(90.0), Math.toRadians(270.0))

    val startPose: Pose = poseFactory.of(56.0, 8.0, 90.0)
    val shootingPositionPose: Pose = poseFactory.of(56.0, 36.0, 90.0)
    val shootingPositionPath: Path get() = Paths.line(startPose, shootingPositionPose).constant(Math.toRadians(90.0))
    val reloadingStartPath: Path get() = Paths.line(shootingPositionPose, poseFactory.of(34.8, 11.0, 90.0)).constant(Math.toRadians(90.0))
    val loadGardenPath: Path get() = Paths.line(poseFactory.of(34.8, 11.0, 90.0), poseFactory.of(11.0, 14.2, 90.0)).constant(Math.toRadians(90.0))
    val oppositeShootingPositionPath: Path get() = Paths.line(poseFactory.of(11.0, 14.2, 90.0), poseFactory.of(52.8, 126.9, 270.0)).linear(Math.toRadians(90.0), Math.toRadians(270.0))

    // =========================================================================
    // Red Alliance Active Starting Poses & Dest Poses
    // =========================================================================
    // altStartPose: Red alt start, bottom wall against glass.
    val altStartPose: Pose = poseFactory.of(39.0, 9.0, 90.0) // Flat against wall
    val altIntermediateTarget: Pose = poseFactory.of(39.0, 12.0, 75.0) // Pulled off wall, angled 15° to hive (90° -> 75°)

    val aprilTagStartPose: Pose = poseFactory.of(63.0, 9.0, 90.0)
    val aprilTagIntermediatePose: Pose = poseFactory.of(24.0, 120.0, 90.0)
    val aprilTagFinalPose: Pose = poseFactory.of(55.0, 125.0, 270.0)
    val parkEndPose: Pose = poseFactory.of(12.0, 94.0, 90.0) // Red Park (Mirrored: X=12.0", Y=94.0")

    val redTopAltStartPose: Pose = poseFactory.of(39.0, 128.0, 278.0)
    val redTopStdStartPose: Pose = poseFactory.of(55.0, 135.0, 270.0)

    // Red Trajectories (100% Linear & Orthogonal)
    fun aprilTagIntermediatePaths(): List<Path> {
        val p1 = poseFactory.of(24.0, aprilTagStartPose.y(), 90.0)
        return listOf(
            Paths.line(aprilTagStartPose, p1).constant(Math.toRadians(90.0)),
            Paths.line(p1, aprilTagIntermediatePose).constant(Math.toRadians(90.0))
        )
    }

    fun aprilTagFinalPaths(): List<Path> {
        val p1 = poseFactory.of(24.0, aprilTagFinalPose.y(), 90.0)
        return listOf(
            Paths.line(aprilTagIntermediatePose, p1).constant(Math.toRadians(90.0)),
            Paths.line(p1, aprilTagFinalPose).linear(Math.toRadians(90.0), Math.toRadians(270.0))
        )
    }

    fun redTopToBottomPaths(): List<Path> {
        val p1 = poseFactory.of(55.0, aprilTagStartPose.y(), 270.0)
        return listOf(
            Paths.line(aprilTagFinalPose, p1).constant(Math.toRadians(270.0)),
            Paths.line(p1, aprilTagStartPose).linear(Math.toRadians(270.0), Math.toRadians(90.0))
        )
    }

    fun aprilTagStartPaths(): List<Path> {
        val p1 = poseFactory.of(24.0, aprilTagStartPose.y(), 90.0)
        return listOf(
            Paths.line(aprilTagIntermediatePose, p1).constant(Math.toRadians(90.0)),
            Paths.line(p1, aprilTagStartPose).constant(Math.toRadians(90.0))
        )
    }

    val altIntermediatePose: Pose = poseFactory.of(24.0, 120.0, 90.0)
    val altIntermediatePath: Path get() = Paths.line(altStartPose, altIntermediatePose).constant(Math.toRadians(90.0))
    val altFinalPath: Path get() = Paths.line(altIntermediatePose, aprilTagFinalPose).constant(Math.toRadians(270.0))
    
    // Create an explicit 3-step orthogonal bypass for the Angled Start (Tag Lost) Traverse
    fun redAltToOppositePaths(): List<Path> {
        val p1 = poseFactory.of(24.0, altIntermediateTarget.y(), 90.0) 
        val p2 = poseFactory.of(24.0, aprilTagFinalPose.y(), 90.0)
        return listOf(
            Paths.line(altIntermediateTarget, p1).linear(Math.toRadians(75.0), Math.toRadians(90.0)),
            Paths.line(p1, p2).constant(Math.toRadians(90.0)),
            Paths.line(p2, aprilTagFinalPose).linear(Math.toRadians(90.0), Math.toRadians(270.0))
        )
    }

    fun redAltInitialAlignPaths(): Path {
        return Paths.line(altStartPose, altIntermediateTarget).linear(Math.toRadians(90.0), Math.toRadians(75.0))
    }

    // =========================================================================
    // Blue Alliance Active Starting Poses & Dest Poses
    // =========================================================================
    // blueAltStartPose: Blue alt start, bottom wall against glass.
    val blueAltStartPose: Pose = poseFactory.of(105.0, 9.0, 90.0) // Flat against wall
    val blueAltIntermediateTarget: Pose = poseFactory.of(105.0, 12.0, 105.0) // Pulled off wall, angled 15° to hive (90° -> 105°)

    val blueAprilTagStartPose: Pose = poseFactory.of(89.0, 135.0, 270.0)
    val blueAprilTagIntermediatePose: Pose = poseFactory.of(89.0, 120.0, 270.0)
    val blueAprilTagFinalPose: Pose = poseFactory.of(81.0, 9.0, 90.0)
    val blueParkEndPose: Pose = poseFactory.of(132.0, 50.0, 270.0) // Blue Park (X=132.0", Y=50.0", 270°)

    val blueTopStdStartPose: Pose = poseFactory.of(89.0, 135.0, 270.0)
    val blueTopAltStartPose: Pose = poseFactory.of(105.0, 128.0, 262.0)

    // Blue Trajectories (100% Linear & Orthogonal)
    fun blueAprilTagIntermediatePaths(): List<Path> {
        val p1 = poseFactory.of(blueAprilTagStartPose.x(), blueAprilTagIntermediatePose.y(), 270.0)
        return listOf(
            Paths.line(blueAprilTagStartPose, p1).constant(Math.toRadians(270.0)),
            Paths.line(p1, blueAprilTagIntermediatePose).constant(Math.toRadians(270.0))
        )
    }

    fun blueAprilTagFinalPaths(): List<Path> {
        val p1 = poseFactory.of(blueAprilTagIntermediatePose.x(), blueAprilTagFinalPose.y(), 270.0)
        return listOf(
            Paths.line(blueAprilTagIntermediatePose, p1).constant(Math.toRadians(270.0)),
            Paths.line(p1, blueAprilTagFinalPose).linear(Math.toRadians(270.0), Math.toRadians(90.0))
        )
    }

    fun blueTopToBottomPaths(): List<Path> {
        val p1 = poseFactory.of(blueAprilTagFinalPose.x(), blueAprilTagStartPose.y(), 90.0)
        return listOf(
            Paths.line(blueAprilTagFinalPose, p1).constant(Math.toRadians(90.0)),
            Paths.line(p1, blueAprilTagStartPose).linear(Math.toRadians(90.0), Math.toRadians(270.0))
        )
    }

    fun blueAprilTagStartPaths(): List<Path> {
        val p1 = poseFactory.of(blueAprilTagIntermediatePose.x(), blueAprilTagStartPose.y(), 90.0)
        return listOf(
            Paths.line(blueAprilTagIntermediatePose, p1).constant(Math.toRadians(90.0)),
            Paths.line(p1, blueAprilTagStartPose).constant(Math.toRadians(270.0))
        )
    }

    val blueAltIntermediatePose: Pose = poseFactory.of(120.0, 120.0, 90.0)
    val blueAltIntermediatePath: Path get() = Paths.line(blueAltStartPose, blueAltIntermediatePose).constant(Math.toRadians(90.0))
    val blueAltFinalPath: Path get() = Paths.line(blueAltIntermediatePose, blueAprilTagFinalPose).constant(Math.toRadians(270.0))

    fun blueAltToOppositePaths(): List<Path> {
        val p1 = poseFactory.of(120.0, blueAltIntermediateTarget.y(), 90.0) 
        val p2 = poseFactory.of(120.0, blueAprilTagStartPose.y(), 90.0)
        return listOf(
            Paths.line(blueAltIntermediateTarget, p1).linear(Math.toRadians(105.0), Math.toRadians(90.0)),
            Paths.line(p1, p2).constant(Math.toRadians(90.0)),
            Paths.line(p2, blueAprilTagStartPose).linear(Math.toRadians(90.0), Math.toRadians(270.0))
        )
    }

    fun blueAltInitialAlignPaths(): Path {
        return Paths.line(blueAltStartPose, blueAltIntermediateTarget).linear(Math.toRadians(90.0), Math.toRadians(105.0))
    }

    // =========================================================================
    // Park Auto Specific Linear Segments (Used by ParkAutoOpMode)
    // =========================================================================
    val parkStartPose: Pose = poseFactory.of(63.0, 9.0, 90.0)
    val parkPose1: Pose = poseFactory.of(28.0, 9.0, 90.0)
    val parkPose2: Pose = poseFactory.of(28.0, 96.0, 90.0)

    val parkPathSegment1: Path get() = Paths.line(parkStartPose, parkPose1).constant(Math.toRadians(90.0))
    val parkPathSegment2: Path get() = Paths.line(parkPose1, parkPose2).constant(Math.toRadians(90.0))
    val parkPathSegment3: Path get() = Paths.line(parkPose2, parkEndPose).constant(Math.toRadians(90.0))

    // Pure Orthogonal 3-Step Line Commands for Parking (100% Axis-Aligned, Zero Diagonal!)
    fun createOrthogonalParkPath(fromPose: Pose, isBlueAlliance: Boolean = false): List<Path> {
        return if (isBlueAlliance) {
            val p1 = poseFactory.of(120.0, fromPose.y(), 270.0)
            val p2 = poseFactory.of(120.0, 50.0, 270.0)
            listOf(
                // Step 1: Strafe East along X-axis to Column 5/6 Tile Seam (X=120.0") keeping 270°
                Paths.line(fromPose, p1).constant(Math.toRadians(270.0)),
                // Step 2: Drive to Blue Park Y level (Y=50.0") keeping 270°
                Paths.line(p1, p2).constant(Math.toRadians(270.0)),
                // Step 3: Slide East along X-axis into Blue Park (132.0, 50.0) keeping 270°
                Paths.line(p2, blueParkEndPose).constant(Math.toRadians(270.0))
            )
        } else {
            val headingRad = fromPose.heading()
            val targetY = if (fromPose.y() > 100.0) 94.0 else fromPose.y()
            val p1 = poseFactory.of(24.0, targetY, Math.toDegrees(headingRad))
            val p2 = poseFactory.of(24.0, 94.0, Math.toDegrees(headingRad))
            val finalParkPose = poseFactory.of(12.0, 94.0, Math.toDegrees(headingRad))
            listOf(
                // Step 1: Pure West Strafe along X-axis to Column 1/2 Tile Seam (X=24.0") maintaining current heading
                Paths.line(fromPose, p1).constant(headingRad),
                // Step 2: Pure Drive along Y-axis to Red Park (Y=94.0") maintaining current heading
                Paths.line(p1, p2).constant(headingRad),
                // Step 3: Pure West Slide along X-axis into Red Park (12.0, 94.0) maintaining current heading
                Paths.line(p2, finalParkPose).constant(headingRad)
            )
        }
    }
}
