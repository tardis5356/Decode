package org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto;

import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.DecodeAuto.MAX_CYCLES;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables.aColor;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;

public class AutoTrajectories {

    // Key poses (will be mirrored when alliance is blue)
    public static Pose2d frontStartPos, backStartPos;
    public static Pose2d frontShootPos, backShootPos;
    public static Pose2d frontSpikePos, midSpikePos, backSpikePos, gateReleasePos;

    // Actions: for each cycle index we create startToSpike and spikeToShoot actions
    public static Action[] startToSpike = new Action[MAX_CYCLES];
    public static Action[] spikeToShoot = new Action[MAX_CYCLES];
    public static Action gateAction;

    // Tangents in DEGREES (easy to read & edit)
    // Spike tangents: [front, mid, back]
    public static double[] spikeStartTangentDeg = {0, 180};
    public static double[] spikeEndTangentDeg   = {90, 90, 90};

    // Shoot tangents: [front, back]
    public static double[] shootStartTangentDeg = {270, 270}; // start tangent when approaching shoot
    public static double[] shootEndTangentDeg   = {135, 270}; // final heading tangent at shoot

    // Gate release tangents [front, back]
    public static double[] gateReleaseStartTangentDeg = {0 , 180};
    public static double gateReleaseEndTangentDeg   = 270;



    // Helper: mirror coordinate if alliance is blue
    public static Pose2d allianceCoordinate(Pose2d coordinate) {
        if ("blue".equals(aColor)) {
            return new Pose2d(coordinate.position.x, -coordinate.position.y, -coordinate.heading.toDouble());
        }
        return coordinate;
    }

    // Helper: returns tangent in radians and mirrors sign if blue
    public static double allianceTangent(double degrees) {
        if ("blue".equals(aColor)) {
            return Math.toRadians(-degrees);
        }
        return Math.toRadians(degrees);
    }

    // Populate key poses (call when alliance color chosen)
    public static void updateAlliancePoses() {
        backStartPos   = allianceCoordinate(new Pose2d(62, 24, Math.toRadians(90)));
        frontStartPos  = allianceCoordinate(new Pose2d(-48, 52, Math.toRadians(308)));
        frontSpikePos  = allianceCoordinate(new Pose2d(-12, 45, Math.toRadians(90)));
        midSpikePos    = allianceCoordinate(new Pose2d(12, 45, Math.toRadians(90)));
        backSpikePos   = allianceCoordinate(new Pose2d(35, 45, Math.toRadians(90)));
        frontShootPos  = allianceCoordinate(new Pose2d(-12, 17, Math.toRadians(90)));
        backShootPos   = allianceCoordinate(new Pose2d(48, 10, Math.toRadians(90)));
        gateReleasePos = allianceCoordinate(new Pose2d(0, 56, Math.toRadians(90)));
    }

    /**
     * Build actions for every cycle:
     * - startToSpikeActions[i] : from currentStart -> spike (uses spikeStartTangentDeg[spikeChoice] and spikeEndTangentDeg[spikeChoice])
     * - spikeToShootActions[i] : from spike -> shoot (uses shootStartTangentDeg[shootChoice] and shootEndTangentDeg[shootChoice])
     * final gateAction uses gateReleaseStart/End tangents.
     *
     * @param drive    MecanumDrive instance
     * @param choices  choices[cycleIndex][0=shootChoice,1=spikeChoice]
     * @param cycles   number of cycles to build (1..3)
     * @param startPos starting pose for cycle 1
     */
    public static void generateTrajectories(MecanumDrive drive, int[][] choices, int cycles, Pose2d startPos) {
        Pose2d[] shootPositions = {frontShootPos, backShootPos};
        Pose2d[] spikePositions = {frontSpikePos, midSpikePos, backSpikePos};

        Pose2d currentStart = (startPos != null) ? startPos : frontStartPos;

        int shootChoice = 0;
        for (int i = 0; i < cycles; i++) {
            shootChoice = choices[i][0];
            int spikeChoice = choices[i][1];

            Pose2d spikePose = spikePositions[spikeChoice];
            Pose2d shootPose = shootPositions[shootChoice];

            // === SPIKE START TANGENT LOGIC ===
            double spikeStartDeg;

            // ---- First cycle special case ----
            if (i == 0) {
                if (currentStart.equals(frontStartPos)) {
                    spikeStartDeg = 0;     // front start
                } else if (currentStart.equals(backStartPos)) {
                    spikeStartDeg = 245;   // back start
                } else {
                    // fallback if startPos doesn’t exactly match
                    spikeStartDeg = spikeStartTangentDeg[spikeChoice];
                }
            } else {
                // ---- Normal rule (depends on shootPos from previous cycle) ----
                if (shootPose.equals(backShootPos)) {
                    spikeStartDeg = 0;     // shooting from back → start next path forward
                } else {
                    spikeStartDeg = 180;   // shooting from front → turn around
                }
            }

            // End tangent for spike
            double spikeEndRad = allianceTangent(spikeEndTangentDeg[spikeChoice]);
            double spikeStartRad = allianceTangent(spikeStartDeg);

            // === START → SPIKE ===
            startToSpike[i] = drive.actionBuilder(currentStart)
                    .setTangent(spikeStartRad)
                    .splineToLinearHeading(spikePose, spikeEndRad)
                    .build();

            // === SPIKE → SHOOT ===
            double shootStartRad = allianceTangent(shootStartTangentDeg[shootChoice]);
            double shootEndRad = allianceTangent(shootEndTangentDeg[shootChoice]);

            spikeToShoot[i] = drive.actionBuilder(spikePose)
                    .setTangent(shootStartRad)
                    .splineToLinearHeading(shootPose, shootEndRad)
                    .build();

            // Next cycle starts from the shoot pose
            currentStart = shootPose;
        }

        // === Final Gate Release ===
        double gateStartRad = allianceTangent(gateReleaseStartTangentDeg[shootChoice]);
        double gateEndRad = allianceTangent(gateReleaseEndTangentDeg);

        gateAction = drive.actionBuilder(currentStart)
                .setTangent(gateStartRad)
                .splineToLinearHeading(gateReleasePos, gateEndRad)
                .build();
    }

}
