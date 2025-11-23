package org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto;

import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.DecodeAuto.MAX_CYCLES;
import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.DecodeAuto.gateCycleIndex;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables.aColor;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;

public class AutoTrajectories {

    // Key poses (will be mirrored when alliance is blue)
    public static Pose2d goalStartPos, audienceStartPos;
    public static Pose2d goalShootPos, audienceShootPos;
    public static Pose2d goalSpikePos, midSpikePos, audienceSpikePos, gateReleasePos;
    public static Pose2d gateExitWaypointPos;


    // Actions: for each cycle index we create startToSpike and spikeToShoot actions
    public static Action[] startToSpike = new Action[MAX_CYCLES];
    public static Action[] spikeToShoot = new Action[MAX_CYCLES];
    public static Action gateRelease, gateExit;


    // Spike tangents: [goal, mid, audience]
    public static double[] spikeEndTangentDeg = {90, 90, 90};

    // Shoot tangents: [goal, audience]
    public static double[] shootStartTangentDeg = {270, 270}; // start tangent when approaching shoot
    public static double[] shootEndTangentDeg = {225, 0}; // final heading tangent at shoot

    // Gate release tangents [goal, audience]
    public static double[] gateReleaseStartTangentDeg = {0, 180};
    public static double gateReleaseEndTangentDeg = 270;


    public static double[] startPosTangentDeg = {245, 180}; // [goalStart, audienceStart]

    public static double[] spikeStartTangentDeg = {0, 180};    // [goalShoot, audienceShoot]


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
        audienceStartPos = allianceCoordinate(new Pose2d(62.75, 24, Math.toRadians(90)));
        goalStartPos = allianceCoordinate(new Pose2d(-48, 52, Math.toRadians(308)));
        goalSpikePos = allianceCoordinate(new Pose2d(-12, 45, Math.toRadians(90)));
        midSpikePos = allianceCoordinate(new Pose2d(12, 45, Math.toRadians(90)));
        audienceSpikePos = allianceCoordinate(new Pose2d(35, 45, Math.toRadians(90)));
        goalShootPos = allianceCoordinate(new Pose2d(-12, 17, Math.toRadians(90)));
        audienceShootPos = allianceCoordinate(new Pose2d(48, 10, Math.toRadians(90)));
        gateReleasePos = allianceCoordinate(new Pose2d(0, 46, Math.toRadians(90)));
        gateExitWaypointPos = allianceCoordinate(new Pose2d(0, 56, Math.toRadians(90)));
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
        Pose2d[] shootPositions = {goalShootPos, audienceShootPos};
        Pose2d[] spikePositions = {goalSpikePos, midSpikePos, audienceSpikePos};

        Pose2d currentStart = (startPos != null) ? startPos : goalStartPos;

        int shootChoice = 0;
        for (int i = 0; i < cycles; i++) {
            shootChoice = choices[i][0];
            int spikeChoice = choices[i][1];

            Pose2d spikePose = spikePositions[spikeChoice];
            Pose2d shootPose = shootPositions[shootChoice];

            // === SPIKE START TANGENT LOGIC ===
            double spikeStartDeg;
            if (i == 0) {
                // first cycle: based on start position
                spikeStartDeg = currentStart.equals(goalStartPos)
                        ? startPosTangentDeg[0]
                        : startPosTangentDeg[1];
            }
            else if (i - 1 == gateCycleIndex) {
                spikeStartDeg = allianceTangent(270);
                currentStart = gateExitWaypointPos;
            }
            else {
                // later cycles: based on previous shoot position
                spikeStartDeg = spikeStartTangentDeg[choices[i - 1][0]];
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

                double gateStartRad = allianceTangent(gateReleaseStartTangentDeg[choices[gateCycleIndex][0]]);
                double gateEndRad = allianceTangent(gateReleaseEndTangentDeg);

                gateRelease = drive.actionBuilder(shootPositions[choices[gateCycleIndex][0]])
                        .setTangent(gateStartRad)
                        .splineToLinearHeading(gateReleasePos, gateEndRad)
                        .build();

                gateExit = drive.actionBuilder(gateReleasePos)
                        .setTangent(allianceTangent(270))
                        .splineToLinearHeading(gateExitWaypointPos, allianceTangent(270))
                        .build();



        }



    }

}
