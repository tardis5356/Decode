package org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto;

import static org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto.DecodeAuto.MAX_CYCLES;
import static org.firstinspires.ftc.teamcode.Zenith.Auto.PenfieldAuto.DecodeAuto.gateCycleIndex;
import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables.aColor;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Zenith.Auto.MecanumDrive;

import java.util.Arrays;

public class AutoTrajectories {

    // Key poses (will be mirrored when alliance is blue)
    public static Pose2d goalStartPos, audienceStartPos;
    public static Pose2d goalShootPos, audienceShootPos;

    public static Pose2d goalIntakePos, midIntakePos, audienceIntakePos, cornerIntakePos;
    public static Pose2d presetLZIntakePos, randomLZIntakePos;
    public static Pose2d gateReleasePos;
    public static Pose2d gateReadyToReleasePos;
    public static Pose2d parkPos;

    // Actions: for each cycle index we create startToIntake and intakeToShoot actions
//    public static Action[] startToIntakeWaypoint = new Action[MAX_CYCLES];
//    public static Action[] intakeWaypointToIntake = new Action[MAX_CYCLES];
    public static Action[] startToIntake = new Action[MAX_CYCLES];
    public static Action[] intakeToShoot = new Action[MAX_CYCLES];
    public static Action gateRelease, gateExit;

    public static Action goalStartToGoalShoot;

    public static Action park;


    // Intake tangents: [goal, mid, audience, LZ preset, LZ random]
    public static double[] intakeEndTangentDeg = {90, 90, 90, 90 /*check*/, 90 /*45*/};

    // Shoot tangents: [goal, audience]
    public static double[] shootStartTangentDeg = {270, 270}; // start tangent when approaching shoot
    public static double[] shootEndTangentDeg = {180, 0 }; // final heading tangent at shoot

    // Gate release tangents [goal, audience]
    public static double[] gateReleaseStartTangentDeg = {0, 180}; // start tangent when approaching gate
    public static double gateReleaseEndTangentDeg = 270; // final heading tangent at gate


    // [goalStart, audienceStart]
    public static double[] startPosTangentDeg = {0, 180};

    // [goalShoot, audienceShoot]
    public static double[] intakeStartTangentDeg = {90, 180};

    public static MinVelConstraint BaseConstraint = new MinVelConstraint(
            Arrays.asList(
                    new TranslationalVelConstraint(55),//inches per second
                    new AngularVelConstraint(Math.PI) // remember the units you're working in, especially for angular constraints!
            )
    );

    public static MinVelConstraint SlowConstraint = new MinVelConstraint(
            Arrays.asList(
                    new TranslationalVelConstraint(30),//inches per second
                    new AngularVelConstraint(Math.PI) // remember the units you're working in, especially for angular constraints!
            )
    );



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

    public static double allianceValue(double value) {
        if ("blue".equals(aColor)) {
            return -value;
        }
        return value;
    }

    // Populate key poses (call when alliance color chosen)
    public static void updateAlliancePoses() {
        audienceStartPos = allianceCoordinate(new Pose2d(62.75, 24, Math.toRadians(90)));
        goalStartPos = allianceCoordinate(new Pose2d(-49, 53, Math.toRadians(38)));
        goalIntakePos = allianceCoordinate(new Pose2d(-12, 53, Math.toRadians(90)));
        midIntakePos = allianceCoordinate(new Pose2d(13, 55, Math.toRadians(90)));
        audienceIntakePos = allianceCoordinate(new Pose2d(33, 50, Math.toRadians(90)));
        goalShootPos = allianceCoordinate(new Pose2d(0, 12, Math.toRadians(90)));
//        goalShootPos = allianceCoordinate(new Pose2d(-29, 8, Math.toRadians(90)));
        audienceShootPos = allianceCoordinate(new Pose2d(48, 10, Math.toRadians(90)));
        gateReleasePos = allianceCoordinate(new Pose2d(0, 49, Math.toRadians(0)));
        gateReadyToReleasePos = allianceCoordinate(new Pose2d(0, 38, Math.toRadians(90)));
        presetLZIntakePos = allianceCoordinate(new Pose2d(58, 57, 90));
        randomLZIntakePos = allianceCoordinate(new Pose2d(58, 57, 45));

        parkPos = allianceCoordinate(new Pose2d(30, -30, 180));


    }

    /**
     * Build actions for every cycle:
     * - startToIntakeActions[i] : from currentStart -> intake (uses intakeStartTangentDeg[IntakeChoice] and intakeEndTangentDeg[intakeChoice])
     * - intakeToShootActions[i] : from intake -> shoot (uses shootStartTangentDeg[shootChoice] and shootEndTangentDeg[shootChoice])
     * final gateAction uses gateReleaseStart/End tangents.
     *
     * @param drive    MecanumDrive instance
     * @param choices  choices[cycleIndex][0=shootChoice,1=intakeChoice]
     * @param cycles   number of cycles to build (1..3)
     * @param startPos starting pose for cycle 1
     */
    public static void generateTrajectories(MecanumDrive drive, int[][] choices, int cycles, Pose2d startPos) {
        Pose2d[] shootPositions = {goalShootPos, audienceShootPos};
        Pose2d[] intakePositions = {goalIntakePos, midIntakePos, audienceIntakePos, presetLZIntakePos, randomLZIntakePos};

        Pose2d currentStart = (startPos != null) ? startPos : goalStartPos;

        int shootChoice = 0;
        for (int i = 0; i < cycles; i++) {
            shootChoice = choices[i][0];
            int intakeChoice = choices[i][1];

            Pose2d intakePose = intakePositions[intakeChoice];
            Pose2d shootPose = shootPositions[shootChoice];

            // === intake START TANGENT LOGIC ===
            double intakeStartDeg;
            if (i == 0) {
                // first cycle: based on start position
                intakeStartDeg = currentStart.equals(goalStartPos) ? startPosTangentDeg[0] : startPosTangentDeg[1];


            }
//            else if (i - 1 == gateCycleIndex) {
//                intakeStartDeg = allianceTangent(270);
//                currentStart = gateExitWaypointPos;
//            }
            else {
                // later cycles: based on previous shoot position
                intakeStartDeg = intakeStartTangentDeg[choices[i - 1][0]];
            }

            if (startPos == goalStartPos && i == 0){
                goalStartToGoalShoot = drive.actionBuilder(currentStart)
                        .strafeToLinearHeading(new Vector2d(-12, allianceValue(5)), allianceTangent(90))
                        .build();
                currentStart = new Pose2d(new Vector2d(-12, allianceValue(5)), allianceTangent(90));
            }




            // End tangent for intake
            double intakeEndRad = allianceTangent(intakeEndTangentDeg[intakeChoice]);
            double intakeStartRad = allianceTangent(intakeStartDeg);

            // === START → INTAKE POS ===
//            startToIntakeWaypoint[i] = drive.actionBuilder(currentStart)
//                    .strafeToLinearHeading(new Vector2d(intakePose.position.x, allianceValue(33)), allianceTangent(90))
//                    .build();
//
//            intakeWaypointToIntake[i] = drive.actionBuilder(new Pose2d(new Vector2d(intakePose.position.x, allianceValue(33)), allianceTangent(90)))
//                    .splineToLinearHeading(intakePose, intakeEndRad, BaseConstraint)
//                    .build();

            startToIntake[i] = drive.actionBuilder(currentStart)
                    .setTangent(intakeStartRad)
                    .splineToLinearHeading(intakePose, intakeEndRad)
                    .build();

            // === INTAKE POS → SHOOT ===
            double shootStartRad = allianceTangent(shootStartTangentDeg[shootChoice]);
            double shootEndRad = allianceTangent(shootEndTangentDeg[shootChoice]);

            intakeToShoot[i] = drive.actionBuilder(intakePose)
                    .setTangent(shootStartRad)
                    .splineToLinearHeading(shootPose, shootEndRad)
                    .build();

            // Next cycle starts from the shoot pose
            currentStart = shootPose;


        }

//        int gateShootChoice = (gateCycleIndex < cycles) ? choices[gateCycleIndex][0] : choices[cycles - 1][0];   // last cycle shoot option
////
//        double gateStartRad = allianceTangent(gateReleaseStartTangentDeg[gateShootChoice]);
//        double gateEndRad = allianceTangent(gateReleaseEndTangentDeg);
////
//        gateRelease = drive.actionBuilder(shootPositions[choices[gateCycleIndex][0]]).setTangent(gateStartRad).splineToLinearHeading(gateReleasePos, gateEndRad).build();
//
//        gateExit = drive.actionBuilder(gateReleasePos).setTangent(allianceTangent(270)).splineToLinearHeading(gateExitWaypointPos, allianceTangent(90)).build();

        park = drive.actionBuilder(drive.localizer.getPose()).splineToLinearHeading(parkPos, allianceTangent(0)).build();


    }

}
