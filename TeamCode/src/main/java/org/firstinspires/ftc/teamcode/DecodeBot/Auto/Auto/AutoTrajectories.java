package org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto;

import static org.firstinspires.ftc.teamcode.DecodeBot.Auto.Auto.DecodeAuto.startPos;
import static org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.GlobalVariables.aColor;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.DecodeBot.Auto.MecanumDrive;

public class AutoTrajectories {

    //heading input is degrees
    static Pose2d mirroredCoordinate;

    public static Pose2d allianceCoordinate(Pose2d coordinate) {
        if ((aColor.equals("blue"))) {
            mirroredCoordinate = new Pose2d(coordinate.position.x, coordinate.position.y * -1, Math.toRadians(-coordinate.heading.toDouble()));
            return mirroredCoordinate;
        } else {

            return new Pose2d(coordinate.position.x, coordinate.position.y, Math.toRadians(coordinate.heading.toDouble()));
        }

    }

    private static double allianceTangent(double tangent) {

        double mirroredTangent;

        if ((aColor.equals("blue"))) {
            mirroredTangent = Math.toRadians(-tangent);
            return mirroredTangent;
        } else {
            return Math.toRadians(tangent);
        }

    }


    public static Pose2d shootPos1;

    public static double shootEndTangent1;
    public static double shootStartTangent1;
    public static double shootEndTangent2;
    public static double shootStartTangent2;

    public static double nextStartTangent1;

    public static double nextStartTangent2;

    public static double backShootStartTangent = 270;
    public static double backShootEndTangent =270;
    public static double nextBackStartTangent = 180;


    public static double frontShootStartTangent = 270;
    public static double frontShootEndTangent = 245;
    public static double nextFrontStartTangent = 0;

    public static Pose2d shootPos2;
    public static Pose2d shootPos3;
    //Actions
    public static Action backStartToBackSpike;

    public static Action originTestToStartPos;

    public static Action backSpikeToShoot1;
    public static Action shootPos1ToMidSpike;
    public static Action midSpikeToShoot2;
    //    public static Action MidSpikeToGate;
    public static Action frontSpikeIntake;
    public static Action midSpikeToFrontSpike;
    public static Action frontSpikeToGate;
    public static Action shootPos2ToGate;
    public static Action frontStartToFrontSpike;
    public static Action shootMidSpike;
    public static Action shootFrontSpike;
    private static double startPosTangent;
    public static  Pose2d backStartPos;
    public static  Pose2d frontStartPos;
    public static Pose2d frontSpikePos;
    public static  Pose2d midSpikePos;

    public static Pose2d backSpikePos;
    public static Pose2d cornerPickupPos;
    public static Pose2d gateReleasePos;
    public static Pose2d backShootPos;
    public static Pose2d frontShootPos;

    public static Pose2d originTestPoint = new Pose2d(0,0,0);


    public static void updateAlliancePoses() {
     backStartPos = allianceCoordinate(new Pose2d(-62, 24, 90));
    frontStartPos = allianceCoordinate(new Pose2d(-54, 47, 305));
    frontSpikePos = allianceCoordinate(new Pose2d(-12, 43, 90)); //PPG
     midSpikePos = allianceCoordinate(new Pose2d(12, 43, 90)); //PGP
     backSpikePos = allianceCoordinate(new Pose2d(-35, 43, 90)); //GPP
   cornerPickupPos = allianceCoordinate(new Pose2d(48, 60, 90)); //pick up corner PGP
   gateReleasePos = allianceCoordinate(new Pose2d(0, 52, 90)); //open the gate
    backShootPos = allianceCoordinate(new Pose2d(48, 10, 90)); //PPG
   frontShootPos = allianceCoordinate(new Pose2d(-12, 17, 90)); //PPG
}
    public static void generateTrajectories(MecanumDrive drive, int shootChoice1, int shootChoice2) {

        switch (shootChoice1) {
            case 0 :
                shootPos1 = frontShootPos;
                shootStartTangent1 = frontShootStartTangent;
                shootEndTangent1 = frontShootEndTangent;
                nextStartTangent1 = nextFrontStartTangent;
                break;
            case 1 :
                shootPos1 = backShootPos;
                shootStartTangent1 = backShootStartTangent;
                shootEndTangent1 = backShootEndTangent;
                nextStartTangent1 = nextBackStartTangent;
                break;
        }

        switch (shootChoice2) {
            case 0 :
                shootPos2 = frontShootPos;
                shootStartTangent2 = frontShootStartTangent;
                shootEndTangent2 = frontShootEndTangent;
                nextStartTangent2 = nextFrontStartTangent;
                break;
            case 1 :
                shootPos2 = backShootPos;
                shootStartTangent2 = backShootStartTangent;
                shootEndTangent2 = backShootEndTangent;
                nextStartTangent2 = nextBackStartTangent;
                break;
        }
        originTestToStartPos =
        drive.actionBuilder(originTestPoint)
                .splineToLinearHeading(backStartPos, allianceTangent(90))
                .build();

        backStartToBackSpike =
                drive.actionBuilder(backStartPos)
                        .setTangent(allianceTangent(180))
                        .splineToLinearHeading(backSpikePos, allianceTangent(90))
                        .build();


        backSpikeToShoot1 =
                drive.actionBuilder(backSpikePos)
                        .setTangent(allianceTangent(shootStartTangent1))
                        .splineToLinearHeading(shootPos1, allianceTangent(shootEndTangent1))
                        .build();

        shootPos1ToMidSpike =
                drive.actionBuilder(shootPos1)
                        .setTangent(allianceTangent(nextStartTangent1))
                        .splineToLinearHeading(midSpikePos, allianceTangent(90))
                        .build();

        midSpikeToShoot2 =
                drive.actionBuilder(midSpikePos)
                        .setTangent(allianceTangent(shootStartTangent2))
                        .splineToLinearHeading(shootPos2, allianceTangent(shootEndTangent2))
                        .build();

        shootPos2ToGate =
                drive.actionBuilder(shootPos2)
                        .setTangent(allianceTangent(nextStartTangent2))
                        .splineToLinearHeading(gateReleasePos, allianceTangent(90))
                        .build();







//        frontStartToFrontSpike =
//                drive.actionBuilder(startPos)
//                        .setTangent(allianceTangent(180))
//                        .splineToLinearHeading(backSpikePos, allianceTangent(90))
//                        .build();
//
//        backSpikeToShoot1 =
//                drive.actionBuilder(backSpikePos)
//                        .setTangent(allianceTangent(0))
//                        .splineToLinearHeading(backShootPos, allianceTangent(270))
//                        .build();
//
//        shootPos1ToMidSpike =
//                drive.actionBuilder(backShootPos)
//                        .setTangent(allianceTangent(180))
//                        .splineToLinearHeading(midSpikePos, allianceTangent(90))
//                        .build();
//
//        midSpikeToShoot2 =
//                drive.actionBuilder(midSpikePos)
//                        .setTangent(allianceTangent(225))
//                        .splineToLinearHeading(frontShootPos, allianceTangent(245))
//                        .build();
//
//        shootPos2ToGate =
//                drive.actionBuilder(midSpikePos)
//                        .setTangent(allianceTangent(0))
//                        .splineToLinearHeading(gateReleasePos, allianceTangent(90))
//                        .build();


    }
}
