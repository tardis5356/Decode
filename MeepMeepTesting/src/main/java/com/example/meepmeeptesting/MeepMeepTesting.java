package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;



public class MeepMeepTesting {

    static String aColor = "blue";

    public static final Pose2d presetLZPos = allianceCoordinate(new Pose2d(58,57,45));


    public static final Pose2d parkPos = allianceCoordinate(new Pose2d(30,-30,180));

    public static final Pose2d backStartPos = allianceCoordinate(new Pose2d(63,26,90));
    public static final Pose2d frontStartPos = allianceCoordinate(new Pose2d(-54,47,305));
    public static final Pose2d frontSpikePos = allianceCoordinate(new Pose2d(-12,43,90)); //PPG
    public static final Pose2d midSpikePos = allianceCoordinate(new Pose2d(12,43,90)); //PGP
    public static final Pose2d backSpikePos = allianceCoordinate(new Pose2d(35,43,90)); //GPP
    public static final Pose2d cornerPickupPos = allianceCoordinate(new Pose2d(48,60,90)); //pick up corner PGP
    public static final Pose2d gateReleasePos = allianceCoordinate(new Pose2d(7,57,120)); //open the gate
    public static final Pose2d backShootPos = allianceCoordinate(new Pose2d(48,10,90)); //PPG
    public static final Pose2d frontShootPos = allianceCoordinate(new Pose2d(-12,17,90)); //PPG

    static Pose2d mirroredCoordinate;
    private static Pose2d allianceCoordinate(Pose2d coordinate) {
        if(aColor == "blue"){
            mirroredCoordinate =new Pose2d(coordinate.getX(), coordinate.getY() * -1 , Math.toRadians(-coordinate.getHeading()));
            return mirroredCoordinate;
        }else {

        return new Pose2d(coordinate.getX(),coordinate.getY(), Math.toRadians(coordinate.getHeading()));
        }

    }

    private static double allianceTangent(double tangent) {

        double mirroredTangent;

        if(aColor == "blue"){
            mirroredTangent = Math.toRadians(-tangent);
            return mirroredTangent;
        }else {
            return Math.toRadians(tangent);
        }

    }


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(17, 16) // width, height in inches (or your field unit)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
               .followTrajectorySequence(driveShim ->
//                       driveShim.trajectorySequenceBuilder(frontStartPos)
//                               .waitSeconds(1.5)
//                               .setTangent(allianceTangent(305))
//                               .splineToLinearHeading(frontSpikePos,allianceTangent(90))//fill in tangent
//                               .setTangent(allianceTangent(270))
//                               .splineToLinearHeading(frontShootPos, allianceTangent(270))
//                               .waitSeconds(1.5)
//
//                               .setTangent(allianceTangent(0))
//                               .splineToLinearHeading(midSpikePos, allianceTangent(90))
//                               .setTangent(allianceTangent(225))
//                               .splineToLinearHeading(frontShootPos, allianceTangent(245))
//                               .waitSeconds(1.5)
//                               .setTangent(allianceTangent(0))
//                               .splineToLinearHeading(gateReleasePos, allianceTangent(90))

                       driveShim.trajectorySequenceBuilder(gateReleasePos)
//                               .setTangent(allianceTangent(180))
//                                .splineToLinearHeading(presetLZPos,allianceTangent(45))//fill in tangent
                               .waitSeconds(10)
//                                .setTangent(allianceTangent(270))
//                                .splineToLinearHeading(backShootPos, allianceTangent(270))
//                               .waitSeconds(1.5)
////
//                               .setTangent(allianceTangent(180))
//                               .splineToLinearHeading(midSpikePos, allianceTangent(90))
//                               .setTangent(allianceTangent(225))
//                               .splineToLinearHeading(frontShootPos, allianceTangent(245))
//                               .waitSeconds(1.5)
//                               .setTangent(allianceTangent(0))
//                               .splineToLinearHeading(gateReleasePos, allianceTangent(90))
//                               .back(20)
//                               .setTangent(allianceTangent(270))
//                               .splineToLinearHeading(frontSpikePos, allianceTangent(90))
                               .build());

        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\trant\\Downloads\\field-2025-official.png")); }
       // try { img = ImageIO.read(new File("C:\\Users\\Icy\\Downloads\\field-2025-official.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setAxesInterval(10)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}