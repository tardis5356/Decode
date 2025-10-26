package com.example.meepmeeptesting;

import static com.acmerobotics.roadrunner.geometry.Pose2dKt.times;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;



public class MeepMeepTesting {
static String aColor = "blue";

    public static final Pose2d BackStartPos = allianceCoordinate(new Pose2d(63,33,180));
    public static final Pose2d FrontStartPos = allianceCoordinate(new Pose2d(-36,33,270));
    public static final Pose2d FrontSpikePos = allianceCoordinate(new Pose2d(-12,31.5,90)); //PPG
    public static final Pose2d MidSpikePos = allianceCoordinate(new Pose2d(12,31.5,90)); //PGP
    public static final Pose2d BackSpikePos = allianceCoordinate(new Pose2d(36,31.5,90)); //GPP
    public static final Pose2d CornerPickupPos = allianceCoordinate(new Pose2d(60,60,90)); //pick up corner PGP
    public static final Pose2d GatePrepPos = allianceCoordinate(new Pose2d(0,48,180)); //go to gate position
    public static final Pose2d GateReleasePos = allianceCoordinate(new Pose2d(0,56,180)); //open the gate

    static Pose2d mirroredCoordinate;
    private static Pose2d allianceCoordinate(Pose2d coordinate) {
        if(aColor == "blue"){
            mirroredCoordinate =new Pose2d(new Vector2d(coordinate.getX(), coordinate.getY() * -1) , Math.toRadians(Math.toDegrees(-coordinate.getHeading()) ));
            return mirroredCoordinate;
        }else {
        return coordinate;
        }

    }


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14, 17) // width, height in inches (or your field unit)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
               .followTrajectorySequence(driveShim ->
                       driveShim.trajectorySequenceBuilder(allianceCoordinate(BackStartPos))
                               .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(BackSpikePos, Math.toRadians(90))//fill in tangent
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(CornerPickupPos, Math.toRadians(90))
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(MidSpikePos, Math.toRadians(90))
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(GatePrepPos, Math.toRadians(90))
                                .waitSeconds(100000)
                               .build());


        Image img = null;
       // try { img = ImageIO.read(new File("C:\\Users\\trant\\Downloads\\field-2025-official.png")); }
        try { img = ImageIO.read(new File("C:\\Users\\Icy\\Downloads\\field-2025-official.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setAxesInterval(10)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}