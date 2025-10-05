package com.example.meepmeeptesting;

import static com.acmerobotics.roadrunner.geometry.Pose2dKt.times;

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




    public static final Pose2d bigStartPos = new Pose2d(56, -47, Math.toRadians(125));
    public static final Pose2d smallStartPos = new Pose2d(-62, -29, Math.toRadians(0));
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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(allianceCoordinate(bigStartPos))
                        .waitSeconds(100000)
                        .build());


        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\trant\\Downloads\\field-2025-official.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setAxesInterval(10)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}