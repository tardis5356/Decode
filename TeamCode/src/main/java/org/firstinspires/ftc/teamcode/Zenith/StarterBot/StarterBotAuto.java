package org.firstinspires.ftc.teamcode.Zenith.StarterBot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Zenith.Auto.MecanumDrive;

@Config
@Autonomous(name="StarterBotAuto", group = "StartBotAuto")
public class StarterBotAuto extends LinearOpMode {
    private DcMotor mFL = null;
    private DcMotor mFR = null;
    private DcMotor mBL = null;
    private DcMotor mBR = null;

    private DcMotorSimple mS = null;
    private CRServo sL = null;
    private CRServo sR = null;
    private ElapsedTime runtime = new ElapsedTime();


    public class Launcher{
            private DcMotorSimple launcher;
            public Launcher(HardwareMap hardwareMap){
            mS = hardwareMap.get(DcMotorSimple.class, "mS");

        }}

            public class LeftFeeder{
                private DcMotorSimple leftFeeder;
                public LeftFeeder(HardwareMap hardwareMap){
                    sL = hardwareMap.get(CRServo.class, "sL");

                }}

                public class RightFeeder{
                    private DcMotorSimple rightFeeder;
                    public RightFeeder(HardwareMap hardwareMap){
                        sR = hardwareMap.get(CRServo.class, "sR");

                    }
                }

    private ElapsedTime time_since_start;

        @Override
    public void runOpMode(){

        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");
        mS = hardwareMap.get(DcMotorSimple.class, "mS");
        Pose2d initialPose = new Pose2d(60,0,Math.toRadians(0));
            Launcher launcher = new Launcher (hardwareMap);
            LeftFeeder leftFeeder = new LeftFeeder(hardwareMap);
            RightFeeder rightFeeder = new RightFeeder(hardwareMap);

            sL.setDirection(CRServo.Direction.REVERSE);
            mFL.setDirection(DcMotor.Direction.REVERSE);
            mBL.setDirection(DcMotor.Direction.REVERSE);

            time_since_start = new ElapsedTime();
            runtime.reset();


            waitForStart();
            while (opModeIsActive()) {
                // do nothing, just wait
                if(runtime.seconds() < 3) {


                    mFL.setPower(0.75);
                    mBL.setPower(0.75);
                    mFR.setPower(0.75);
                    mBR.setPower(0.75);

                } else{

                mFL.setPower(0);
                mBL.setPower(0);
                mFR.setPower(0);
                mBR.setPower(0);

                sL.setPower(0.5);
                sR.setPower(0.5);
                mS.setPower(1.0);



            }}

// stop motors








        }
    }
