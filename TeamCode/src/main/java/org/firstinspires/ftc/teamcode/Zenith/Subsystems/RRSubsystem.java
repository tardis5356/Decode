package org.firstinspires.ftc.teamcode.Zenith.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class RRSubsystem extends SubsystemBase {
    private DcMotorEx mFL;
    private DcMotorEx mFR;
    private DcMotorEx mBL;
    private DcMotorEx mBR;

    //    BNO055IMU imu;
//    public static IMU imu;

    double startingErrorRads = 0;
    double startingOffsetRads = 0;

//    double correctedAngleDegrees = 0;

    public RRSubsystem(HardwareMap hardwareMap) {
//        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
//        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
//        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
//        mBR = hardwareMap.get(DcMotorEx.class, "mBR");

//        imu = hardwareMap.get(BNO055IMU.class, "imuEx");

//        imu.initialize(new BNO055IMU.Parameters());


//        imu = hardwareMap.get(IMU.class, "imu");
//
//        imu.initialize(
//                new IMU.Parameters(
//                        new RevHubOrientationOnRobot(
//                                RevHubOrientationOnRobot.LogoFacingDirection.DOWN   ,
//                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//                        )
//                )
//        );
//
//        imu.resetYaw();
    }



//    public void setStartingOffsetDegs(int offsetDeg){
//        startingOffsetRads = Math.toRadians(offsetDeg);
//    }
//
//    public double getStartingOffsetDegs(){
//        return Math.toDegrees(startingOffsetRads);
//    }
//
//    public double getRawYawDegrees(){
//        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//    }
//
    @Override
    public void periodic() {

    }

//    public double getYawDegrees(){
//        return getRawYawDegrees() + getStartingOffsetDegs();
//    }
//
//    public double getYawRadians(){
//        return Math.toRadians(getYawDegrees());
//    }
//
//    public double getPose2dYawRads(){
//        return Math.toRadians((getYawDegrees() + 360) % 360) + Math.toRadians(getStartingOffsetDegs());
//    }
//
//    public double getPose2dYawDegs(){
//        return ((getYawDegrees() + 360) % 360);
//    }






}
