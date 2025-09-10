package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
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
    private IMU imu;

    double startingErrorRads = 0;
    double startingOffsetRads = 0;

//    double correctedAngleDegrees = 0;

    public RRSubsystem(HardwareMap hardwareMap) {
        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");

//        imu = hardwareMap.get(BNO055IMU.class, "imuEx");

//        imu.initialize(new BNO055IMU.Parameters());


        imu = hardwareMap.get(IMU.class, "pinpoint");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.   ,
                                RevHubOrientationOnRobot.UsbFacingDirection.
                        )
                )
        );

        imu.resetYaw();
    }



    public void setStartingOffsetDegs(int offsetDeg){
        startingOffsetRads = Math.toRadians(offsetDeg);
    }

    public double getStartingOffsetDegs(){
        return Math.toDegrees(startingOffsetRads);
    }

    public double getRawYawDegrees(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    @Override
    public void periodic() {

    }

    public double getYawDegrees(){
        return getRawYawDegrees() + getStartingOffsetDegs();
    }

    public double getYawRadians(){
        return Math.toRadians(getYawDegrees());
    }

    public double getPose2dYawRads(){
        return Math.toRadians((getYawDegrees() + 360) % 360);
    }

    public double getPose2dYawDegs(){
        return ((getYawDegrees() + 360) % 360);
    }





    /*
    public BNO055IMU getImu() {
        return imu;
    }

    public Orientation getIMUOrientation(){
        return imu.getAngularOrientation();
    }

    public void setStartingError(){
        startingErrorRads = AngleUnit.normalizeDegrees(-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);
    }
    public void setStartingOffsetDegs(int offsetDeg){
        startingOffsetRads = Math.toRadians(offsetDeg);
    }

    public double getYawRadians(){
        return AngleUnit.normalizeRadians(-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle-startingErrorRads+startingOffsetRads);
    }

    public double getYawDegrees(){
        return AngleUnit.normalizeDegrees(-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle-Math.toDegrees(startingErrorRads)+Math.toDegrees(startingOffsetRads));
    }

    public double getRawYawDegrees(){
        return AngleUnit.normalizeDegrees(-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
    }*/

    // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
    // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
    // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

}
