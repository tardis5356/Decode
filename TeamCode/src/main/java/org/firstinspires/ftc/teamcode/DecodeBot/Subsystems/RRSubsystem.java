package org.firstinspires.ftc.teamcode.DecodeBot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class RRSubsystem extends SubsystemBase{
        private DcMotorEx mFL;
        private DcMotorEx mFR;
        private DcMotorEx mBL;
        private DcMotorEx mBR;
        private MecanumDrive drive; // Your Roadrunner drive implementation



        public RRSubsystem(final HardwareMap hMap) {
            mFL = hMap.get(DcMotorEx.class, "mFL");
            mFR = hMap.get(DcMotorEx.class,"mFR");
            mBL = hMap.get(DcMotorEx.class, "mBL");
            mBR = hMap.get(DcMotorEx.class, "mBR");
            return;
        }

        /**
         * Creates a new ExampleSubsystem.
         *
         */


        @Override
        public void periodic() {
            // This method will be called once per scheduler run
        }

}
