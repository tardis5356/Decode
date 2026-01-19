package WallE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "Wall-E_Tele")
public class WallETeleOp extends LinearOpMode {

    DcMotor mL, mR;
    ColorSensor sL, sR;

    @Override
    public void runOpMode(){

        mL = hardwareMap.get(DcMotor.class,"mL");
        mR = hardwareMap.get(DcMotor.class,"mR");
//        mR.setDirection(DcMotorSimple.Direction.REVERSE);

        sL = hardwareMap.get(ColorSensor.class,"sL");
        sR = hardwareMap.get(ColorSensor.class,"sR");

        waitForStart();

        while(opModeIsActive())   {
            mL.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
            mR.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
        }
    }
}
