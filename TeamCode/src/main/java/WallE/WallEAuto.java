package WallE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Wall-E_Auto")
public class WallEAuto extends LinearOpMode {
    DcMotor mL, mR;
    ColorSensor sL, sR;
    double rotate;

    @Override
    public void runOpMode(){
        mL = hardwareMap.get(DcMotor.class,"mL");
        mR = hardwareMap.get(DcMotor.class,"mR");

        sL = hardwareMap.get(ColorSensor.class,"sL");
        sR = hardwareMap.get(ColorSensor.class,"sR");

        waitForStart();
        while(opModeIsActive()){
            mL.setPower(rotate-.1);
            mR.setPower(-rotate-.1);

            if(sL.red() < 800 && sR.red()>800){
                rotate = 0;
            }
            else if(sR.red() <= 800){
                rotate = -.07;
            }
            else if(sL.red() >= 800){
                rotate = .07;
            }

            telemetry.addData("sL_red",sL.red());
            telemetry.addData("sR_red",sR.red());
            telemetry.update();
        }
    }
}
