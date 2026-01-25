package WallE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="Adafruit_BeamBreak_Test")
public class ConceptAdafruitBeamBreak extends LinearOpMode {

    DigitalChannel adaFruitBeamBreak;
    @Override
    public void runOpMode(){
        adaFruitBeamBreak = hardwareMap.get(DigitalChannel.class,"adaFruitBeamBreak");
        adaFruitBeamBreak.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();
        while(opModeIsActive()){

            telemetry.addData("sensorState", adaFruitBeamBreak.getState());
            telemetry.update();
        }

    }
}
