package Optimus.Optimus.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper extends SubsystemBase {

    private Servo sG, sW;

    public Gripper(HardwareMap hardwareMap){

        sG = hardwareMap.get(Servo.class,"sG");
        sW = hardwareMap.get(Servo.class,"sW");

    }

    @Override

    public void periodic(){



    }

    public void open(){

        sG.setPosition(OptimusBotPositions.GRIPPER_OPEN);

    }

    public void close(){

        sG.setPosition(OptimusBotPositions.GRIPPER_CLOSE);

    }

    public void up(){

        sW.setPosition(OptimusBotPositions.GRIPPER_UP);

    }
    public void down(){

        sW.setPosition(OptimusBotPositions.GRIPPER_DOWN);

    }
    public void middle(){

        sW.setPosition(OptimusBotPositions.GRIPPER_MIDDLE);

    }
}
