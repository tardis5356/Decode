package org.firstinspires.ftc.teamcode.Zenith.TeleOps;

import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables.currentArtifacts;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.BellyPan;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.BrakePad;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Storage;

@TeleOp (name="Indexer")
public class IndexServos extends CommandOpMode {

    Storage storage;
    Shooter shooter;
    BrakePad brakePad;
    BellyPan bellyPan;
    Intake intake;
    GamepadEx driver;

    @Override
    public void initialize() {

        driver = new GamepadEx(gamepad1);

        storage = new Storage(hardwareMap);
        shooter = new Shooter(hardwareMap);
        brakePad = new BrakePad(hardwareMap);
        bellyPan = new BellyPan(hardwareMap);
        intake = new Intake(hardwareMap);



        shooter.sH.setPosition(0);

        new Trigger(()-> driver.getButton(GamepadKeys.Button.BACK))
                .whenActive(()->shooter.sH.setPosition(.95));


        new Trigger(()-> driver.getButton(GamepadKeys.Button.A))
                .toggleWhenActive(storage::closeGate, storage::openGate);

        new Trigger(()-> driver.getButton(GamepadKeys.Button.B))
                .toggleWhenActive(storage::lowerKicker, storage::raiseKicker);

//        new Trigger(()-> driver.getButton(GamepadKeys.Button.X))
//                .toggleWhenActive(storage::storeSlot, storage::returnSlot);
//
////        new Trigger(()-> driver.getButton(GamepadKeys.Button.Y))
//                .toggleWhenActive(storage::closeBack, storage::openBack);


        new Trigger(()->driver.getButton(GamepadKeys.Button.START))
                .toggleWhenActive(brakePad::deploy, brakePad::retract);

        new Trigger(()->driver.getButton(GamepadKeys.Button.DPAD_UP))
                .toggleWhenActive(bellyPan::engagePTO, bellyPan::disEngagePTO);
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("PTO_State", bellyPan.PTO_Engaged);
        telemetry.addData("BreakPad_State", brakePad.breakPadEngaged);
//        telemetry.addData("YolkPos",storage.sS.getPosition());
        telemetry.addData("GatePos",storage.sG.getPosition());
        telemetry.addData("KickerPos",storage.sK.getPosition());
//        telemetry.addData("BackPos",storage.sBG.getPosition());
//
//        telemetry.addData("IntakeRed", intake.bbI.red());
//        telemetry.addData("IntakeGreen", intake.bbI.green());
//        telemetry.addData("IntakeBlue", intake.bbI.blue());
//        telemetry.addData("IntakeDist", intake.bbI.getDistance(DistanceUnit.CM));


//        telemetry.addData("MiddleRed", intake.bbM.red());
//        telemetry.addData("MiddleGreen", intake.bbM.green());
//        telemetry.addData("MiddleBlue", intake.bbM.blue());
//        telemetry.addData("MiddleDist", intake.bbM.getDistance(DistanceUnit.CM));
//
//
//        telemetry.addData("ShooterRed", intake.bbSh.red());
//        telemetry.addData("ShooterGreen", intake.bbSh.green());
//        telemetry.addData("ShooterBlue", intake.bbSh.blue());
//        telemetry.addData("ShooterDist", intake.bbSh.getDistance(DistanceUnit.CM));
//
//
//        telemetry.addData("YolkRed", intake.cSSt.red());
//        telemetry.addData("YolkGreen", intake.cSSt.green());
//        telemetry.addData("YolkBlue", intake.cSSt.blue());
//        telemetry.addData("YolkDist", intake.cSSt.getDistance(DistanceUnit.CM));

        intake.setCurrentArtifacts();

        telemetry.addData("artifactState", currentArtifacts );

        telemetry.update();
    }
}
