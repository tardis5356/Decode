package org.firstinspires.ftc.teamcode.Zenith.TeleOps;

import static org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables.currentArtifacts;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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



        //shooter.sH.setPosition(0);



        new Trigger(()-> driver.getButton(GamepadKeys.Button.BACK))
                .toggleWhenActive(()->shooter.hoodOffset = .05,()->shooter.hoodOffset = 0.95);


        new Trigger(()-> driver.getButton(GamepadKeys.Button.A))
                .toggleWhenActive(storage::closeGate, storage::openGate);

//        new Trigger(()-> driver.getButton(GamepadKeys.Button.B))
//                .toggleWhenActive(storage::lowerKicker, storage::raiseKicker);

        new Trigger(()->driver.getButton(GamepadKeys.Button.START))
                .toggleWhenActive(brakePad::deploy, brakePad::retract);

        new Trigger(()->driver.getButton(GamepadKeys.Button.DPAD_UP))
                .toggleWhenActive(bellyPan::engagePTO, bellyPan::disEngagePTO);
    }

    @Override
    public void run() {
        super.run();
        shooter.targeting = false;

        telemetry.addData("hoodPos", shooter.hoodOffset);

        telemetry.addData("PTO_State", bellyPan.PTO_Engaged);
        telemetry.addData("BreakPad_State", brakePad.breakPadEngaged);

        telemetry.addData("GatePos",storage.sG.getPosition());
//        telemetry.addData("KickerPos",storage.sK.getPosition());

        telemetry.addData("IntakeBeamBreak", intake.bbF.getState());
        telemetry.addData("MiddleBeamBreak", intake.bbM.getState());
        telemetry.addData("ShooterBeamBreak", intake.bbSh.getState());

        intake.setCurrentArtifacts();

        telemetry.addData("artifactState", currentArtifacts );

        telemetry.update();
    }
}
