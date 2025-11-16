package org.firstinspires.ftc.teamcode.DecodeBot.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BellyPan;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.BreakPad;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.DecodeBot.Subsystems.Storage;

@TeleOp (name="Indexer")
public class IndexServos extends CommandOpMode {

    Storage storage;
    Shooter shooter;
    BreakPad breakPad;
    BellyPan bellyPan;
    Intake intake;
    GamepadEx driver;

    @Override
    public void initialize() {

        driver = new GamepadEx(gamepad1);

        storage = new Storage(hardwareMap);
        shooter = new Shooter(hardwareMap);
        breakPad = new BreakPad(hardwareMap);
        bellyPan = new BellyPan(hardwareMap);

        shooter.sH.setPosition(0);

        new Trigger(()-> driver.getButton(GamepadKeys.Button.BACK))
                .whenActive(()->shooter.sH.setPosition(.95));


        new Trigger(()-> driver.getButton(GamepadKeys.Button.A))
                .toggleWhenActive(storage::closeGate, storage::openGate);

        new Trigger(()-> driver.getButton(GamepadKeys.Button.B))
                .toggleWhenActive(storage::lowerKicker, storage::raiseKicker);

        new Trigger(()-> driver.getButton(GamepadKeys.Button.X))
                .toggleWhenActive(storage::storeSlot, storage::returnSlot);

        new Trigger(()-> driver.getButton(GamepadKeys.Button.Y))
                .toggleWhenActive(storage::closeBack, storage::openBack);


        new Trigger(()->driver.getButton(GamepadKeys.Button.START))
                .toggleWhenActive(breakPad::deployBreakPad, breakPad::retractBreakPad);

        new Trigger(()->driver.getButton(GamepadKeys.Button.DPAD_UP))
                .toggleWhenActive(bellyPan::engagePTO, bellyPan::disEngagePTO);
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("PTO_State", bellyPan.PTO_Engaged);
        telemetry.addData("BreakPad_State", breakPad.breakPadEngaged);
        telemetry.addData("YolkPos",storage.sS.getPosition());
        telemetry.addData("GatePos",storage.sG.getPosition());
        telemetry.addData("KickerPos",storage.sK.getPosition());
        telemetry.addData("BackPos",storage.sBG.getPosition());

        telemetry.addData("IntakeRed", intake.cSI.red());
        telemetry.addData("IntakeGreen", intake.cSI.green());
        telemetry.addData("IntakeBlue", intake.cSI.blue());

        telemetry.addData("MiddleRed", intake.cSM.red());
        telemetry.addData("MiddleGreen", intake.cSM.green());
        telemetry.addData("MiddleBlue", intake.cSM.blue());

        telemetry.addData("ShooterRed", intake.cSSh.red());
        telemetry.addData("ShooterGreen", intake.cSSh.green());
        telemetry.addData("ShooterBlue", intake.cSSh.blue());

        telemetry.addData("YolkRed", intake.cSSt.red());
        telemetry.addData("YolkGreen", intake.cSSt.green());
        telemetry.addData("YolkBlue", intake.cSSt.blue());

        telemetry.update();
    }
}
