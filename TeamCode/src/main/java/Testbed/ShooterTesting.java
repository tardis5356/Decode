package Testbed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Zenith.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.GlobalVariables;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Zenith.Subsystems.Storage;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name="1.4.26_Shooter_Test")
//@Disabled
public class ShooterTesting extends CommandOpMode {

    public static float vP = 0.009f, vI = 0.000f, vD = 0.000f, vV = 0.000435f, vS = 0.11f;

    PIDController velPIDController = new PIDController(vP, vI, vD);
    SimpleMotorFeedforward velFFController = new SimpleMotorFeedforward(vS, vV);

    private ElapsedTime myTimer = new ElapsedTime();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    VoltageSensor voltageSensor;

    DcMotorEx mW;
    DcMotorEx m2;
    Servo sH;
    GamepadEx driver1;

    Intake intake;
    Storage storage;

    double hoodPos = 0.95;
    public static double wheelSpeedOne = 1410, wheelSpeedTwo = 1125, wheelSpeed_Three = 800;
    double wheelSpeed;

    double e1, e2, t;


    @Override
    public void initialize(){

        mW = hardwareMap.get(DcMotorEx.class,"mSR");
        m2 = hardwareMap.get(DcMotorEx.class,"mSL");
        sH = hardwareMap.get(Servo.class,"sH");
        driver1 = new GamepadEx(gamepad1);
        intake = new Intake(hardwareMap);
        storage = new Storage(hardwareMap);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        m2.setDirection(DcMotorSimple.Direction.REVERSE);

        mW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        new Trigger(()-> driver1.getButton(GamepadKeys.Button.DPAD_UP))
                .whenInactive(new InstantCommand(()->
                        hoodPos += .05)
                );

        new Trigger(()-> driver1.getButton(GamepadKeys.Button.DPAD_DOWN))
                .whenInactive(new InstantCommand(()->
                        hoodPos -= .05)
                );

        new Trigger(()-> driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .toggleWhenActive(storage::closeGate, storage::openGate);


        new Trigger(()-> driver1.getButton(GamepadKeys.Button.Y))
                .whenActive(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> wheelSpeed = wheelSpeedOne),
                                new InstantCommand(()->myTimer.reset())
                                //new InstantCommand(()->e1 = mW.getCurrentPosition())

                        )
                );

        new Trigger(()-> driver1.getButton(GamepadKeys.Button.B))
                .whenActive(new SequentialCommandGroup(
                                new InstantCommand(()-> wheelSpeed = wheelSpeedTwo),
                                new InstantCommand(()->myTimer.reset())
                                //new InstantCommand(()->e1 = mW.getCurrentPosition())

                        )
                );

        new Trigger(()-> driver1.getButton(GamepadKeys.Button.A))
                .whenActive(new SequentialCommandGroup(
                                new InstantCommand(()-> wheelSpeed = wheelSpeed_Three),
                                new InstantCommand(()->myTimer.reset())//,
                                //new InstantCommand()
//
                        )
                );

        new Trigger(()-> driver1.getButton(GamepadKeys.Button.X))
                .whenActive(new SequentialCommandGroup(
                                new InstantCommand(()-> wheelSpeed = 0),
                                new InstantCommand(()->myTimer.reset())//,
                                //new InstantCommand()
//
                        )
                );
        new Trigger(()->driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
                .whenActive(new InstantCommand(intake::in));

        new Trigger(()->driver1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON))
                .whenActive(new InstantCommand(intake::stop));

//        new Trigger(()-> driver1.getButton(GamepadKeys.Button.X))
//                .whenActive(new SequentialCommandGroup(
//                                new InstantCommand(()-> wheelSpeed = .9),
//                                new InstantCommand(()->myTimer.reset())//,
//                                //new InstantCommand()
//
//                        )
//                );

        new Trigger(()->myTimer.time(TimeUnit.SECONDS)>3)
                .whileActiveOnce(
                        new InstantCommand(()-> e1 = mW.getCurrentPosition())
                );

        new Trigger(()-> driver1.getButton(GamepadKeys.Button.START))
                .toggleWhenActive(intake::in, intake::stop);


//        new Trigger(()-> driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER))
//                .whenActive(
//                        new SequentialCommandGroup(
//                                new InstantCommand(storage::raiseKicker),
//                                new WaitCommand(BotPositions.KICKER_WAIT),
//                                new InstantCommand(storage::lowerKicker)
//                        )
//                );


    }

    public void run() {
        super.run();

        velPIDController.setPID(vP,vI,vD);
intake.setCurrentArtifacts();

        //mW.setVelocity(wheelSpeed*360/60, AngleUnit.DEGREES);
        mW.setPower(calculateBangBangFlyWheelPower(wheelSpeed));
        m2.setPower(calculateBangBangFlyWheelPower(wheelSpeed));
        sH.setPosition(Math.abs(hoodPos));



        t = myTimer.time(TimeUnit.SECONDS);



        telemetry.update();
        //telemetry.addData("Wheel_RPM",mW.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("motorPos", mW.getCurrentPosition());

        telemetry.addData("motorPower",mW.getPower());

        telemetry.addData("Intake power",intake.mI.getPower());

        telemetry.addData("currentArtifacts", GlobalVariables.currentArtifacts);

        telemetry.addData("CommandedPower", wheelSpeed);

        telemetry.addData("T/s",mW.getVelocity());

        telemetry.addData("HoodPos", sH.getPosition());
        telemetry.addData("mwah<3",true);


    }


    public double calculateFlyWheelPower(double tps){
        double neededVoltage;
        if(tps == 0){
            neededVoltage = 0;
        }else{
            neededVoltage = velPIDController.calculate(mW.getVelocity(), tps) + velFFController.calculate(tps);
        }
        return neededVoltage * 12.5 / voltageSensor.getVoltage();
    }
    public double calculateBangBangFlyWheelPower(double tps) {
        if (mW.getVelocity() > tps) {
            return 0;
        } else return 1;
    }
}
