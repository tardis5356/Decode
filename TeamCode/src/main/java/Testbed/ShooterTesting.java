package Testbed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name="10.2.25_Shooter_Test")
public class ShooterTesting extends CommandOpMode {

    public static float vP = 0.0013f, vI = 0.000012f, vD = 0.000015f, vV = 0.00052f, vS = 0;

    PIDController velPIDController = new PIDController(vP, vI, vD);
    SimpleMotorFeedforward velFFController = new SimpleMotorFeedforward(vS, vV);

    private ElapsedTime myTimer = new ElapsedTime();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    DcMotorEx mW;
    DcMotorEx m2;
    Servo sH;
    GamepadEx driver1;

    double hoodPos = 0.95;
    public static double wheelSpeedOne = 1400, wheelSpeedTwo = 1100, wheelSpeed_Three = 800;
    double wheelSpeed;

    double e1, e2, t;


    @Override
    public void initialize(){

        mW = hardwareMap.get(DcMotorEx.class,"mST");
        m2 = hardwareMap.get(DcMotorEx.class,"mSB");
        sH = hardwareMap.get(Servo.class,"sH");
        driver1 = new GamepadEx(gamepad1);

        mW.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        new Trigger(()-> driver1.getButton(GamepadKeys.Button.DPAD_UP))
                .whenInactive(new InstantCommand(()->
                        hoodPos += .05)
                );

        new Trigger(()-> driver1.getButton(GamepadKeys.Button.DPAD_DOWN))
                .whenInactive(new InstantCommand(()->
                        hoodPos -= .05)
                );


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
    }

    public void run() {
        super.run();

        dashboard.updateConfig();


        //mW.setVelocity(wheelSpeed*360/60, AngleUnit.DEGREES);
        mW.setPower(calculateFlyWheelPower(wheelSpeed));
        m2.setPower(calculateFlyWheelPower(wheelSpeed));
        sH.setPosition(Math.abs(hoodPos));


        t = myTimer.time(TimeUnit.SECONDS);



        telemetry.update();
        //telemetry.addData("Wheel_RPM",mW.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("motorPos", mW.getCurrentPosition());

        telemetry.addData("motorPower",mW.getPower());

        telemetry.addData("CommandedPower", wheelSpeed);

        telemetry.addData("T/s_method",mW.getVelocity());

        telemetry.addData("HoodPos", sH.getPosition());


    }


    public double calculateFlyWheelPower(double tps){
        return velPIDController.calculate(mW.getVelocity(), tps) + velFFController.calculate(tps);
    }
}
