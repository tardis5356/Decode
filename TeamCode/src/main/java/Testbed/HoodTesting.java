package Testbed;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Hood_Test")
@Disabled
public class HoodTesting extends CommandOpMode {

    private ElapsedTime myTimer = new ElapsedTime();

    DcMotorEx mW;
    Servo sH;
    GamepadEx driver1;

    double hoodPos = 0.95, wheelSpeed = 0;

    double e1, e2, t;


    @Override
    public void initialize(){
        mW = hardwareMap.get(DcMotorEx.class, "mW");
        sH = hardwareMap.get(Servo.class, "sH");

        mW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //mW.setVelocityPIDFCoefficients(.01,0,.002);


        driver1 = new GamepadEx(gamepad1);


        new Trigger(()-> driver1.getButton(GamepadKeys.Button.DPAD_DOWN))
                .whileActiveContinuous(new InstantCommand(()->
                        hoodPos -= .01)
                );

        new Trigger(()-> driver1.getButton(GamepadKeys.Button.DPAD_UP))
                .whileActiveContinuous(new InstantCommand(()->
                        hoodPos += .01)
                );


        new Trigger(()-> driver1.getButton(GamepadKeys.Button.A))
                .whenActive(
                        new SequentialCommandGroup(
                                new InstantCommand(()-> wheelSpeed -= .05),
                                new InstantCommand(()->myTimer.reset())
                                //new InstantCommand(()->e1 = mW.getCurrentPosition())

                        )
                );

        new Trigger(()-> driver1.getButton(GamepadKeys.Button.Y))
                .whenActive(new SequentialCommandGroup(
                                new InstantCommand(()-> wheelSpeed += .05),
                                new InstantCommand(()->myTimer.reset())
                                //new InstantCommand(()->e1 = mW.getCurrentPosition())

                        )
                );

//        new Trigger(()-> driver1.getButton(GamepadKeys.Button.B))
//                .whenActive(new SequentialCommandGroup(
//                                new InstantCommand(()-> wheelSpeed = .8),
//                                new InstantCommand(()->myTimer.reset())//,
//                                //new InstantCommand()
//
//                        )
//                );

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

        //mW.setVelocity(wheelSpeed*360/60, AngleUnit.DEGREES);
        mW.setPower(Math.abs(wheelSpeed));
        sH.setPosition(Math.abs(hoodPos));


        t = myTimer.time(TimeUnit.SECONDS);

        if (myTimer.time(TimeUnit.SECONDS)>3){
            e2 = mW.getCurrentPosition();
            telemetry.addData("tics/second", (e2-e1)/t);
        }
        else{
            e2 = 0;
        }

        telemetry.update();
        //telemetry.addData("Wheel_RPM",mW.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("motorPos", mW.getCurrentPosition());

        telemetry.addData("motorPower",mW.getPower());

        telemetry.addData("CommandedPower", wheelSpeed);

        telemetry.addData("T/s_method",mW.getVelocity());

        telemetry.addData("HoodPos", sH.getPosition());


    }
}
