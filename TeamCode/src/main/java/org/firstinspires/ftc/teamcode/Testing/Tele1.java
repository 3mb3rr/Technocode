package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.fallenConeGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.grabberOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.imuReset;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tArmDown;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tArmDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tLowPole;
import org.firstinspires.ftc.teamcode.Mech.Commands.Cycle;
import org.firstinspires.ftc.teamcode.Mech.Commands.TeleConeGrab;
import org.firstinspires.ftc.teamcode.Mech.Commands.TeleDrop;
import org.firstinspires.ftc.teamcode.Mech.Commands.TeleHigh;
import org.firstinspires.ftc.teamcode.Mech.Commands.TeleMid;
import org.firstinspires.ftc.teamcode.Mech.Commands.transfer;
import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

import java.util.function.BooleanSupplier;

@TeleOp
@Disabled
public class Tele1 extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    ElapsedTime timer = new ElapsedTime();

    public void runOpMode() {
//        PhotonCore.enable();
        IntakeSubsystem IntakeSub = new IntakeSubsystem(hardwareMap);
        hSlideSubsystem hSlideSub = new hSlideSubsystem(hardwareMap);
        DepositSubsystem DepositSub = new DepositSubsystem(hardwareMap);
        vSlideSubsystem vSlideSub = new vSlideSubsystem(hardwareMap);
        ChassisSubsystem ChassisSub = new ChassisSubsystem(hardwareMap);

        ChassisSub.auto = false;
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx mechOp = new GamepadEx(gamepad2);
        CommandScheduler.getInstance().reset();
        SubConstants.conestackHeight = 5;
        BooleanSupplier isStarted = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return isStarted();
            }
        };
        BooleanSupplier hasCone = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return IntakeSub.hasCone();
            }
        };
        CommandScheduler.getInstance().registerSubsystem(DepositSub, vSlideSub, IntakeSub, hSlideSub, ChassisSub);
//        CommandScheduler.getInstance().registerSubsystem(ChassisSub, DepositSub, vSlideSub, IntakeSub, hSlideSub);
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenReleased(new InstantCommand(() -> {
                    hSlideSub.hSlideSetPower(0);
                }))
                .whenPressed(new ConditionalCommand(new WaitCommand(0), new SequentialCommandGroup(new grabberOpen(IntakeSub), new tArmDown(IntakeSub)), hasCone))
                .whileActiveContinuous(new InstantCommand(() -> {
                    hSlideSub.hSlideSetPower(1);
                }));

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(new TeleConeGrab(IntakeSub, hSlideSub));
        driverOp.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(new transfer(IntakeSub, DepositSub));
        driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenReleased(new grabberOpen(IntakeSub))
                .whenPressed(new tLowPole(IntakeSub));
        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .whenReleased(new SequentialCommandGroup(new grabberOpen(IntakeSub), new WaitCommand(200), new tArmDrop(IntakeSub)))
                .whenPressed(new tArmDown(IntakeSub));
//                        (new SequentialCommandGroup(new tArmDown(IntakeSub), new WaitCommand(500), new grabberOpen(IntakeSub)));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .toggleWhenPressed(new TeleHigh(DepositSub, vSlideSub, IntakeSub));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .toggleWhenPressed(new TeleMid(DepositSub, vSlideSub, IntakeSub));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .toggleWhenPressed(new TeleDrop(DepositSub, vSlideSub));
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new fallenConeGrab(IntakeSub));
        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(new imuReset(ChassisSub));
        driverOp.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(new Cycle(DepositSub, vSlideSub, IntakeSub, hSlideSub));
        while ((!isStopRequested()) && (!isStarted())) {
            hSlideSub.hSlideSetPower(-0.3);
            CommandScheduler.getInstance().run();
            telemetry.addLine("initialization");
            telemetry.update();
        }
        hSlideSub.resetEncoder();
        timer.reset();
        while (!isStopRequested()) {

            if((timer.milliseconds()>80000) && (timer.milliseconds()>80300)){
                telemetry.addLine("rumbling");
                gamepad1.rumble(200);
                gamepad2.rumble(200);
            }
            if((timer.seconds()>90000) && (timer.milliseconds()>92000)){
                telemetry.addLine("rumbling");
                gamepad1.rumble(200);
                gamepad2.rumble(200);
            }
            telemetry.addData("timer", timer.milliseconds()/1000);
            IntakeSub.depositCone(DepositSub.hasCone());
            ChassisSub.xInput = driverOp.getLeftX();
            ChassisSub.yInput = driverOp.getLeftY();
            ChassisSub.turnInput = driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)-driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            CommandScheduler.getInstance().run();
            if((driverOp.getRightY()>0.1) || (driverOp.getRightY()<-0.1)){
                IntakeSub.grotateLevel(true);
                IntakeSub.armToAngle(IntakeSub.armTargetAngle- (driverOp.getRightY()*2.5));
            }
            DepositSub.setTTPower(0.7 * driverOp.getRightX());
            if (0.5 * (mechOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - mechOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))<0.05){DepositSub.turntableToAngle((int)DepositSub.getTTAngle());}
            telemetry.addData("ArmAngle", IntakeSub.getArmAngle());
            telemetry.addData("ArmTargetAngle", IntakeSub.armTargetAngle);
            telemetry.addData("Y", ChassisSub.getY());
            telemetry.addData("X", ChassisSub.getX());
            telemetry.addData("Heading", ChassisSub.getHeading());
            telemetry.addData("eY", ChassisSub.egetY());
            telemetry.addData("eX", ChassisSub.egetX());
            telemetry.addData("eHeading", ChassisSub.egetHeading());
            telemetry.addData("joystickX", ChassisSub.xInput);
            telemetry.addData("joystickY", ChassisSub.yInput);
            telemetry.addData("turn", ChassisSub.turnInput);

            telemetry.addData("acorrect postion", ChassisSub.atCorrectPosition());
            telemetry.addData("depositCone", IntakeSub.depositCone());
            telemetry.update();
        }
    }
    public void brake() {

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

}
