package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.armDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.autoArmDown;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.chassisContestedPole;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.fallenConeGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.grabberOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.leveller;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tArmDown;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tLowPole;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeDrop;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeGrab;
import org.firstinspires.ftc.teamcode.Mech.Commands.TeleConeGrab;
import org.firstinspires.ftc.teamcode.Mech.Commands.TeleDrop;
import org.firstinspires.ftc.teamcode.Mech.Commands.TeleHigh;
import org.firstinspires.ftc.teamcode.Mech.Commands.TeleMid;
import org.firstinspires.ftc.teamcode.Mech.Commands.TeleOpChassis;
import org.firstinspires.ftc.teamcode.Mech.Commands.tConeExtend;
import org.firstinspires.ftc.teamcode.Mech.Commands.transfer;
import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

import java.util.function.BooleanSupplier;

@TeleOp
public class TeleFinal extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public void runOpMode() {
        IntakeSubsystem IntakeSub = new IntakeSubsystem(hardwareMap);
        hSlideSubsystem hSlideSub = new hSlideSubsystem(hardwareMap);
        DepositSubsystem DepositSub = new DepositSubsystem(hardwareMap);
        vSlideSubsystem vSlideSub = new vSlideSubsystem(hardwareMap);
        ChassisSubsystem ChassisSub = new ChassisSubsystem(hardwareMap);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
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
        CommandScheduler.getInstance().registerSubsystem(ChassisSub, DepositSub, vSlideSub, IntakeSub, hSlideSub);
        mechOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenReleased(new InstantCommand(() -> {hSlideSub.hSlideSetPower(0);}))
                .whenPressed(new ConditionalCommand(new WaitCommand(0), new SequentialCommandGroup(new grabberOpen(IntakeSub), new tArmDown(IntakeSub)), hasCone))
                .whileActiveContinuous(new InstantCommand(() -> {hSlideSub.hSlideSetPower(0.75);}));

        mechOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(new TeleConeGrab(IntakeSub, hSlideSub));
        mechOp.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(new transfer(IntakeSub, DepositSub));
        mechOp.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(new SequentialCommandGroup(new tLowPole(IntakeSub), new WaitCommand(500),new grabberOpen(IntakeSub)));
        mechOp.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(new SequentialCommandGroup(new tArmDown(IntakeSub), new WaitCommand(500), new grabberOpen(IntakeSub)));
        mechOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .toggleWhenPressed(new TeleHigh(DepositSub, vSlideSub, IntakeSub));
        mechOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .toggleWhenPressed(new TeleMid(DepositSub, vSlideSub, IntakeSub));
        mechOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .toggleWhenPressed(new TeleDrop(DepositSub, vSlideSub));
        mechOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new fallenConeGrab(IntakeSub));
        while((!isStopRequested()) && (!isStarted())){
            hSlideSub.hSlideSetPower(-0.3);
            CommandScheduler.getInstance().run();
            telemetry.addLine("initialization");
            telemetry.update();
            hSlideSub.resetEncoder();
        }
        CommandScheduler.getInstance().schedule(new TeleOpChassis(ChassisSub));
        while (!isStopRequested()) {
            double r = Math.hypot(driverOp.getLeftY(), -driverOp.getLeftX());
            double robotAngle = Math.atan2(-driverOp.getLeftY(), driverOp.getLeftX()) - Math.PI / 4  - (ChassisSub.getHeading());
            double rightX = (driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)-driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))/2;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            if (gamepad2.left_stick_y != 0 || gamepad2.left_stick_x != 0 || gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0 ) {

                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftRear.setPower(v3);
                rightRear.setPower(v4);
            }
            CommandScheduler.getInstance().run();
            DepositSub.setTTPower(0.5*(mechOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)-mechOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
            telemetry.addData("Y", ChassisSub.getY());
            telemetry.addData("X", ChassisSub.getX());
            telemetry.addData("Heading", ChassisSub.getHeading());
            telemetry.addData("eY", ChassisSub.egetY());
            telemetry.addData("eX", ChassisSub.egetX());
            telemetry.addData("eHeading", ChassisSub.egetHeading());
            telemetry.addData("acorrect postion", ChassisSub.atCorrectPosition());
            telemetry.update();
        }
    }
}
