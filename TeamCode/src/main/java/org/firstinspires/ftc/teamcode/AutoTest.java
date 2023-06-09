package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.armDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.autoArmDown;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.chassisContestedPole;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.grabberOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.highSlideOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.leveller;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.vSlideClose;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeDrop;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeGrab;
import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

import java.util.function.BooleanSupplier;

@Autonomous
public class AutoTest extends LinearOpMode {

    public Servo grotate;
    public DcMotorEx hSlide;
    public void runOpMode() {
        hSlide = hardwareMap.get(DcMotorEx.class, "hslide");
        grotate = hardwareMap.get(Servo.class, "grotate");
        DepositSubsystem DepositSub = new DepositSubsystem(hardwareMap);
        vSlideSubsystem vSlideSub = new vSlideSubsystem(hardwareMap);
        IntakeSubsystem IntakeSub = new IntakeSubsystem(hardwareMap);
        hSlideSubsystem hSlideSub = new hSlideSubsystem(hardwareMap);
        ChassisSubsystem ChassisSub = new ChassisSubsystem(hardwareMap);
        CommandScheduler.getInstance().reset();
        SubConstants.conestackHeight = 5;
        BooleanSupplier isStarted = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return isStarted();
            }
        };
        CommandScheduler.getInstance().registerSubsystem(ChassisSub, DepositSub, vSlideSub, IntakeSub, hSlideSub);
//        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
//                        new grabberOpen(IntakeSub),
//                        new WaitUntilCommand(isStarted),
//                        new chassisContestedPole(ChassisSub),
//                        new ParallelCommandGroup(new AutoConeGrab(IntakeSub, hSlideSub),new SequentialCommandGroup(new dropperDrop(DepositSub), new WaitCommand(500), new dropperGrab(DepositSub))),
//                        new ParallelCommandGroup(new AutoConeGrab(IntakeSub, hSlideSub),new SequentialCommandGroup(new dropperDrop(DepositSub), new WaitCommand(500), new dropperGrab(DepositSub))),
//                        new ParallelCommandGroup(new AutoConeGrab(IntakeSub, hSlideSub),new SequentialCommandGroup(new dropperDrop(DepositSub), new WaitCommand(500), new dropperGrab(DepositSub))),
//                        new ParallelCommandGroup(new AutoConeGrab(IntakeSub, hSlideSub),new SequentialCommandGroup(new dropperDrop(DepositSub), new WaitCommand(500), new dropperGrab(DepositSub))),
//                        new ParallelCommandGroup(new AutoConeGrab(IntakeSub, hSlideSub),new SequentialCommandGroup(new dropperDrop(DepositSub), new WaitCommand(500), new dropperGrab(DepositSub)))
//                )
//        );
//        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new AutoConeDrop(DepositSub, vSlideSub),
//                new AutoConeDrop(DepositSub, vSlideSub),
//                new AutoConeDrop(DepositSub, vSlideSub),
//                new AutoConeDrop(DepositSub, vSlideSub),
//                new AutoConeDrop(DepositSub, vSlideSub)));
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new highSlideOpen(vSlideSub), new vSlideClose(vSlideSub)));
        while((!isStopRequested()) && (!isStarted())){

            telemetry.addLine("initialization");
            telemetry.update();
        }
        while (!isStopRequested()) {
            CommandScheduler.getInstance().run();
            telemetry.addData("vSlide", vSlideSub.getSlidePosition());
            telemetry.addData("angle", DepositSub.getTTAngle());
            telemetry.update();
        }
//while(!isStopRequested()){
//    vSlideSub.vSlideToPosition(300);
//}
    }
}
