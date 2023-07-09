package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.chassisReposition;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoMidDrop;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoMidExtend;
import org.firstinspires.ftc.teamcode.Mech.Commands.Retract;
import org.firstinspires.ftc.teamcode.Mech.Commands.chassisContestedPole;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideClose;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tArmDrop;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeDrop;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeExtend;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.chassisRetreat;
import org.firstinspires.ftc.teamcode.Mech.Commands.chassisMidPole;
import org.firstinspires.ftc.teamcode.Mech.Commands.park;
import org.firstinspires.ftc.teamcode.Mech.Commands.zoneDetection;
import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.Camera;
import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

import java.util.function.BooleanSupplier;

@Autonomous
public class AutoMidRight extends LinearOpMode {


    public void runOpMode() {

        DepositSubsystem DepositSub = new DepositSubsystem(hardwareMap);
        vSlideSubsystem vSlideSub = new vSlideSubsystem(hardwareMap);
        IntakeSubsystem IntakeSub = new IntakeSubsystem(hardwareMap);
        hSlideSubsystem hSlideSub = new hSlideSubsystem(hardwareMap);
        ChassisSubsystem ChassisSub = new ChassisSubsystem(hardwareMap);
        ChassisSub.BLorRR = true;
        ChassisSub.auto = true;
        Camera camera = new Camera(hardwareMap);
        CommandScheduler.getInstance().reset();
        SubConstants.conestackHeight = 5;
        ElapsedTime timer = new ElapsedTime();
        BooleanSupplier DepositCone = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return DepositSub.hasCone();
            }
        };
        BooleanSupplier notInitLoop = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return ((isStopRequested()) || (isStarted()));
            }
        };
        CommandScheduler.getInstance().registerSubsystem(ChassisSub, DepositSub, vSlideSub, IntakeSub, hSlideSub, camera);
        CommandScheduler.getInstance().schedule(new ParallelCommandGroup(new hSlideClose(hSlideSub), new zoneDetection(camera)).deadlineWith(new WaitUntilCommand(notInitLoop)));
        while ((!isStopRequested()) && (!isStarted())) {
            CommandScheduler.getInstance().run();
            telemetry.addLine("initialization");

            telemetry.addData("zone", camera.parkingZone);
            telemetry.addData("insight", camera.inSight);
            telemetry.update();
        }

//        new park(ChassisSub, camera.parkingZone));
        timer.reset();
        IntakeSub.botCommandComplete = true;
        while (!isStopRequested()) {
            CommandScheduler.getInstance().run();
            IntakeSub.depositCone(DepositSub.hasCone());
            if (timer.milliseconds() < 27500) {
                if (ChassisSub.pushed() && (ChassisSub.chassisState == ChassisSubsystem.chassis.holding)) {
                    CommandScheduler.getInstance().schedule(true, new SequentialCommandGroup( new Retract(IntakeSub, hSlideSub, DepositSub, vSlideSub), new chassisRetreat(ChassisSub),
                            new WaitCommand(200), new chassisReposition(ChassisSub), new InstantCommand(() -> {
                        IntakeSub.botCommandComplete = true;
                    })));
                } else {
                    if (IntakeSub.botCommandComplete && (SubConstants.conestackHeight == 5)) {
                        IntakeSub.botCommandComplete = false;
                        CommandScheduler.getInstance().schedule(true, new SequentialCommandGroup(new chassisMidPole(ChassisSub), new ParallelCommandGroup(
                                new AutoMidDrop(DepositSub, vSlideSub, ChassisSub.BLorRR),
                                new SequentialCommandGroup(
                                        new AutoConeExtend(IntakeSub, hSlideSub),
                                        new AutoConeGrab(IntakeSub, hSlideSub))
                        )));
                    } else if (IntakeSub.botCommandComplete && (SubConstants.conestackHeight == 0)) {
                        IntakeSub.botCommandComplete = false;
                        CommandScheduler.getInstance().schedule(true, new SequentialCommandGroup(new tArmDrop(IntakeSub),
                                new AutoMidDrop(DepositSub, vSlideSub, ChassisSub.BLorRR),
                                new InstantCommand(() -> {
                                    telemetry.addLine("parking reached");
                                }),
                                new park(ChassisSub, camera.parkingZone)));
                    } else if (IntakeSub.botCommandComplete && (SubConstants.conestackHeight < 5) && (SubConstants.conestackHeight > 0)) {
                        CommandScheduler.getInstance().schedule( new ParallelCommandGroup(
                                new AutoMidDrop(DepositSub, vSlideSub, ChassisSub.BLorRR),
                                new SequentialCommandGroup(
                                        new AutoConeExtend(IntakeSub, hSlideSub),
                                        new AutoConeGrab(IntakeSub, hSlideSub))
                        ));
                    }
                }
            } else {
                if ((ChassisSub.chassisState == ChassisSubsystem.chassis.parking) || (ChassisSub.chassisState == ChassisSubsystem.chassis.parked)) {

                } else {
                    CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new Retract(IntakeSub, hSlideSub, DepositSub, vSlideSub), new park(ChassisSub, camera.parkingZone)));
                }
            }
            telemetry.addData("slide state", hSlideSub.hSlideState);
            telemetry.update();
        }
    }
}
