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

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.chassisOtherSide;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.chassisReposition;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.turnOtherSide;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoFastDrop;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoFastGrab;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoSafeDrop;
import org.firstinspires.ftc.teamcode.Mech.Commands.Retract;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideClose;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tArmDrop;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeExtend;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.chassisRetreat;
import org.firstinspires.ftc.teamcode.Mech.Commands.chassisContestedPole;
import org.firstinspires.ftc.teamcode.Mech.Commands.fastContestedPole;
import org.firstinspires.ftc.teamcode.Mech.Commands.fastPark;
import org.firstinspires.ftc.teamcode.Mech.Commands.park;
import org.firstinspires.ftc.teamcode.Mech.Commands.turnContestedPole;
import org.firstinspires.ftc.teamcode.Mech.Commands.xFastPark;
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
public class AutoFast extends LinearOpMode {


    public void runOpMode() {

        DepositSubsystem DepositSub = new DepositSubsystem(hardwareMap);
        vSlideSubsystem vSlideSub = new vSlideSubsystem(hardwareMap);
        IntakeSubsystem IntakeSub = new IntakeSubsystem(hardwareMap);
        hSlideSubsystem hSlideSub = new hSlideSubsystem(hardwareMap);
        ChassisSubsystem ChassisSub = new ChassisSubsystem(hardwareMap);
        ChassisSub.BLorRR = false;
        ChassisSub.auto = true;
        Camera camera = new Camera(hardwareMap);
        boolean temp = false;
        CommandScheduler.getInstance().reset();
        SubConstants.conestackHeight = 5;
        ElapsedTime timer = new ElapsedTime();
        BooleanSupplier DepositCone = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return DepositSub.hasCone();
            }
        };
        BooleanSupplier notDepositCone = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !DepositSub.hasCone();
            }
        };
        BooleanSupplier noIntakeCone = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return ((IntakeSub.hasCone()) && (hSlideSub.hSlideState == hSlideSubsystem.HSlide.retracting));
            }
        };
        BooleanSupplier notInitLoop = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return ((isStopRequested()) || (isStarted()));
            }
        };
        CommandScheduler.getInstance().registerSubsystem(ChassisSub, DepositSub, vSlideSub, IntakeSub, hSlideSub, camera);
        CommandScheduler.getInstance().schedule(new ParallelCommandGroup(new hSlideClose(hSlideSub), new tArmDrop(IntakeSub), new zoneDetection(camera)).deadlineWith(new WaitUntilCommand(notInitLoop)));
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
            telemetry.addLine("entered");
            telemetry.addData("pushed", ChassisSub.pushed());
            telemetry.update();
            CommandScheduler.getInstance().run();
            IntakeSub.depositCone(DepositSub.hasCone());
                    if (IntakeSub.botCommandComplete && (SubConstants.conestackHeight == 5) && (!temp)) {
                        IntakeSub.botCommandComplete = false;
                        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new tArmDrop(IntakeSub), new fastContestedPole(ChassisSub)/*, new turnContestedPole(ChassisSub)*/, new ParallelCommandGroup(
                                new AutoFastDrop(DepositSub, vSlideSub, ChassisSub.BLorRR),
                                new SequentialCommandGroup(
                                        new AutoConeExtend(IntakeSub, hSlideSub),
                                        new AutoFastGrab(IntakeSub, hSlideSub))
                        )));
                    } else if (IntakeSub.botCommandComplete && (SubConstants.conestackHeight == 1) && (!temp)) {
                        IntakeSub.botCommandComplete = false;
                        temp = true;
                        CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
                                new ParallelCommandGroup(
                                        new AutoFastDrop(DepositSub, vSlideSub, ChassisSub.BLorRR),
                                        new SequentialCommandGroup(
                                                new AutoConeExtend(IntakeSub, hSlideSub),
                                                new AutoFastGrab(IntakeSub, hSlideSub))),
                                new SequentialCommandGroup( new WaitUntilCommand(noIntakeCone),
                                new chassisOtherSide(ChassisSub), new InstantCommand(() -> {SubConstants.conestackHeight=5;}),
                                        new InstantCommand(() -> {ChassisSub.BLorRR=true;}))
                                ));
                    } else if (IntakeSub.botCommandComplete && (SubConstants.conestackHeight < 6) && (SubConstants.conestackHeight > 1) && (!temp)) {
                        CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
                                new AutoFastDrop(DepositSub, vSlideSub, ChassisSub.BLorRR),
                                new SequentialCommandGroup(
                                        new AutoConeExtend(IntakeSub, hSlideSub),
                                        new AutoFastGrab(IntakeSub, hSlideSub))
                        ));
                    }
                    else if (IntakeSub.botCommandComplete && (SubConstants.conestackHeight < 6) && (SubConstants.conestackHeight >2) && (temp)) {
                        CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
                                new AutoFastDrop(DepositSub, vSlideSub, true),
                                new SequentialCommandGroup(
                                        new AutoConeExtend(IntakeSub, hSlideSub),
                                        new AutoFastGrab(IntakeSub, hSlideSub))
                        ));
                    }
                    else if((temp) && (SubConstants.conestackHeight==2) && (IntakeSub.botCommandComplete)){
                        IntakeSub.botCommandComplete = false;
                        CommandScheduler.getInstance().schedule( new SequentialCommandGroup(new tArmDrop(IntakeSub),
                                new ParallelCommandGroup(
                                        new AutoFastDrop(DepositSub, vSlideSub, true),
                                        new SequentialCommandGroup(new WaitUntilCommand(notDepositCone), new WaitCommand(200), new xFastPark(ChassisSub, camera.parkingZone)))));
                    }


        }
    }
}
