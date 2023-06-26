package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.chassisContestedPole;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tArmDown;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tArmDrop;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeDrop;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeExtend;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeGrab;
import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

import java.util.function.BooleanSupplier;

@Autonomous
public class AutoFinal extends LinearOpMode {

    public Servo grotate;
    public boolean running = true;

    public void runOpMode() {

        DepositSubsystem DepositSub = new DepositSubsystem(hardwareMap);
        vSlideSubsystem vSlideSub = new vSlideSubsystem(hardwareMap);
        IntakeSubsystem IntakeSub = new IntakeSubsystem(hardwareMap);
        hSlideSubsystem hSlideSub = new hSlideSubsystem(hardwareMap);
        ChassisSubsystem ChassisSub = new ChassisSubsystem(hardwareMap);
        CommandScheduler.getInstance().reset();
        SubConstants.conestackHeight = 5;
        BooleanSupplier DepositCone = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return DepositSub.hasCone();
            }
        };
        while((!isStopRequested()) && (!isStarted())){
            hSlideSub.hSlideSetPower(-0.3);
            telemetry.addLine("initialization");
            telemetry.update();
            hSlideSub.resetEncoder();
        }
        CommandScheduler.getInstance().registerSubsystem(ChassisSub, DepositSub, vSlideSub, IntakeSub, hSlideSub);
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new ParallelCommandGroup(
                        new AutoConeDrop(DepositSub, vSlideSub),
                        new SequentialCommandGroup(
                                new AutoConeExtend(IntakeSub, hSlideSub),
                                new AutoConeGrab(IntakeSub, hSlideSub, DepositCone))
                ),
        new ParallelCommandGroup(
                new AutoConeDrop(DepositSub, vSlideSub),
                new SequentialCommandGroup(
                        new AutoConeExtend(IntakeSub, hSlideSub),
                        new AutoConeGrab(IntakeSub, hSlideSub, DepositCone))
        ),
                        new ParallelCommandGroup(
                                new AutoConeDrop(DepositSub, vSlideSub),
                                new SequentialCommandGroup(
                                        new AutoConeExtend(IntakeSub, hSlideSub),
                                        new AutoConeGrab(IntakeSub, hSlideSub, DepositCone))
                        ),
                        new ParallelCommandGroup(
                                new AutoConeDrop(DepositSub, vSlideSub),
                                new SequentialCommandGroup(
                                        new AutoConeExtend(IntakeSub, hSlideSub),
                                        new AutoConeGrab(IntakeSub, hSlideSub, DepositCone))
                        ),
                        new ParallelCommandGroup(
                                new AutoConeDrop(DepositSub, vSlideSub),
                                new SequentialCommandGroup(
                                        new AutoConeExtend(IntakeSub, hSlideSub),
                                        new AutoConeGrab(IntakeSub, hSlideSub, DepositCone))
                        ),
                new tArmDrop(IntakeSub),
                new AutoConeDrop(DepositSub, vSlideSub)
                )
        );
        while (!isStopRequested()) {
            running = CommandScheduler.getInstance().isScheduled(new AutoConeGrab(IntakeSub, hSlideSub, DepositCone)) || CommandScheduler.getInstance().isScheduled(new AutoConeDrop(DepositSub, vSlideSub));
            CommandScheduler.getInstance().run();
//            if((SubConstants.conestackHeight!=0) && (!running)){
//                CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
//                    new AutoConeDrop(DepositSub, vSlideSub),
//                    new SequentialCommandGroup(
//                        new AutoConeExtend(IntakeSub, hSlideSub),
//                        new AutoConeGrab(IntakeSub, hSlideSub, DepositCone))
//                )
//                );
//            }
            telemetry.addData("condition", ((SubConstants.conestackHeight!=0) && (!running)));
            telemetry.addData("deposithascone", DepositSub.hasCone());
            telemetry.addData("hclose", hSlideSub.hClose());
            telemetry.update();
        }
//while(!isStopRequested()){
//    vSlideSub.vSlideToPosition(300);
//}
    }
}
