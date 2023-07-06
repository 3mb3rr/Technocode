package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mech.Commands.chassisContestedPole;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideClose;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tArmDrop;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeDrop;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeExtend;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeGrab;
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
public class AutoFinal extends LinearOpMode {

    public Servo grotate;
    public boolean running = true;

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
        while((!isStopRequested()) && (!isStarted())){
            CommandScheduler.getInstance().run();
            telemetry.addLine("initialization");

            telemetry.addData("zone", camera.parkingZone);
            telemetry.addData("insight", camera.inSight);
            telemetry.update();
        }
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new SequentialCommandGroup(new chassisContestedPole(ChassisSub), new ParallelCommandGroup(
                        new AutoConeDrop(DepositSub, vSlideSub, ChassisSub.BLorRR),
                        new SequentialCommandGroup(
                                new AutoConeExtend(IntakeSub, hSlideSub),
                                new AutoConeGrab(IntakeSub, hSlideSub))
                ),
        new ParallelCommandGroup(
                new AutoConeDrop(DepositSub, vSlideSub, ChassisSub.BLorRR),
                new SequentialCommandGroup(
                        new AutoConeExtend(IntakeSub, hSlideSub),
                        new AutoConeGrab(IntakeSub, hSlideSub))
        ),
                        new ParallelCommandGroup(
                                new AutoConeDrop(DepositSub, vSlideSub, ChassisSub.BLorRR),
                                new SequentialCommandGroup(
                                        new AutoConeExtend(IntakeSub, hSlideSub),
                                        new AutoConeGrab(IntakeSub, hSlideSub))
                        ),
                        new ParallelCommandGroup(
                                new AutoConeDrop(DepositSub, vSlideSub, ChassisSub.BLorRR),
                                new SequentialCommandGroup(
                                        new AutoConeExtend(IntakeSub, hSlideSub),
                                        new AutoConeGrab(IntakeSub, hSlideSub))
                        ),
                        new ParallelCommandGroup(
                                new AutoConeDrop(DepositSub, vSlideSub, ChassisSub.BLorRR),
                                new SequentialCommandGroup(
                                        new AutoConeExtend(IntakeSub, hSlideSub),
                                        new AutoConeGrab(IntakeSub, hSlideSub))
                        ),
                new tArmDrop(IntakeSub),
                new AutoConeDrop(DepositSub, vSlideSub, ChassisSub.BLorRR),
                new park(ChassisSub, camera.parkingZone)
                )
        ));
//        new park(ChassisSub, camera.parkingZone));
        while (!isStopRequested()) {
            CommandScheduler.getInstance().run();
            IntakeSub.depositCone(DepositSub.hasCone());
//            if((SubConstants.conestackHeight!=0) && (!running)){
//                CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
//                    new AutoConeDrop(DepositSub, vSlideSub),
//                    new SequentialCommandGroup(
//                        new AutoConeExtend(IntakeSub, hSlideSub),
//                        new AutoConeGrab(IntakeSub, hSlideSub, DepositCone))
//                )
//                );
//            }
            telemetry.addData("zone", camera.parkingZone);
            telemetry.addData("insight", camera.inSight);
            telemetry.addData("grabberCone", IntakeSub.hasCone());
            telemetry.addData("heading", ChassisSub.getHeading());

            telemetry.addData("holding", ChassisSub.chassisState);
            telemetry.addData("atCorrectPosition", ChassisSub.atCorrectPosition());
            telemetry.addData("deposithascone", IntakeSub.depositCone());
            telemetry.addData("hclose", hSlideSub.hClose());
            telemetry.addData("hclose", hSlideSub.hClose());
            telemetry.addData("armState", IntakeSub.armState);
            telemetry.addData("hslideState", hSlideSub.hSlideState);
            telemetry.addData("autoextend", CommandScheduler.getInstance().isScheduled(new AutoConeExtend(IntakeSub, hSlideSub)));
            telemetry.addData("autograb", CommandScheduler.getInstance().isScheduled(new AutoConeGrab(IntakeSub, hSlideSub)));
            telemetry.update();
        }
//while(!isStopRequested()){
//    vSlideSub.vSlideToPosition(300);
//}
    }
}
