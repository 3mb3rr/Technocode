package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideClose;
import org.firstinspires.ftc.teamcode.Mech.Commands.chassisContestedPole;
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
        Camera camera = new Camera(hardwareMap);
        CommandScheduler.getInstance().reset();
        SubConstants.conestackHeight = 5;
        ChassisSub.BLorRR=true;
        BooleanSupplier notInitLoop = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return ((isStopRequested()) || (isStarted()));
            }
        };
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
//        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new highSlideOpen(vSlideSub), new vSlideClose(vSlideSub)));
        CommandScheduler.getInstance().schedule(new ParallelCommandGroup(new hSlideClose(hSlideSub), new zoneDetection(camera)).deadlineWith(new WaitUntilCommand(notInitLoop)));
        while((!isStopRequested()) && (!isStarted())){
            CommandScheduler.getInstance().run();
            telemetry.addData("zone", camera.parkingZone);
            telemetry.addData("insight", camera.inSight);
            telemetry.addLine("initialization");
            telemetry.update();
        }
        CommandScheduler.getInstance().schedule(new park(ChassisSub, camera.parkingZone));
        while (!isStopRequested()) {
            CommandScheduler.getInstance().run();
            telemetry.addData("holding", ChassisSub.chassisState);

            telemetry.addData("holding", ChassisSub.trajectoryCompleted);
            telemetry.addData("vSlide", vSlideSub.getSlidePosition());
            telemetry.addData("angle", DepositSub.getTTAngle());

            telemetry.addData("Y", ChassisSub.getY());
            telemetry.addData("X", ChassisSub.getX());
            telemetry.addData("Heading", ChassisSub.getHeading());
            telemetry.addData("eY", ChassisSub.egetY());
            telemetry.addData("eX", ChassisSub.egetX());
            telemetry.addData("eHeading", ChassisSub.egetHeading());
            telemetry.addData("acorrect postion", ChassisSub.atCorrectPosition());            telemetry.update();
        }
//while(!isStopRequested()){
//    vSlideSub.vSlideToPosition(300);
//}
    }
}
