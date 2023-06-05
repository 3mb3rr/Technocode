package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.chassisContestedPole;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeDrop;
import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeGrab;
import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

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
        waitForStart();
        CommandScheduler.getInstance().schedule(new chassisContestedPole(ChassisSub));
        while (!isStopRequested()) {
            running = CommandScheduler.getInstance().isScheduled(new AutoConeGrab(IntakeSub, hSlideSub)) || CommandScheduler.getInstance().isScheduled(new AutoConeDrop(DepositSub, vSlideSub));
            CommandScheduler.getInstance().run();
            if((SubConstants.conestackHeight!=0) && (!running)){
                CommandScheduler.getInstance().schedule(new chassisContestedPole(ChassisSub));
            }
        }
//while(!isStopRequested()){
//    vSlideSub.vSlideToPosition(300);
//}
    }
}
