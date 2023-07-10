package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

public class chassisRetreat extends CommandBase {

    // The subsystem the command runs on
    private final ChassisSubsystem ChassisSub;

    public chassisRetreat(ChassisSubsystem subsystem) {
        ChassisSub = subsystem;
        addRequirements(ChassisSub);
    }

    @Override
    public void initialize() {
        ChassisSub.chassisState = ChassisSubsystem.chassis.driving;
        if(ChassisSub.BLorRR)
            ChassisSub.splineTo(new Pose2d(-26, -1, Math.toRadians(90)), Math.toRadians(0));
        else ChassisSub.splineTo(new Pose2d(-26, -1, Math.toRadians(-90)), Math.toRadians(0));
    }
    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }


    @Override
    public boolean isFinished() {
        return ChassisSub.trajectoryCompleted;
    }

}