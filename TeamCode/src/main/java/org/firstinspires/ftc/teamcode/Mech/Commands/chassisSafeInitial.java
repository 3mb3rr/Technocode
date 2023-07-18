package org.firstinspires.ftc.teamcode.Mech.Commands;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

public class chassisSafeInitial extends CommandBase {

    // The subsystem the command runs on
    private final ChassisSubsystem ChassisSub;

    public chassisSafeInitial(ChassisSubsystem subsystem) {
        ChassisSub = subsystem;
        addRequirements(ChassisSub);
    }

    @Override
    public void initialize() {
        ChassisSub.chassisState = ChassisSubsystem.chassis.driving;
        if(ChassisSub.BLorRR)
            ChassisSub.moveTo(new Pose2d(-49, 2, Math.toRadians(0)));
        else ChassisSub.moveTo(new Pose2d(-49, -4, Math.toRadians(0)));
    }
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }


    @Override
    public boolean isFinished() {
        return !ChassisSub.drive.isBusy();
    }

}