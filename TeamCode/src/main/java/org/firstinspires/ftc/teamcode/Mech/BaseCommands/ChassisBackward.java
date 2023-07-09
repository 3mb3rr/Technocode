package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

public class ChassisBackward extends CommandBase {

    // The subsystem the command runs on
    private final ChassisSubsystem ChassisSub;

    public ChassisBackward(ChassisSubsystem subsystem) {
        ChassisSub = subsystem;
        addRequirements(ChassisSub);
    }

    @Override
    public void initialize() {
        ChassisSub.chassisState = ChassisSubsystem.chassis.driving;
        ChassisSub.simpleMoveB(2);
    }
    @Override
    public void execute() {
        ChassisSub.chassisState = ChassisSubsystem.chassis.holding;
    }

    @Override
    public void end(boolean interrupted) {

    }


    @Override
    public boolean isFinished() {
        return ChassisSub.trajectoryCompleted;
    }

}