package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

public class midReposition extends CommandBase {

    // The subsystem the command runs on
    private final ChassisSubsystem ChassisSub;

    public midReposition(ChassisSubsystem subsystem) {
        ChassisSub = subsystem;
        addRequirements(ChassisSub);
    }

    @Override
    public void initialize() {
        ChassisSub.chassisState = ChassisSubsystem.chassis.driving;
        if(ChassisSub.BLorRR)
            ChassisSub.moveTo(new Pose2d(-50, -3.5, Math.toRadians(90)));
        else ChassisSub.moveTo(new Pose2d(-50, 1, Math.toRadians(-88)));
    }
    @Override
    public void execute() {
        if(ChassisSub.trajectoryCompleted){
            ChassisSub.chassisState = ChassisSubsystem.chassis.correcting;
        }
    }

    @Override
    public void end(boolean interrupted) {
        ChassisSub.chassisState = ChassisSubsystem.chassis.holding;
    }


    @Override
    public boolean isFinished() {
        return ChassisSub.atCorrectPosition();
    }

}

