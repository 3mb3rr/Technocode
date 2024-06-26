package org.firstinspires.ftc.teamcode.Mech.Commands;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

public class chassisMidPole extends CommandBase {

    // The subsystem the command runs on
    private final ChassisSubsystem ChassisSub;

    public chassisMidPole(ChassisSubsystem subsystem) {
        ChassisSub = subsystem;
        addRequirements(ChassisSub);
    }

    @Override
    public void initialize() {
        ChassisSub.chassisState = ChassisSubsystem.chassis.driving;
        if(ChassisSub.BLorRR)
            ChassisSub.moveTo(SubConstants.chassisMidRight, 88);
        else ChassisSub.moveTo(SubConstants.chassisMidLeft, -88);
    }
    @Override
    public void execute() {
        if(!ChassisSub.drive.isBusy()){
            ChassisSub.chassisState = ChassisSubsystem.chassis.holding;
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