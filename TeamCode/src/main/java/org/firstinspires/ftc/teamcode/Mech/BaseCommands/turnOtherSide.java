package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

public class turnOtherSide extends CommandBase {

    // The subsystem the command runs on
    private final ChassisSubsystem ChassisSub;
    private Pose2d pose = new Pose2d();

    public turnOtherSide(ChassisSubsystem subsystem) {
        ChassisSub = subsystem;
        addRequirements(ChassisSub);
    }

    @Override
    public void initialize() {
        ChassisSub.chassisState = ChassisSubsystem.chassis.driving;
        ChassisSub.smoveTo(new Pose2d(-49, 74.5, Math.toRadians(88)));
    }
    @Override
    public void execute() {
        if(!ChassisSub.drive.isBusy()){
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

