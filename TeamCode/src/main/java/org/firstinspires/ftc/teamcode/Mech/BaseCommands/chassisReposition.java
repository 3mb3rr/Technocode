package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

public class chassisReposition extends CommandBase {

    // The subsystem the command runs on
    private final ChassisSubsystem ChassisSub;
    private Pose2d pose = new Pose2d();

    public chassisReposition(ChassisSubsystem subsystem) {
        ChassisSub = subsystem;
        addRequirements(ChassisSub);
        if(ChassisSub.BLorRR)
            pose = new Pose2d(SubConstants.chassisContestedRight.getX(), SubConstants.chassisContestedRight.getY(), Math.toRadians(88));
        else
            pose = new Pose2d(SubConstants.chassisContestedLeft.getX(), SubConstants.chassisContestedLeft.getY(), Math.toRadians(-89));
    }

    @Override
    public void initialize() {
        ChassisSub.chassisState = ChassisSubsystem.chassis.driving;
            ChassisSub.splineTo(pose, Math.toRadians(90));
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

