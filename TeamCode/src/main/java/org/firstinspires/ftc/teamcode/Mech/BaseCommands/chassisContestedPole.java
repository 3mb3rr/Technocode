package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

public class chassisContestedPole extends CommandBase {

    // The subsystem the command runs on
    private final ChassisSubsystem ChassisSub;

    public chassisContestedPole(ChassisSubsystem subsystem) {
        ChassisSub = subsystem;
        addRequirements(ChassisSub);
    }

    @Override
    public void initialize() {
        ChassisSub.moveTo(new Pose2d(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}