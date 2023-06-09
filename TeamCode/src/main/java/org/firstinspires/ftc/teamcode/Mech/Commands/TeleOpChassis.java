package org.firstinspires.ftc.teamcode.Mech.Commands;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

public class TeleOpChassis extends CommandBase {

    // The subsystem the command runs on
    private final ChassisSubsystem ChassisSub;

    public TeleOpChassis(ChassisSubsystem subsystem) {
        ChassisSub = subsystem;
        addRequirements(ChassisSub);
    }

    @Override
    public void execute() {
        ChassisSub.TeleOp();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}