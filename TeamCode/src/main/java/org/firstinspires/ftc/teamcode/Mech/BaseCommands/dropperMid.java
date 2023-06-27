package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

public class dropperMid extends CommandBase {

    // The subsystem the command runs on
    private final DepositSubsystem DepositSub;

    public dropperMid(DepositSubsystem subsystem) {
        DepositSub = subsystem;
        addRequirements(DepositSub);
    }

    @Override
    public void initialize() {
        DepositSub.dropperMid();
        DepositSub.openAligner();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}