package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

public class dropperDrop extends CommandBase {

    // The subsystem the command runs on
    private final DepositSubsystem DepositSub;

    public dropperDrop(DepositSubsystem subsystem) {
        DepositSub = subsystem;
        addRequirements(DepositSub);
    }

    @Override
    public void initialize() {
        DepositSub.dropperDrop();
        DepositSub.openAligner();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}