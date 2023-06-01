package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

public class ttTurnMiddle extends CommandBase {

    // The subsystem the command runs on
    private final DepositSubsystem DepositSub;

    public ttTurnMiddle(DepositSubsystem subsystem) {
        DepositSub = subsystem;
        addRequirements(DepositSub);
    }

    @Override
    public void execute() {
        DepositSub.turntableToAngle((0));
    }

    @Override
    public boolean isFinished() {
        if((DepositSub.getTTVelocity()<5) && (DepositSub.getTTAngle()<(1)) && (DepositSub.getTTAngle()>(-1)))
        {return true;}
        return false;
    }

}