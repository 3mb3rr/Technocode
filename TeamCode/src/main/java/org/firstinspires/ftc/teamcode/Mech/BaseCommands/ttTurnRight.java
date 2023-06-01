package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;

import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;

public class ttTurnRight extends CommandBase {

    // The subsystem the command runs on
    private final DepositSubsystem DepositSub;

    public ttTurnRight(DepositSubsystem subsystem) {
        DepositSub = subsystem;
        addRequirements(DepositSub);
    }

    @Override
    public void execute() {
        DepositSub.turntableToAngle((SubConstants.ttRightAngle));
    }

    @Override
    public boolean isFinished() {
        if((DepositSub.getTTVelocity()<5) && (DepositSub.getTTAngle()<(SubConstants.ttRightAngle+1)) && (DepositSub.getTTAngle()>(SubConstants.ttRightAngle-1)))
        {return true;}
        return false;
    }

}