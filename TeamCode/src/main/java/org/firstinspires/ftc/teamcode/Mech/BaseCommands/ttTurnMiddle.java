package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;

public class ttTurnMiddle extends CommandBase {

    // The subsystem the command runs on
    private final DepositSubsystem DepositSub;

    public ttTurnMiddle(DepositSubsystem subsystem) {
        DepositSub = subsystem;
        addRequirements(DepositSub);
    }

    @Override
    public void initialize() {
        DepositSub.turntableToAngle((0));
        DepositSub.ttState = DepositSub.ttState.turning;
    }
    @Override
    public void end(boolean interrupted) {
        DepositSub.ttState = DepositSub.ttState.holding;
    }
    @Override
    public boolean isFinished() {
        if((DepositSub.getTTVelocity()<5) && (DepositSub.getTTAngle()<(3)) && (DepositSub.getTTAngle()>(-3)))
        {return true;}
        return false;
    }

}