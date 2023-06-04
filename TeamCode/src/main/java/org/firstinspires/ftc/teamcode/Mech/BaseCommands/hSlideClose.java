package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class hSlideClose extends CommandBase {

    // The subsystem the command runs on
    private final hSlideSubsystem hSlideSub;

    public hSlideClose(hSlideSubsystem subsystem) {
        hSlideSub = subsystem;
        addRequirements(hSlideSub);
    }

    @Override
    public void execute() {
        hSlideSub.hSlideToPosition(0);
    }
    @Override
    public void end(boolean interrupted) {
        hSlideSub.hSlideSetPower(0.1);
    }

    @Override
    public boolean isFinished() {
        if((hSlideSub.getSlidePosition()<(50)) || (hSlideSub.slideCurrentSpike()) || (hSlideSub.hClose()))
        {return true;}
        return false;
    }

}