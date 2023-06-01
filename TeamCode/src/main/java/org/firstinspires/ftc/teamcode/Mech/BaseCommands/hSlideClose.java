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
    public boolean isFinished() {
        if((hSlideSub.getSlideVelocity()<5) && (hSlideSub.getSlidePosition()<(5)))
        {return true;}
        return false;
    }

}