package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class vSlideClose extends CommandBase {

    // The subsystem the command runs on
    private final vSlideSubsystem vSlideSub;

    public vSlideClose(vSlideSubsystem subsystem) {
        vSlideSub = subsystem;
        addRequirements(vSlideSub);
    }

    @Override
    public void execute() {
        vSlideSub.vSlideToPosition(0);
    }

    @Override
    public boolean isFinished() {
        if(((vSlideSub.getSlideVelocity()<5) && (vSlideSub.getSlidePosition()<(5))) || (vSlideSub.slideCurrentSpike()))
        {return true;}
        return false;
    }

}