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
    public void initialize() {
        vSlideSub.vSlideToPosition(0);
    }
    @Override
    public void end(boolean interrupted) {
        vSlideSub.setPower(0);
    }

    @Override
    public boolean isFinished() {
        if(((vSlideSub.getSlideVelocity()<5) && (vSlideSub.getSlidePosition()<(15))) || (vSlideSub.slideCurrentSpike()) || (vSlideSub.vClose()))
        {return true;}
        return false;
    }

}