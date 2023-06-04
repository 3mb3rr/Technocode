package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;

import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class hSlideAutoOpen extends CommandBase {

    // The subsystem the command runs on
    private final hSlideSubsystem hSlideSub;
    private int target = (SubConstants.hSlidePos[5-SubConstants.conestackHeight]);

    public hSlideAutoOpen(hSlideSubsystem subsystem) {
        hSlideSub = subsystem;
        addRequirements(hSlideSub);
    }

    @Override
    public void initialize() {
        target = (SubConstants.hSlidePos[5-SubConstants.conestackHeight]);
        hSlideSub.hSlideToPosition(target);
    }

    @Override
    public boolean isFinished() {
        if((hSlideSub.getSlideVelocity()<5) && (hSlideSub.getSlidePosition()<(target+2)) && (hSlideSub.getSlidePosition()>(target-2)))
        {return true;}
        return false;
    }

}