package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;

import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class highSlideOpen extends CommandBase {

    // The subsystem the command runs on
    private final vSlideSubsystem vSlideSub;

    public highSlideOpen(vSlideSubsystem subsystem) {
        vSlideSub = subsystem;
        addRequirements(vSlideSub);
    }

    @Override
    public void execute() {
        vSlideSub.vSlideToPosition((SubConstants.hPolePos));
    }

    @Override
    public boolean isFinished() {
        if((vSlideSub.getSlideVelocity()<5) && (vSlideSub.getSlidePosition()<(SubConstants.hPolePos+2)) && (vSlideSub.getSlidePosition()>(SubConstants.hPolePos-2)))
        {return true;}
        return false;
    }

}