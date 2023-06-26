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
    public void initialize() {
        hSlideSub.hSlideSetPower(-1);;
    }
    @Override

    public void end(boolean interrupted) {
        hSlideSub.resetEncoder();
        hSlideSub.hSlideToPosition(0);}
//    public void end(boolean interrupted) {hSlideSub.hSlideSetPower(0);}


    @Override
    public boolean isFinished() {
        if((hSlideSub.hClose()) || (hSlideSub.slideCurrentSpike()))
        {return true;}
        return false;
    }

}