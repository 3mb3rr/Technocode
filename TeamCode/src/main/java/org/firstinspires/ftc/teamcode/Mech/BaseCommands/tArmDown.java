package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class tArmDown extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem IntakeSub;

    public tArmDown(IntakeSubsystem subsystem) {
        IntakeSub = subsystem;
        addRequirements(IntakeSub);
    }

    @Override
    public void initialize() {
        IntakeSub.armState = IntakeSubsystem.Arm.grabbing;
        SubConstants.armFeedforward = 0.29;
        IntakeSub.armToAngle(-19);
        //change: -15 to -15
        IntakeSub.grotateToAngle(19);
        IntakeSub.fallenCone = false;
        //change: 15 to
    }

    @Override
    public void end(boolean interrupted) {
        IntakeSub.armState = IntakeSubsystem.Arm.holding;
        SubConstants.armFeedforward = 0.23;
    }
    @Override
    public boolean isFinished() {
        if(((IntakeSub.getArmVelocity()<1) && (IntakeSub.getArmAngle()<-17) && (IntakeSub.getArmAngle()>-15))
                || (IntakeSub.getArmVelocity()==0))
        {return true;}
        return false;
    }

}