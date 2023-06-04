package org.firstinspires.ftc.teamcode.Mech.BaseCommands;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class armDrop extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem IntakeSub;

    public armDrop(IntakeSubsystem subsystem) {
        IntakeSub = subsystem;
        addRequirements(IntakeSub);
    }
    @Override
    public void initialize() {
        IntakeSub.grotateToAngle(85);
        SubConstants.conestackHeight--;
    }
    @Override
    public void execute() {
        IntakeSub.armToAngle(90);
    }
    @Override
    public void end(boolean interrupted) {
        IntakeSub.setArmPower(-0.15);
    }


    @Override
    public boolean isFinished() {
        if((IntakeSub.getArmVelocity()<1) && (IntakeSub.getArmAngle()>85))
        {return true;}
        return false;
    }

}