package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.grabberGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideClose;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tArmDrop;
import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;

public class TeleConeGrab extends SequentialCommandGroup {

    InstantCommand DecreaseStackHeight = new InstantCommand(() ->{
        SubConstants.conestackHeight--;
    }) ;
    IntakeSubsystem IntakeSubs;

    public TeleConeGrab(IntakeSubsystem IntakeSub, hSlideSubsystem hSlideSub)
    {
        IntakeSubs=IntakeSub;
        addCommands (
                new grabberGrab(IntakeSub),
                new WaitCommand(300),
                new ParallelCommandGroup(
                        new tArmDrop(IntakeSub),
                        new hSlideClose(hSlideSub)
                )
        );
        addRequirements(IntakeSub);
    }

}
