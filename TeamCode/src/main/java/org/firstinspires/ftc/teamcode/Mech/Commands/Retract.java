package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideClose;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tArmDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.ttTurnMiddle;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.vSlideClose;
import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class Retract extends SequentialCommandGroup {

    InstantCommand DecreaseStackHeight = new InstantCommand(() ->{
        SubConstants.conestackHeight--;
    }) ;

    public Retract(IntakeSubsystem IntakeSub, hSlideSubsystem hSlideSub, DepositSubsystem DepositSub, vSlideSubsystem vSlideSub)
    {
        addCommands (new SequentialCommandGroup(new dropperGrab(DepositSub),
                new ParallelCommandGroup(new hSlideClose(hSlideSub), new tArmDrop(IntakeSub), new vSlideClose(vSlideSub), new ttTurnMiddle(DepositSub))
        ), new transfer(IntakeSub, DepositSub));

        addRequirements(IntakeSub);
    }

}
