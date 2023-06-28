package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.depositClose;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.depositOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.highSlideOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.ttTurnMiddle;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.ttTurnRight;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.vSlideClose;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

public class Cycle extends SequentialCommandGroup {

    public Cycle(DepositSubsystem DepositSub, vSlideSubsystem vSlideSub, IntakeSubsystem IntakeSub, hSlideSubsystem hSlideSub)
    {
        addCommands (
                new ParallelCommandGroup(
                        new tCycleDrop(DepositSub, vSlideSub),
                        new SequentialCommandGroup(
                                new tCycleExtend(IntakeSub, hSlideSub),
                                new AutoConeGrab(IntakeSub, hSlideSub))
                )
        );
        addRequirements(DepositSub, vSlideSub);
    }

}

        
