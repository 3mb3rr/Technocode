package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.depositOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.highSlideOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.ttTurnMiddle;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.ttTurnRight;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.vSlideClose;

public class AutoConeDrop extends SequentialCommandGroup {

    public AutoConeDrop(DepositSubsystem DepositSub, vSlideSubsystem vSlideSub)
    {
        addCommands (
            new SequentialCommandGroup(new ttTurnRight(DepositSub), new highSlideOpen(vSlideSub), new dropperDrop(DepositSub)),
            new depositOpen(DepositSub),
            new WaitCommand(100),
            new SequentialCommandGroup(new ttTurnMiddle(DepositSub), new vSlideClose(vSlideSub), new dropperGrab(DepositSub))
        );
        addRequirements(DepositSub);
    }

}
