package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.armDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.autoArmDown;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.depositClose;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.grabberGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.grabberOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideAutoOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideClose;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.slideToConestack;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tArmDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.vSlideClose;
import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

import java.util.function.BooleanSupplier;

public class Retract extends SequentialCommandGroup {

    InstantCommand DecreaseStackHeight = new InstantCommand(() ->{
        SubConstants.conestackHeight--;
    }) ;

    public Retract(IntakeSubsystem IntakeSub, hSlideSubsystem hSlideSub, DepositSubsystem DepositSub, vSlideSubsystem vSlideSub)
    {
        addCommands (
                new ParallelCommandGroup(new hSlideClose(hSlideSub), new tArmDrop(IntakeSub), new vSlideClose(vSlideSub), new dropperGrab(DepositSub))
        );

        addRequirements(IntakeSub);
    }

}
