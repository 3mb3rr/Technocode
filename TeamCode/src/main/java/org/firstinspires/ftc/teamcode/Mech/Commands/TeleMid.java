package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.MediumSlideOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.depositClose;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperMid;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.tArmDrop;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.depositOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.dropperGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.highSlideOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.ttTurnMiddle;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.ttTurnRight;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.vSlideClose;

import java.util.function.BooleanSupplier;

public class TeleMid extends SequentialCommandGroup {

    public TeleMid(DepositSubsystem DepositSub, vSlideSubsystem vSlideSub, IntakeSubsystem IntakeSub)
    {
        BooleanSupplier DepositHasCone = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return DepositSub.hasCone();
            }
        };
        addCommands (new ConditionalCommand(
                new SequentialCommandGroup(
                        new tArmDrop(IntakeSub),
                        new depositClose(DepositSub),
                        new dropperMid(DepositSub),
                        new MediumSlideOpen(vSlideSub)
                ),
                new SequentialCommandGroup(
                        new transfer(IntakeSub, DepositSub),
                        new WaitCommand(200),
                        new tArmDrop(IntakeSub),
                        new depositClose(DepositSub),
                        new dropperMid(DepositSub),
                        new MediumSlideOpen(vSlideSub)
                ),
                DepositHasCone


        ));
        addRequirements(DepositSub, vSlideSub, IntakeSub);
    }

}
