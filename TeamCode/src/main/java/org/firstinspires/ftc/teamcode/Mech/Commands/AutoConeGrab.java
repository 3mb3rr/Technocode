package org.firstinspires.ftc.teamcode.Mech.Commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Mech.BaseCommands.armDrop;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.autoArmDown;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.grabberGrab;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.grabberOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideAutoOpen;
import org.firstinspires.ftc.teamcode.Mech.BaseCommands.hSlideClose;
import org.firstinspires.ftc.teamcode.Mech.SubConstants;
import org.firstinspires.ftc.teamcode.Mech.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.hSlideSubsystem;

import java.util.function.BooleanSupplier;

public class AutoConeGrab extends SequentialCommandGroup {

    InstantCommand DecreaseStackHeight = new InstantCommand(() ->{
        SubConstants.conestackHeight--;
    }) ;
    IntakeSubsystem IntakeSubs;

    public AutoConeGrab(IntakeSubsystem IntakeSub, hSlideSubsystem hSlideSub, BooleanSupplier ConeDropped)
    {
        IntakeSubs=IntakeSub;
        addCommands (
                    new WaitUntilCommand(ConeDropped),
                    new WaitCommand(1000),
                    new grabberGrab(IntakeSub),
                    new WaitCommand(400),
                    new ParallelCommandGroup(
                        new armDrop(IntakeSub),
                        new hSlideClose(hSlideSub),
                        new InstantCommand(() ->{
                        SubConstants.conestackHeight--;
                        })
//                        new ConditionalCommand(
//                            new InstantCommand(() ->{
//                            SubConstants.conestackHeight--;
//                            }),
//                            new SequentialCommandGroup(
//                                    new AutoConeExtend(IntakeSub, hSlideSub),
//                                    new grabberGrab(IntakeSub),
//                                    new WaitCommand(200),
//                                    new ParallelCommandGroup(new armDrop(IntakeSub), new hSlideClose(hSlideSub),
//                                            new ConditionalCommand(
//                                                new InstantCommand(() ->{
//                                                    SubConstants.conestackHeight--;
//                                                }),
//                                                new SequentialCommandGroup(
//                                                    new InstantCommand(() ->{
//                                                        SubConstants.conestackHeight--;
//                                                    }),
//                                                    new AutoConeExtend(IntakeSub, hSlideSub),
//                                                    new grabberGrab(IntakeSub),
//                                                    new WaitCommand(200),
//                                                    new ParallelCommandGroup(new armDrop(IntakeSub), new hSlideClose(hSlideSub))),
//                                            () -> IntakeSub.hasCone()))),
//                            () -> IntakeSub.hasCone())
                    ),
                    new grabberOpen(IntakeSub),
                    new WaitCommand(200)
        );
        addRequirements(IntakeSub);
    }

}
