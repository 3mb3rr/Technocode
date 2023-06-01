package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mech.Commands.AutoConeDrop;
import org.firstinspires.ftc.teamcode.Mech.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Mech.subsystems.vSlideSubsystem;

@Autonomous
public class AutoTest extends LinearOpMode{

    public Servo aligner, deposit, dropper;
    public DcMotorEx vSlide, turntable;

    public void runOpMode() {

        aligner = hardwareMap.get(Servo .class, "aligner");
        deposit = hardwareMap.get(Servo.class, "deposit");
        dropper = hardwareMap.get(Servo.class, "dropper");
        vSlide =  hardwareMap.get(DcMotorEx .class, "vslide");
        turntable =  hardwareMap.get(DcMotorEx.class, "turntable");
        DepositSubsystem DepositSub = new DepositSubsystem(hardwareMap);
        vSlideSubsystem vSlideSub = new vSlideSubsystem(hardwareMap);
        waitForStart();
        CommandScheduler.getInstance().schedule(new AutoConeDrop(DepositSub, vSlideSub));
        CommandScheduler.getInstance().run();
    }
}

