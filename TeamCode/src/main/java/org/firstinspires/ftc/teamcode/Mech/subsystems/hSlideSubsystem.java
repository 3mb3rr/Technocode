package org.firstinspires.ftc.teamcode.Mech.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;

public class hSlideSubsystem extends SubsystemBase {

    public double hSlideOutput = 0;
    private final DcMotorEx hSlide;
    private final DigitalChannel hClose;
    private int targetPos = 0;
    PIDCoefficients coefficients = new PIDCoefficients(SubConstants.hKp, SubConstants.hKi, SubConstants.hKd);
    BasicPID controller = new BasicPID(coefficients);
    public ElapsedTime slideTime = new ElapsedTime();


    public hSlideSubsystem(final HardwareMap hMap) {
        hSlide =  hMap.get(DcMotorEx.class, "hslide");
        hClose = hMap.get(DigitalChannel.class, "hclose");
        hSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        hSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void hSlideToPosition(int TargetPos){
        targetPos = TargetPos;
    }
    public void hSlideSetPower(double power){
        targetPos = -1;
        hSlideOutput = power;
    }
    public double getSlideVelocity() { return hSlide.getVelocity();}
    public double getSlidePosition() { return hSlide.getCurrentPosition();}
    public boolean slideCurrentSpike() { return hSlide.isOverCurrent();}

    public boolean hClose() { return !hClose.getState();}
    @Override
    public void periodic(){
        if (targetPos!=-1) {
            hSlideOutput = controller.calculate(targetPos, hSlide.getCurrentPosition());
        }
        hSlide.setPower(hSlideOutput);

    }


}