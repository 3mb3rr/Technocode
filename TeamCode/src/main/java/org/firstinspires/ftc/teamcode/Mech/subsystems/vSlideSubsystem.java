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

public class vSlideSubsystem extends SubsystemBase {

    public double vSlideOutput = 0;
    private final DcMotorEx vSlide;
    private final DigitalChannel vClose;
    private int targetPos = 0;
    PIDCoefficients coefficients = new PIDCoefficients(SubConstants.vKp, SubConstants.vKi, SubConstants.vKd);
    BasicPID controller = new BasicPID(coefficients);
    public ElapsedTime slideTime = new ElapsedTime();
    public enum Deposit {
        hasCone, noCone
    }
    public enum VSlide {
        extending, retracting, holding
    }
    public VSlide vSlideState = VSlide.holding;
    Deposit depositState = Deposit.hasCone;
    public vSlideSubsystem(final HardwareMap hMap) {
        register();
        vSlide =  hMap.get(DcMotorEx.class, "vslide");
        vSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        vSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        vClose = hMap.get(DigitalChannel.class, "vclose");
    }

    public void vSlideToPosition(int TargetPos){
        targetPos = TargetPos;
    }
    public void setPower(double Power){
        targetPos = -1;
        vSlideOutput = Power;
    }
    public void resetEncoder(){
        vSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getSlideVelocity() { return vSlide.getVelocity();}
    public double getSlidePosition() { return vSlide.getCurrentPosition();}
    public boolean slideCurrentSpike() { return vSlide.isOverCurrent();}
    public boolean vClose() { return
            !vClose.getState();}

    @Override
    public void periodic(){
        if (targetPos!=-1){
            vSlideOutput = controller.calculate(targetPos, vSlide.getCurrentPosition());
        }
        vSlide.setPower(vSlideOutput+0.15);
    }

}