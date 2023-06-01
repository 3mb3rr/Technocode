package org.firstinspires.ftc.teamcode.Mech.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Mech.SubConstants;

public class DepositSubsystem extends SubsystemBase {

    private final Servo aligner, deposit, dropper;
    public double ttOutput = 0;
    public double ttAngle = 0;
    private final DcMotorEx turntable;
    PIDCoefficients coefficients = new PIDCoefficients(SubConstants.tKp, SubConstants.tKi, SubConstants.tKd);
    BasicPID controller = new BasicPID(coefficients);
    public ElapsedTime slideTime = new ElapsedTime();
    public enum Deposit {
        hasCone, noCone
    }
    Deposit depositState = Deposit.hasCone;
    public DepositSubsystem(final HardwareMap hMap) {
        aligner = hMap.get(Servo.class, "aligner");
        deposit = hMap.get(Servo.class, "deposit");
        dropper = hMap.get(Servo.class, "dropper");
        turntable =  hMap.get(DcMotorEx.class, "turntable");
        ttAngle = (turntable.getCurrentPosition()*SubConstants.degspertick);
    }


    public void turntableToAngle(int degrees){
        ttAngle = (turntable.getCurrentPosition()*SubConstants.degspertick);
        ttOutput = controller.calculate(45, ttAngle);
        turntable.setPower(ttOutput);
    }
    public void openAligner(){
        aligner.setPosition(SubConstants.alignerOpen);
    }
    public void closeAligner(){
        aligner.setPosition(SubConstants.alignerClose);
    }
    public void openDeposit(){
        deposit.setPosition(SubConstants.depositOpen);
    }
    public void closeDeposit(){
        deposit.setPosition(SubConstants.depositClose);
    }
    public void dropperDrop(){
        dropper.setPosition(SubConstants.dropperDrop);
    }
    public void dropperGrab(){
        aligner.setPosition(SubConstants.dropperCollect);
    }
    public void dropperMid(){
        aligner.setPosition(SubConstants.dropperMid);
    }

    public double getTTVelocity() { return turntable.getVelocity();}
    public double getTTAngle() { return ttAngle;}

    public boolean hasCone() {
        switch (depositState) {
            case hasCone:
                return true;
        }
        return false;
    }
    public void hasCone(boolean decision){
        if(decision){depositState = Deposit.hasCone;}
        else {depositState = Deposit.noCone;}
    }
}