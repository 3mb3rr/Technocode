package org.firstinspires.ftc.teamcode.Mech.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

import org.checkerframework.checker.units.qual.degrees;
import org.firstinspires.ftc.teamcode.Mech.SubConstants;

public class DepositSubsystem extends SubsystemBase {

    private final Servo aligner, deposit, dropper;
    public double ttOutput = 0;
    public double ttAngle = 0;
    public double targetTTAngle = 0;
    private final DcMotorEx turntable;
    PIDCoefficients coefficients = new PIDCoefficients(SubConstants.tKp, SubConstants.tKi, SubConstants.tKd);
    BasicPID controller = new BasicPID(coefficients);
    public ElapsedTime slideTime = new ElapsedTime();
    public enum Deposit {
        hasCone, noCone
    }
    public enum Turntable {
        turning, holding
    }
    Deposit depositState = Deposit.hasCone;
    public Turntable ttState = Turntable.holding;
    public DepositSubsystem(final HardwareMap hMap) {
        register();
        aligner = hMap.get(Servo.class, "aligner");
        deposit = hMap.get(Servo.class, "deposit");
        dropper = hMap.get(Servo.class, "dropper");
        turntable =  hMap.get(DcMotorEx.class, "turntable");
        turntable.setDirection(DcMotorSimple.Direction.REVERSE);
        turntable.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turntable.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        ttAngle = (turntable.getCurrentPosition()*SubConstants.degspertick);
        dropperGrab();
        closeAligner();
        openDeposit();

    }


    public void turntableToAngle(int degrees){
        targetTTAngle = degrees;
    }
    public void setTTPower(double power){
        targetTTAngle = 1000;
        turntable.setPower(power);
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
        dropper.setPosition(SubConstants.dropperCollect);
    }
    public void dropperMid(){
        dropper.setPosition(SubConstants.dropperMid);
    }

    public double getTTVelocity() { return turntable.getVelocity();}
    public double getTTAngle() {
        ttAngle = (turntable.getCurrentPosition()*SubConstants.degspertick);
        return ttAngle;}

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
    @Override
    public void periodic(){
        if (targetTTAngle!=1000){
            ttAngle = (turntable.getCurrentPosition()*SubConstants.degspertick);
            ttOutput = controller.calculate(targetTTAngle, ttAngle);
            turntable.setPower(ttOutput);}



    }

}