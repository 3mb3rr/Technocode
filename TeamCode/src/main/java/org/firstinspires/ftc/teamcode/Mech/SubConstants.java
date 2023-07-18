package org.firstinspires.ftc.teamcode.Mech;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class SubConstants {

    public static double vKp = 0.007;
    // change : 0.014, 0.0110

    public static double vKi = 0.0005;
//    public static double vKi = 0.0002;

    public static double vKd = 0.002;
//    increase vkd in case of oscillation
//    public static double vKd = 0.01;
    public static double tKp = 0.038;

    public static double tKi = 0.002;
//    public static double tKi = 0.002;
    public static double tKd = 0.01;

    public static double hKp = 0.0029;

    public static double hKi = 0;

    public static double hKd = 0.004;

    public static double aKp = 0.028;
// 0.028 to 0.026
    public static double aKi = 0;

    public static double aKd = 0.02;
    public static double cKp = 0.001;

    public static double cKi = 0;

    public static double cKd = 0;

    public static int hPolePos = 1100;
    public static int mPolePos = 510;

    public static int ttRightAngle = -45;
    public static int ttLeftAngle = 40;
    //change -53 to -45
    public static int fLeftAngle = 37;
    public static int fRightAngle = -41;


    public static double dropperDrop = 0;

    public static double dropperMid = 0.42;

    public static double alignerOpen = 0.46;
    // change 0.3 to 0.2

    public static double alignerClose = 0.62;

    public static double dropperCollect = 1;
    public static double depositOpen = 0.27;
    public static double depositClose = 0;
    public static double grabberOpen = 0.81;
//    public static double grabberOpen = 0.81;
    public static double grabberClose = 0.52;
//    public static double grabberClose = 0.5;
    public static double armFeedforward = 0.23;
    // 0.2
    public static double degpervolt=(1.423-0.584)/90;

    public static double degspertick=360/(1425.1*3);
    public static double grotatePosPerDeg=0.0038;
    public static Pose2d chassisContestedLeft= new Pose2d(-49, -4, Math.toRadians(0));
    public static Pose2d chassisContestedRight= new Pose2d(-49, 2, Math.toRadians(0));
    public static Pose2d chassisMidLeft= new Pose2d(-49, 0, Math.toRadians(0));
    public static Pose2d chassisMidRight= new Pose2d(-49, -3.5, Math.toRadians(0));
    public static Pose2d constestedRetreatRight= new Pose2d(-26, -1, Math.toRadians(90));
    public static Pose2d constestedRetreatLeft= new Pose2d(-26, 1, Math.toRadians(-90));
    public static int[] armAngle = {6, 0 , -7, -13, -20};
    //change: 10 to 8, 6 t0 6, 0 to 0,-13 to -12,3-15 TO -14
//    public static int[] hSlidePos = {450, 465, 465, 475, 480, 480};
    // 460, 465, 460, 465, 470
    public static int hSlidePos = 290;
    public static int conestackHeight = 5;
}
