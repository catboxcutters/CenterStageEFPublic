package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;
@Config
public class ExtenderController {
    public enum ExtenderStatus{
        INIT,
        FIX,
        FAR,
        CLOSE,
        FAILSAFE,
        ONE_PIXEL,
        AUTO,
        AUTO_CYCLE_3,
        MANUAL_LOW,
        MANUAL_HIGH,
        CLOSE_AUTO_C1,
        CLOSE_AUTO_C2,
        CLOSE_AUTO_C3,
        CLOSE_AUTO_RED

    }
    public ExtenderStatus currentStatus = ExtenderStatus.INIT;
    public ExtenderStatus previousStatus = null;
    public static int extenderInit =-50; /// Pozitia de init a extenderului
    public int extenderFar = 960; /// Pozitia de extensie a extenderului
    public static double Kp = 0.0021; //0.0028;
    public static double Ki = 0.0005; //0.0006;
    public static double Kd = 0;
    public int CurrentPosition = -50;
    public int extenderFix = 50;
    public int extenderCloseAuto = 675;
    public int extenderCloseAutoRED = 680;
    public int extenderClose = 475;
    public int extenderAutoOnePixel = 485;
    public int extenderAuto = 875;
    public static int extenderFailsafe = 300;
    public static double PowerCap = 1;
    public static double maxSpeed = 1;
    public static int extenderC1=150;
    public static int extenderC2=300;
    public static int extenderAuto_cycle3 = 845;
    public static int extenderC3=760;
    private DcMotor leftExtension;
    private DcMotor rightExtension;
    SimplePIDController MotorColectarePID = null;
    public ExtenderController(RobotMap robot)
    {
        this.leftExtension = robot.leftExtension;
        this.rightExtension = robot.rightExtension;
        MotorColectarePID = new SimplePIDController(Kp,Ki,Kd);
        MotorColectarePID.targetValue= extenderInit;
        MotorColectarePID.maxOutput = maxSpeed;
    }

    public void update(int ColectarePosition) {
        CurrentPosition = ColectarePosition;
        double powerColectare = MotorColectarePID.update(ColectarePosition);
        powerColectare = Math.max(-PowerCap,Math.min(powerColectare,PowerCap));
        this.leftExtension.setPower(powerColectare);
        this.rightExtension.setPower(powerColectare);

        if (currentStatus != previousStatus || currentStatus==ExtenderStatus.MANUAL_HIGH || currentStatus==ExtenderStatus.MANUAL_LOW)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case FIX:
                {
                    MotorColectarePID.targetValue = extenderFix;
                    break;
                }
                case INIT:
                {
                    MotorColectarePID.targetValue = extenderInit;
                    break;
                }
                case FAR:
                {
                    MotorColectarePID.targetValue = extenderFar;
                    break;
                }
                case CLOSE:
                {
                    MotorColectarePID.targetValue = extenderClose;
                    break;
                }
                case CLOSE_AUTO_C1:
                {
                    MotorColectarePID.targetValue = extenderC1;
                    break;
                }
                case CLOSE_AUTO_C2:
                {
                    MotorColectarePID.targetValue = extenderC2;
                    break;
                }
                case CLOSE_AUTO_C3:
                {
                    MotorColectarePID.targetValue = extenderC3;
                    break;
                }
                case CLOSE_AUTO_RED:
                {
                    MotorColectarePID.targetValue = extenderCloseAutoRED;
                    break;
                }
                case FAILSAFE:
                {
                    MotorColectarePID.targetValue = extenderFailsafe;
                    break;
                }
                case AUTO:
                {
                    MotorColectarePID.targetValue = extenderAuto;
                    break;
                }
                case AUTO_CYCLE_3:
                {
                    MotorColectarePID.targetValue = extenderAuto_cycle3;
                    break;
                }
                case ONE_PIXEL:
                {
                    MotorColectarePID.targetValue = extenderAutoOnePixel;
                    break;
                }
                case MANUAL_LOW:
                {
                MotorColectarePID.targetValue = rightExtension.getCurrentPosition()-150;
                    break;
                }
                case MANUAL_HIGH:
                {
              MotorColectarePID.targetValue = rightExtension.getCurrentPosition()+150;
                    break;
                }


            }
        }
    }
}
