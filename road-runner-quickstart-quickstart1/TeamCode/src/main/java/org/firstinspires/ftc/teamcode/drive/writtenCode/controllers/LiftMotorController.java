package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LiftMotorController.LiftStatus.AUTO_CYCLE1_C23;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LiftMotorController.LiftStatus.AUTO_CYCLE2_C1;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LiftMotorController.LiftStatus.AUTO_CYCLE2_C23;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LiftMotorController.LiftStatus.INIT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class LiftMotorController {
    public enum LiftStatus{
        INIT,
        GOING_DOWN,
        LOW,
        MID,
        HIGH,
        LOW_AUTO,
        AUTO_CYCLE2_C1,
        AUTO_CYCLE1_C1,
        AUTO_CYCLE1_C23,
        AUTO_CYCLE2_C23,
        AUTO_CYCLE3,
        AUTO_RAISELIFT,
        PRELOAD,
        liftMosaic,
        liftAngle,

    }
    public LiftStatus currentStatus = LiftStatus.INIT;
    public LiftStatus previousStatus = null;
    public static int initPosition = 5;
    public static int downPosition = -100;
    public static int lowPosition = -700;
    public static int midPosition = -700;
    public static int highPosition = -1630;
    public static int anglePosition = -530;
    public static int lowAuto = -350;
    public static int mosaicPosition = -317;
    public static int autoCycle1_C1 = -570;
    public static int autoRaiseLift = -670;
    public static int autoCycle2_C1 = -750;
    public static int autoCycle1_C23 = -660;
    public static int autoCycle2_C23 = -780;
    public static int autoCycle3= -870;

    public int retardPosition = -100;
    public int currentPosition = initPosition;
    private ForbarOuttakeController forbarOuttakeController = null;
    private TurretController turretController = null;
    private ExtenderController extenderController;
    public static DcMotor liftMotor = null;
    public MotorConfigurationType mct1;
    public ElapsedTime timertoinit= new ElapsedTime();
    double delayinit = 0.1;
    boolean flag=false;

    public LiftMotorController(ForbarOuttakeController forbarOuttakeController, ExtenderController extenderController, RobotMap robot, TurretController turretController)
    {
        this.extenderController = extenderController;
        this.forbarOuttakeController = forbarOuttakeController;
        this.liftMotor = robot.liftMotor;
        this.turretController = turretController;
    }
    public void update()
    {
        mct1 = liftMotor.getMotorType().clone();
        mct1.setAchieveableMaxRPMFraction(1.0);
        liftMotor.setMotorType(mct1);
//        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.liftMotor.setTargetPosition(currentPosition);
        this.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.liftMotor.setPower(1);
        double liftCurrentPosition = this.liftMotor.getCurrentPosition();

        if (currentStatus != previousStatus || currentStatus==INIT)
        {
            /// Atentie baieti! Delay-ul de la ridicare forbarului din outtake
            // merge doar daca liftul vine dintr-o pozitie mai mica decat 0 adica din INIT.
            switch (currentStatus)
            {
                case INIT:
                {
//                    extenderController.currentStatus = ExtenderController.ExtenderStatus.INIT;
//                    if(liftCurrentPosition<=-200)
//                    {
//                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.DOWN;
//                    }
//                    else
//                    {
//                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.GET_COLLECTED_PIXELS;
//                    }
                    if(flag==false)
                    {
                        timertoinit.reset();
                        flag=true;
                    }
                    if(turretController.Turret.getPosition()<0.57 || turretController.Turret.getPosition()>0.41) {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.GET_COLLECTED_PIXELS;
                        if(timertoinit.seconds()>delayinit && flag==true){
                            currentPosition=initPosition;
                            flag=false;
                        }
                    }
                    else
                    {
                        turretController.currentStatus= TurretController.TurretStatus.INIT;
                    }
                    break;
                }
                case GOING_DOWN:
                {
                   //
                    //extenderController.currentStatus = ExtenderController.ExtenderStatus.INIT;
                    forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.DOWN;
                    currentPosition=downPosition;
                    flag=false;
                    break;
                }
                case LOW:
                {
                   // if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                    //{
                      //  extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                    //}
                    currentPosition = lowPosition;
                    if (liftCurrentPosition <=0)
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.MOVE_TO_BACKBOARD_DELAY;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    flag=false;
                    break;
                }
                case MID:
                {
                   // if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                    //{
                    //  extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                  //  }
                    currentPosition = midPosition;
                    if (liftCurrentPosition <=0)
                    {
                        forbarOuttakeController.currentStatus =  ForbarOuttakeController.ForbarStatus.MOVE_TO_BACKBOARD_DELAY;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    flag=false;
                    break;
                }
                case HIGH:
                {
                  //  if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                    //{
                      //  extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                    //}
                    currentPosition = highPosition;
                    if (liftCurrentPosition <=0)
                    {
                        forbarOuttakeController.currentStatus =  ForbarOuttakeController.ForbarStatus.MOVE_TO_BACKBOARD_DELAY;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    flag=false;
                    break;
                }
                case LOW_AUTO:
                {
                   // if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                    //{
                      //  extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                    //}
                    currentPosition = lowAuto;
                    if (liftCurrentPosition <=0)
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.MOVE_TO_BACKBOARD_DELAY;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    flag=false;
                    break;
                }
                case AUTO_CYCLE2_C1:
                {
                  //  if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                   // {
                     //   extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                    //}
                    currentPosition = autoCycle2_C1;
                    if (liftCurrentPosition <=0)
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.MOVE_TO_BACKBOARD_DELAY;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    flag=false;
                    break;
                }
                case AUTO_CYCLE1_C1:
                {
                   // if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                    //{
                      //  extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                    //}
                    currentPosition = autoCycle1_C1;
                    if (liftCurrentPosition <=0)
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.MOVE_TO_BACKBOARD_DELAY;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    flag=false;
                    break;
                }
                case AUTO_CYCLE1_C23:
                {
                   // if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                   // {
                     //   extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                  //  }
                    currentPosition = autoCycle1_C23;
                    if (liftCurrentPosition <=0)
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.MOVE_TO_BACKBOARD_DELAY;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    flag=false;
                    break;
                }
                case AUTO_CYCLE2_C23:
                {
                  //  if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                   // {
                   //     extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                    //}
                    currentPosition = autoCycle2_C23;
                    if (liftCurrentPosition <=0)
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.MOVE_TO_BACKBOARD_DELAY;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    flag=false;
                    break;
                }
                case AUTO_CYCLE3:
                {
                    //  if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                    // {
                    //     extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                    //}
                    currentPosition = autoCycle3;
                    if (liftCurrentPosition <=0)
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.MOVE_TO_BACKBOARD_DELAY;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    flag=false;
                    break;
                }
                case AUTO_RAISELIFT:
                {
                    // if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                    //{
                    //  extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                    //}
                    currentPosition = autoRaiseLift;
                    flag=false;
                    break;
                }
                case liftMosaic:
                {
                  //  if(extenderController.currentStatus == ExtenderController.ExtenderStatus.INIT)
                 //   {
                   //     extenderController.currentStatus = ExtenderController.ExtenderStatus.FIX;
                   // }
                    currentPosition = mosaicPosition;
                    if (liftCurrentPosition <=0)
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.MOVE_TO_BACKBOARD_DELAY;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    flag=false;
                    break;

                }
                case liftAngle:
                {
                    currentPosition = anglePosition;
                    if (liftCurrentPosition <=0)
                    {
                        forbarOuttakeController.currentStatus= ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD_WITH_ANGLE;
                    }
                    else
                    {
                        forbarOuttakeController.currentStatus = ForbarOuttakeController.ForbarStatus.PLACE_ON_BACKBOARD_WITH_ANGLE;
                    }
                    flag=false;
                    break;
                }
            }

        }
            previousStatus = currentStatus;
        }
    }
