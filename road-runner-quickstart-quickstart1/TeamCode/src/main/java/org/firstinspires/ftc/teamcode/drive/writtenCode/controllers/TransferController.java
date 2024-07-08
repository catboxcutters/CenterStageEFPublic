package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController.TransferStatus.FLIP_BOX;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController.TransferStatus.INIT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController.TransferStatus.PLACE_BOX_IN_COLLECT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController.TransferStatus.TRANSFER_PIXELS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class TransferController {
    public enum TransferStatus{
        INIT,
        BLOCHEAZA_TUBULETE,
        FLIP_BOX,
        TRANSFER_PIXELS,
        PLACE_BOX_IN_COLLECT,
    }

    public TransferStatus currentStatus = INIT;
    public TransferStatus previousStatus = null;
    private IntakeController intakeController = null;
    private SigurantaOuttakeController sigurantaOuttakeController = null;
    private TubuleteController tubuleteController = null;
    private ExtenderController extenderController = null;
    private Servo forbarCutieIntake = null;
    private AnalogInput encoderForbarCutie = null;

    /// Aici modifici cat timp ia sa-si dea flip cutia
    public double timerFlipCutie = 0.65;
    /// Aici modifici cat timp ia sa se duca pixelii din cutia de intake in cutia de outtake
    public double timerAsteaptaPixeli = 0.30;
    /// Cat timp sa astepte pana da reverse la colectare
    public double timerReverseIntake = 0;

    /// Cat timp in plus sa astepte pana baga in outtake daca se da reverse la cutie
    public static double timerExtendoToInit = 0.3;


    public double actualTimeForExtendo = 0; /// Nu modificati

    // Pozitii pentru forbar cutie intake
    public static double transferPosition = 0.25;
    public static double collectPosition = 1;
    public static double initPosition = 1;

    public ElapsedTime asteaptaCutie = new ElapsedTime();
    public ElapsedTime asteaptaPixeli = new ElapsedTime();
    public ElapsedTime asteaptaPentruReverse = new ElapsedTime();
    public ElapsedTime asteaptaFlip = new ElapsedTime();
    public TransferController(IntakeController intakeController,
                              TubuleteController tubuleteController,
                              SigurantaOuttakeController sigurantaOuttakeController,
                              ExtenderController extenderController,
                              RobotMap robot)
    {
        this.intakeController = intakeController;
        this.extenderController = extenderController;
        this.tubuleteController = tubuleteController;
        this.forbarCutieIntake = robot.forbarCutieIntake;
        this.encoderForbarCutie = robot.encoderForbarCutie;
        this.sigurantaOuttakeController = sigurantaOuttakeController;
    }

    public void update()
    {
        if (currentStatus != previousStatus || currentStatus == FLIP_BOX || currentStatus == TRANSFER_PIXELS || currentStatus == PLACE_BOX_IN_COLLECT || currentStatus == INIT)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case BLOCHEAZA_TUBULETE: /// Imi pune tubuletele si deschide siguranta la outtake
                {
                    sigurantaOuttakeController.currentStatus = SigurantaOuttakeController.SigurantaOuttakeStatus.OPEN;
                    tubuleteController.currentStatus = TubuleteController.CollectStatus.BLOCARE;
                    /// Puteti scoate daca nu vreti sa opriti colectare
                    intakeController.currentStatus = IntakeController.IntakeStatus.STOP;
                    currentStatus = FLIP_BOX;
                    asteaptaFlip.reset();
                    break;
                }
                case FLIP_BOX: // Imi da flip la cutia de intake
                {

                    if(asteaptaFlip.seconds()>0.2) //s au blocat tubuletele dupa acest timp
                         {
                        forbarCutieIntake.setPosition(transferPosition);
                        asteaptaCutie.reset(); /// start la timp
                        asteaptaPentruReverse.reset();
                        currentStatus = TRANSFER_PIXELS;
                        if(extenderController.currentStatus == ExtenderController.ExtenderStatus.FAR)
                            extenderController.currentStatus = ExtenderController.ExtenderStatus.INIT;
                    }
                    break;
                }
                case TRANSFER_PIXELS: // Astept sa dea flip cutia si apoi fac transferul cu tubuletele
                {
                    if (asteaptaPentruReverse.seconds() > timerReverseIntake)
                    {
                        intakeController.currentStatus = IntakeController.IntakeStatus.REVERSE;
                    }
                    if (asteaptaCutie.seconds()>timerFlipCutie + actualTimeForExtendo && encoderForbarCutie.getVoltage()>2.3)
                    {
                        /// Dupa ce si-a dat flip cutia il opresc.
//                        intakeController.currentStatus = IntakeController.IntakeStatus.STOP;
                        tubuleteController.currentStatus = TubuleteController.CollectStatus.TRANSFER;
                        currentStatus = PLACE_BOX_IN_COLLECT;
                        asteaptaPixeli.reset();
                    }
                    break;
                }
                case PLACE_BOX_IN_COLLECT: // Dupa ce s-a facut transferul , inchid siguranta si dau flip la cutia de intake.
                {
                    if (asteaptaPixeli.seconds()>timerAsteaptaPixeli)
                    {
                        // esti aici?
                        intakeController.currentStatus = IntakeController.IntakeStatus.STOP;
                        tubuleteController.currentStatus = TubuleteController.CollectStatus.COLECTARE;
                        sigurantaOuttakeController.currentStatus = SigurantaOuttakeController.SigurantaOuttakeStatus.CLOSED;
                        forbarCutieIntake.setPosition(collectPosition);

                        currentStatus = INIT;
                    }
                    break;
                }
            }
        }
    }
}
