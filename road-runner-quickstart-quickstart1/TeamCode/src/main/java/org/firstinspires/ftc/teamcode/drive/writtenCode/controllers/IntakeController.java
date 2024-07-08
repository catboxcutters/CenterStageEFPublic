package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class IntakeController {
    public enum IntakeStatus{
        STOP,
        COLLECT_DRIVE,
        REVERSE,
        REVERSE_AUTO,
        STACK,
        FULL,
    }

    public IntakeStatus currentStatus = IntakeStatus.STOP;
    public IntakeStatus previousStatus = null;

    public static double driveCollectPower = 1 ;
    public static double reverseCollectPower = -1;
    public static double reverseAutoPower = -0.4; //-0.3
    public static double fullPower = 1;
    public static double stackPower = 1;
    private DcMotorEx intakeMotor = null;
    public MotorConfigurationType mct1;


    /**
     * Constructor pentru clasa IntakeController
     * Practic cand cream un nou controller trebuie sa pasam robotMap
     * @param robot - Clasa care contine tot hardware-ul robotului
     */
    public IntakeController(RobotMap robot)
    {
        intakeMotor = robot.intakeMotor;
    }

    public void update()
    {
        mct1 = intakeMotor.getMotorType().clone();
        mct1.setAchieveableMaxRPMFraction(1.0);
        intakeMotor.setMotorType(mct1);
        if (currentStatus != previousStatus )
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case STOP:
                {
                    intakeMotor.setPower(0);
                    break;
                }
                case COLLECT_DRIVE:
                {
                    intakeMotor.setPower(driveCollectPower);
                    break;
                }
                case REVERSE:
                {
                    intakeMotor.setPower(reverseCollectPower);
                    break;
                }
                case REVERSE_AUTO:
                {
                    intakeMotor.setPower(reverseAutoPower);
                    break;
                }
                case STACK:
                {
                    intakeMotor.setPower(stackPower);
                    break;
                }
                case FULL:
                {
                    intakeMotor.setPower(fullPower);
                    break;
                }
            }
        }
    }
}