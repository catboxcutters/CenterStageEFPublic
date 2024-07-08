package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class TurretController {
    public enum TurretStatus{
        INIT,
        RUNTO,
    }
    public TurretStatus currentStatus = TurretStatus.INIT;
    public TurretStatus previousStatus = null;

    public double initPosition = 0.49; // De modificat

    public Servo Turret = null;
    public double targetposition = initPosition;
    public double previousPosition;

    public TurretController(RobotMap robot)
    {
        Turret = robot.Turret;
    }

    public void update()
    {
        if (currentStatus != previousStatus || previousPosition != targetposition)
        {
            previousStatus = currentStatus;
            previousPosition = targetposition;
            switch (currentStatus)
            {
                case INIT:
                {
                    Turret.setPosition(initPosition);
                    break;
                }
                case RUNTO:
                {
                    Turret.setPosition(targetposition);
                    break;
                }
            }
        }
    }
}
