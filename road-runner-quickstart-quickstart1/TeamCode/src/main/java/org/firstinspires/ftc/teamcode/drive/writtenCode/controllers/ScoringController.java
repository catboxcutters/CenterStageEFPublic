package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ScoringController.ScoringStatus.DROP_BOTTOM_PIXEL;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ScoringController.ScoringStatus.INIT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ScoringController.ScoringStatus.WAIT_FOR_SAFETY;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ScoringController {
    public enum ScoringStatus{
        INIT,
        DROP_BOTH_PIXELS,
        DROP_ONE_PIXEL,
        WAIT_FOR_SAFETY,
        DROP_BOTTOM_PIXEL,
    }
    public ScoringStatus currentStatus = INIT;
    public ScoringStatus previousStatus = null;
    public TurretController turretController;
    public ParbrizController parbrizController;
    public SigurantaOuttakeController sigurantaOuttakeController;
    public RotateClawController rotateClawController;
    public ElapsedTime waitForPixel2Safety = new ElapsedTime();

    /// Modificati asta pentru cat de mult ii ia servoului de la al 2 lea pixel sa se inchida
    // atunci cand vreti sa dati drop doar la unul
    private double timerSigurantaPixel2 = 0.3;

    public ScoringController(TurretController turretController, SigurantaOuttakeController sigurantaOuttakeController,
                             ParbrizController parbrizController , RotateClawController rotateClawController)
    {
        this.rotateClawController = rotateClawController;
        this.turretController = turretController;
        this.sigurantaOuttakeController = sigurantaOuttakeController;
        this.parbrizController = parbrizController;
    }
    public void update()
    {
        if (currentStatus!=previousStatus || currentStatus == WAIT_FOR_SAFETY)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case DROP_ONE_PIXEL:
                {
//                    turretController.currentStatus = TurretController.Pixel2Status.CLOSED;
                    waitForPixel2Safety.reset();
                    currentStatus = WAIT_FOR_SAFETY;
                    break;
                }
                case WAIT_FOR_SAFETY:
                {
                    if (waitForPixel2Safety.seconds()> timerSigurantaPixel2)
                    {
                        currentStatus = DROP_BOTTOM_PIXEL;
                    }
                    break;
                }
                case DROP_BOTTOM_PIXEL:
                {
                    sigurantaOuttakeController.currentStatus = SigurantaOuttakeController.SigurantaOuttakeStatus.OPEN;
//                    turretController.currentStatus = TurretController.Pixel2Status.CLOSED;
                    currentStatus = INIT;
                    break;
                }
                case DROP_BOTH_PIXELS:
                {
                    /// Dam drumul la ambii pixeli
                    sigurantaOuttakeController.currentStatus = SigurantaOuttakeController.SigurantaOuttakeStatus.OPEN;
                    /// Daca am cutia orizontal clar trebuie sa deschid parbrizul.
                    if (rotateClawController.currentStatus == RotateClawController.RotateStatus.HORIZONTAL)
                    {
                        parbrizController.currentStatus = ParbrizController.ParbrizStatus.OPEN;
                    }
                    currentStatus = INIT;
                    break;
                }
            }
        }
    }
}
