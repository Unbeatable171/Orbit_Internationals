package org.firstinspires.ftc.teamcode.ACTIONS;

import org.firstinspires.ftc.teamcode.ShooterCalculator;
import org.firstinspires.ftc.teamcode.SUBSYSTEMS.FlyWheelSubsystem;

public class ShootAction {

    private final FlyWheelSubsystem flyWheelSubsystem;

    public ShootAction(FlyWheelSubsystem flyWheelSubsystem) {
        this.flyWheelSubsystem = flyWheelSubsystem;
    }

    public void update(ShooterCalculator.ShotSolution shotSolution) {
        if (shotSolution == null) {
            return;
        }

//        flyWheelSubsystem.spinUp(shotSolution.rpm);
//        flyWheelSubsystem.setHoodAngle(shotSolution.hoodAngleDeg);
        flyWheelSubsystem.spinUp(2300);
        flyWheelSubsystem.setHoodAngle(0.5375);
    }

    public boolean readyToShoot(ShooterCalculator.ShotSolution shotSolution, double headingToleranceDeg) {
        return shotSolution != null
                && Math.abs(Math.toDegrees(shotSolution.headingErrorRad)) <= headingToleranceDeg
                && flyWheelSubsystem.isAtSpeed(shotSolution.rpm);
    }

    public void idle() {
        flyWheelSubsystem.idle();
    }

    public void stop() {
        flyWheelSubsystem.stop();
    }
}
