package org.firstinspires.ftc.teamcode.ACTIONS;

import org.firstinspires.ftc.teamcode.SUBSYSTEMS.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.ShooterCalculator;

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


    public void idle() {
        flyWheelSubsystem.idle();
    }

//    public void idlefar() {
//        flyWheelSubsystem.idlefar();
//    }

    public void stop() {
        flyWheelSubsystem.stop();
    }
}
