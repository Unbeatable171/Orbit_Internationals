package org.firstinspires.ftc.teamcode.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SUBSYSTEMS.FlyWheelSubsystem;

public class FlyWheelCommand extends CommandBase {

    public enum State {SPINUP, IDLE, STOP};


    private final FlyWheelSubsystem flywheel;
    private final State state;
    private final double targetRPM;
    private final double hoodAngle;

    public FlyWheelCommand (FlyWheelSubsystem flywheel,State state, double targetRPM, double hoodAngle){
        this.flywheel = flywheel;
        this.state = state;
        this.targetRPM = targetRPM;
        this.hoodAngle = hoodAngle;
        addRequirements(flywheel);
    }

    @Override
    public void execute() {
        switch (state){
            case SPINUP:
                flywheel.spinUp(targetRPM);
                flywheel.setHoodAngle(hoodAngle);
                break;
            case IDLE:
                flywheel.idle();
                break;
            case STOP:
                flywheel.stop();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
