package org.firstinspires.ftc.teamcode.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SUBSYSTEMS.DriveSubsystem;

import java.util.function.Supplier;

public class DriveCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final Supplier<Double> forward, strafe, rotate;


    public DriveCommand(DriveSubsystem drive,Supplier<Double> forward, Supplier<Double> strafe, Supplier<Double> rotate){
        this.drive = drive;
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;
        addRequirements(drive);
    }

    @Override
    public void execute() {

        drive.drive(forward.get(),strafe.get(),rotate.get());
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}
