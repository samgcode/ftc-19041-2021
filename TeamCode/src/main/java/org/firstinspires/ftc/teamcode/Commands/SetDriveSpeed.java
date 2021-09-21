import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.utils.Vector;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class setDriveSpeedCommand extends CommandBase {
    DriveSubsystem driveSubsystem;
    Vector speed;

    boolean done = false;

    public SetDriveSpeed(DriveSubsystem subsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier h) {
        driveSubsystem = subsystem;
        speed = new Vector(x.getAsDouble(), y.getAsDouble(), h.getAsDouble())
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.setSpeed(speed);
        done = true;
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}