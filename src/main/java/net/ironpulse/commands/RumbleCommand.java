package net.ironpulse.commands;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class RumbleCommand extends Command {
    private final GenericHID hid;
    private final Timer timer = new Timer();
    private final Measure<Time> rumbleTime;

    public RumbleCommand(GenericHID hid, Measure<Time> seconds) {
        this.hid = hid;
        this.rumbleTime = seconds;
    }

    @Override
    public void initialize() {
        timer.restart();
        hid.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    }

    @Override
    public void end(boolean interrupted) {
        hid.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(rumbleTime.magnitude());
    }
}
