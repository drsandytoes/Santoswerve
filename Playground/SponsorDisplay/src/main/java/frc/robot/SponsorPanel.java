package frc.robot;


import org.janksters.ExampleCommonClasses.Drawing.BitmapDrawingContext;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SponsorPanel {
    public <T extends Subsystem & BitmapDrawingContext> Command getDisplayCommand(T context, T contextUpper, T contextLower);
}
