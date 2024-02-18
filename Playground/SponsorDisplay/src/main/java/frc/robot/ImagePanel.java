package frc.robot;

import java.io.File;

import org.janksters.ExampleCommonClasses.Commands.DrawImageCommand;
import org.janksters.ExampleCommonClasses.Drawing.BitmapDrawingContext;
import org.janksters.ExampleCommonClasses.Drawing.DrawingSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ImagePanel implements SponsorPanel {
    private File imageFile;

    public ImagePanel(File file) {
        imageFile = file;
    }

    public <T extends Subsystem & BitmapDrawingContext> Command getDisplayCommand(T context, T contextUpper, T contextLower) {
        return new DrawImageCommand(imageFile, context);
    }
}
