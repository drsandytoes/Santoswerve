package frc.robot;

import org.janksters.ExampleCommonClasses.Commands.TextCommand;
import org.janksters.ExampleCommonClasses.Drawing.BitmapDrawingContext;
import org.janksters.ExampleCommonClasses.Drawing.BitmapFont;
import org.janksters.ExampleCommonClasses.Drawing.DrawingSubsystem;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TextPanel implements SponsorPanel {
    private String topLine;
    private String bottomLine;
    
    private BitmapFont font;
    private Color color;

    public TextPanel(String topLine, String bottomLine, BitmapFont font) {
        this.topLine = topLine != null ? topLine : "";
        this.bottomLine = bottomLine != null ? bottomLine : "";
        this.font = font;
        
        this.color = Color.kWhite;
    }

    public TextPanel withColor(Color color) {
        this.color = color;
        return this;
    }

    public <T extends Subsystem & BitmapDrawingContext> Command getDisplayCommand(T context, T contextUpper, T contextLower) {
        return new ParallelCommandGroup(
            new TextCommand(topLine, font, color, contextUpper),
            new TextCommand(bottomLine, font, color, contextLower)
        );
    }
}
