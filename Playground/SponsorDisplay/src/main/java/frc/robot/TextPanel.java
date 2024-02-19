package frc.robot;

import org.janksters.ExampleCommonClasses.Commands.ScrollingTextCommand;
import org.janksters.ExampleCommonClasses.Commands.TextCommand;
import org.janksters.ExampleCommonClasses.Drawing.BitmapDrawingContext;
import org.janksters.ExampleCommonClasses.Drawing.BitmapFont;
import org.janksters.ExampleCommonClasses.Drawing.DrawingSubsystem;
import org.janksters.ExampleCommonClasses.Drawing.Point;

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
        if (topLine.length() > 10 || bottomLine.length() > 10) {
            return new ParallelCommandGroup(
                new ScrollingTextCommand(topLine, new Point(0, 0), 8.0, font, color, contextUpper),
                new ScrollingTextCommand(bottomLine, new Point(0, 0), 8.0, font, color, contextLower)
            );
        } else {
            return new ParallelCommandGroup(
                new TextCommand(topLine, font, color, contextUpper),
                new TextCommand(bottomLine, font, color, contextLower)
            );
        }
    }
}
