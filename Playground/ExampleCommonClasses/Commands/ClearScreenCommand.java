package org.janksters.ExampleCommonClasses.Commands;

import org.janksters.ExampleCommonClasses.Drawing.BitmapDrawingContext;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClearScreenCommand extends Command {
    private BitmapDrawingContext context;

    public  <T extends Subsystem & BitmapDrawingContext> ClearScreenCommand(T subsystem) {
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        context.clearScreen(Color.kBlack);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
