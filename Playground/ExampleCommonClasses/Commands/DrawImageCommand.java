package org.janksters.ExampleCommonClasses.Commands;

import java.io.File;

import javax.imageio.ImageIO;

import java.awt.image.BufferedImage;

import org.janksters.ExampleCommonClasses.Drawing.BitmapDrawing;
import org.janksters.ExampleCommonClasses.Drawing.BitmapDrawingContext;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DrawImageCommand extends Command {
    private BufferedImage image;
    private BitmapDrawingContext context;

    public  <T extends Subsystem & BitmapDrawingContext> DrawImageCommand(File imageFile, T subsystem) {
        image = loadImage(imageFile);
        context = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        BitmapDrawing.setImage(image, context);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    private BufferedImage loadImage(File imageFile) {
        BufferedImage loadedImage;

        try {
            loadedImage = ImageIO.read(imageFile);
            System.out.println("Successfully loaded image file!");
            System.out.println("Image dimensions are: " + loadedImage.getWidth() + " x " + loadedImage.getHeight());
        }
        catch(Exception e) {
            System.out.println("Couldn't load image: " + imageFile.getPath());
            return null;
        }

        return loadedImage;
    }
}
