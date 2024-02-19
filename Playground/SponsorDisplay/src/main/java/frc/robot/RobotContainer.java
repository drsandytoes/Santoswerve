// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;

import org.janksters.ExampleCommonClasses.Commands.ClearScreenCommand;
import org.janksters.ExampleCommonClasses.Drawing.BitmapFont;
import org.janksters.ExampleCommonClasses.Drawing.BitmapFontManager;
import org.janksters.ExampleCommonClasses.Subsystems.LEDPanelSubsystem;
import org.janksters.ExampleCommonClasses.Subsystems.LEDRegionSubsystem;


public class RobotContainer {
  private final LEDPanelSubsystem m_ledSubsystem;
  private final LEDRegionSubsystem m_upperHalf;
  private final LEDRegionSubsystem m_lowerHalf;

  private BitmapFontManager m_fontManager = new BitmapFontManager();


  public RobotContainer() {
    configureBindings();

    // Create the main display system
    m_ledSubsystem = new LEDPanelSubsystem(Constants.LEDConstants.kMatrixWidth, Constants.LEDConstants.kMatrixHeight, Constants.LEDConstants.kLEDPWMPin);
    m_ledSubsystem.brightness = RobotBase.isSimulation() ? 1.0 : Constants.LEDConstants.kBrightness;

    // Create the upper / lower half of the display
    m_upperHalf = new LEDRegionSubsystem(m_ledSubsystem, Constants.LEDConstants.LEDRegions.kUpperHalf);
    m_lowerHalf = new LEDRegionSubsystem(m_ledSubsystem, Constants.LEDConstants.LEDRegions.kLowerHalf);

    // Load the fonts we need
    File deployDirectory = Filesystem.getDeployDirectory();
    m_fontManager.addFont(new BitmapFont(new File(deployDirectory, Constants.Fonts.kSmallFontFile)));
    // m_fontManager.addFont(new BitmapFont(new File(deployDirectory, Constants.Fonts.kMediumFontFile)));
    m_fontManager.addFont(new BitmapFont(new File(deployDirectory, Constants.Fonts.kLargeFontFile)));

    var font = m_fontManager.getFont(Constants.Fonts.kSmallFontFile);

    var sponsors = new ArrayList<SponsorPanel>(Arrays.asList(
      new TextPanel("Top line", "0123456789", font).withColor(Color.kBlue),
      new TextPanel("No Bottom", null, font),
      new TextPanel("", "No Top", font),
      new TextPanel("A really long top line", "A moderately even longer bottom line", font),
      new ImagePanel(new File(deployDirectory, "48x16 Logo.png"))
    ));

    var repeatingSequence = new SequentialCommandGroup(new InstantCommand());
    for (SponsorPanel panel : sponsors) {
      repeatingSequence.addCommands(
        new ClearScreenCommand(m_ledSubsystem),
        new ParallelCommandGroup(
          panel.getDisplayCommand(m_ledSubsystem, m_upperHalf, m_lowerHalf),
          new WaitCommand(Constants.SponsorDisplay.kPanelDelay)
        )
      );
    }

    CommandScheduler.getInstance().schedule(repeatingSequence.repeatedly().ignoringDisable(true));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
