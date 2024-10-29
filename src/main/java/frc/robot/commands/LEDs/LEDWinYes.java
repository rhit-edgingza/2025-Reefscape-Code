package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDWinYes extends Command {
  /*
   * DIO LED Win Enable Command
   * --------------------------
   * 
   * This command hooks up to the LED subsystem and sets
   * the color to win.
   */

  private final LEDSubsystem m_subsystem;

  public LEDWinYes(LEDSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_subsystem.set_win(true);
  }
}
