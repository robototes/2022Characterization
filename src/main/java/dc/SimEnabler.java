package dc;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;

public class SimEnabler implements Sendable {

  public SimEnabler() {
    DriverStationSim.setAutonomous(true);
  }

  public void setEnabled(boolean enabled) {
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
    DriverStation.isNewControlData();
    while (DriverStation.isEnabled() != enabled) {
      try {
        Thread.sleep(1);
      } catch (InterruptedException exception) {
        exception.printStackTrace();
      }
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Enabled", 
                               () -> DriverStation.isEnabled(), 
                               enabled -> setEnabled(enabled));
  }
}
