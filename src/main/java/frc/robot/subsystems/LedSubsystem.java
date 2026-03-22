package frc.robot.subsystems;

import org.littletonrobotics.junction.ConsoleSource.RoboRIO;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotController.RadioLEDState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Shooter Subsystem using velocity control (rotations per second). */
public class LedSubsystem extends SubsystemBase {
    private final int LedLength;
    private final AddressableLED Led;
    private final AddressableLEDBuffer ledBuffer;
    private final double travelTime;
    public final Timer deployedWait = new Timer();
    private boolean LedFree = false;

    public LedSubsystem(int LedLength, int LedPort, double travelTime) {
         Led = new AddressableLED(LedPort);
    ledBuffer = new AddressableLEDBuffer(LedLength);
    this.LedLength = LedLength;
    this.travelTime = travelTime;

    // sets a default pettern, to make it easier to detect full deploy
    for (int i = 0; i < LedLength; i++) {
      ledBuffer.setHSV(i, 0, 255, 10);
    }

    Led.setLength(LedLength);
    Led.setData(ledBuffer);
    Led.start();
    deployedWait.start();
  }

  public void SignalEndDeploy() {
    for (int i = 0; i < LedLength; i++) {
      ledBuffer.setHSV(i, 60, 255, 10);
    }
    Led.setData(ledBuffer);
    LedFree = true;
  }

  public void pattern() {
    if (!LedFree) {
      return;
    }
    int timerPixel = (int)(((LedLength/travelTime)* deployedWait.get()));
    if ((int)(timerPixel/LedLength)%2 == 0) {
      ledBuffer.setHSV(timerPixel%LedLength, 0, 255, 10);
      Led.setData(ledBuffer);
      ledBuffer.setHSV(timerPixel%LedLength, 0, 0, 0);
    }else{
      ledBuffer.setHSV( LedLength-(timerPixel%LedLength)-1, 0, 255, 10);
      Led.setData(ledBuffer);
      ledBuffer.setHSV( LedLength-(timerPixel%LedLength)-1, 0, 0, 0);
      
    }
  }
}
