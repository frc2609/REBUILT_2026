package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.io.motor.PositionMotorIO;
import frc.robot.Constants;
import frc.robot.subsystems.io.encoder.AbsEncoderIO;

/** Shooter Subsystem using velocity control (rotations per second). */
public class LedSubsystem extends SubsystemBase {
    private final int LedLength;
    private final AddressableLED Led;
    private final AddressableLEDBuffer ledBuffer;
    public final Timer deployedWait = new Timer();
    private boolean LedFree = false;

    public LedSubsystem(int LedLength, int LedPort) {
        Led = new AddressableLED(LedPort);
        ledBuffer = new AddressableLEDBuffer(LedLength);
        this.LedLength = LedLength;

        //sets a default pettern, to make it easier to detect full deploy
        for (int i=0;i<LedLength;i++) {
          ledBuffer.setHSV(i,0,255,10);
        }

        Led.setLength(LedLength);
        Led.setData(ledBuffer);
        Led.start();
        deployedWait.start();
    }

    public void SignalEndDeploy() {
        for(int i = 0; i<LedLength;i++){
      ledBuffer.setHSV(i, 60, 255, 10);
    }
    Led.setData(ledBuffer);
    LedFree = true;
    }

    public void pattern() {
        if (!LedFree) {
            return;
        }
        for(int i=0;i<LedLength;i++) {
            ledBuffer.setHSV(i, 0, 255, 10);
            Led.setData(ledBuffer);
            ledBuffer.setHSV(i, 0, 0, 0);
        }
        for(int i =0; i>0;i-=1){
            ledBuffer.setHSV(i, 0, 255, 10);
            Led.setData(ledBuffer);
            ledBuffer.setHSV(i, 0, 0, 0);
        }
    }

}
