package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private final int ledLength;
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    public final Timer deployedWait = new Timer();
    private boolean ledFree = false;

    public LedSubsystem(int ledLength, int ledPort) {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledLength);
        this.ledLength = ledLength;

        //sets a default pettern, to make it easier to detect full deploy
        for (int i=0;i<ledLength;i++) {
          ledBuffer.setHSV(i,0,255,10);
        }

        led.setLength(ledLength);
        led.setData(ledBuffer);
        led.start();
        deployedWait.start();
    }

    public void SignalEndDeploy() {
        for(int i = 0; i<ledLength;i++){
            ledBuffer.setHSV(i, 60, 255, 10);
        }

        led.setData(ledBuffer);
        ledFree = true;
    }

    public void pattern() {
        if (!ledFree) {
            return;
        }
        for(int i=0;i<ledLength;i++) {
            ledBuffer.setHSV(i, 0, 255, 10);
            led.setData(ledBuffer);
            ledBuffer.setHSV(i, 0, 0, 0);
        }
        for(int i =0; i>0;i-=1){
            ledBuffer.setHSV(i, 0, 255, 10);
            led.setData(ledBuffer);
            ledBuffer.setHSV(i, 0, 0, 0);
        }
    }
}
