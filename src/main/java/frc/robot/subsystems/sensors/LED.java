package frc.robot.subsystems.sensors;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LED extends SubsystemBase{
    private static LED instance = null;
    public static synchronized LED getInstance() {
        if (instance == null) instance = new LED();
        return instance;
      }

  
    AddressableLED leds = new AddressableLED(0);
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(60);
    public void turnOn(LEDState state) {
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
        switch (state) {
            case Green:
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setRGB(i, 0, 255, 0);
                }
                break;

            case Red:
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setRGB(i, 255, 0, 0);
                }
                break;

            case Orange:
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setRGB(i, 255, 18, 0);
                }
                break;
        
        
            case BlueGreen:
                for (int i = 0; i < buffer.getLength(); i++) {
                    if (i % 2 == 0){
                        buffer.setRGB(i, 0, 255, 0);
                    }
                    else {
                        buffer.setRGB(i, 0, 0, 255);
                    }       
                }   
                break;
                
            case Intaked: 
                for (int i = 0; i < buffer.getLength(); i++) {
                    if (i > buffer.getLength() / 2){
                        buffer.setRGB(i, 0, 255, 0);
                    }
                    else {
                        buffer.setRGB(i, 0, 0, 255);
                    }
                }
                break;

            case Aiming:
                for (int i = 0; i < buffer.getLength(); i++) {
                    if (i % 2 == 0){
                        buffer.setRGB(i, 0, 255, 0);
                    }
                    else {
                        buffer.setRGB(i, 0, 0, 255);
                    }                
                }
                break;
            case Watermelon:
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setRGB(i, 0, 0, 0);     
                }
                break;  

            case Privileged:
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setRGB(i, 255, 255, 255);     
                }
                break;    

            case Dashboard:
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setRGB(i, (int) SmartDashboard.getNumber("R", 0), (int) SmartDashboard.getNumber("G", 0), (int) SmartDashboard.getNumber("B", 0));     
                }
                break;    
        }
         
         leds.setData(buffer);
    }

}