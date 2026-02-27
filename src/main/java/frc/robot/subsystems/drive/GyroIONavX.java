package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;


public class GyroIONavX implements GyroIO {

    private AHRS ahrs;
    private double last_yaw;

    public GyroIONavX() {
        System.out.println("GyroIONavx constructor");
        try {
            /***********************************************************************
             * navX-MXP:
             * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.
             * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
             * 
             * navX-Micro:
             * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
             * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
             * 
             * Multiple navX-model devices on a single robot are supported.
             ************************************************************************/
            // ahrs = new AHRS(SerialPort.Port.kUSB);
            ahrs = new AHRS(I2C.Port.kMXP);
            ahrs.enableLogging(true);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
        Timer.delay(1.0);
        last_yaw = 0.0;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = ahrs.isConnected();
        inputs.yawPosition = ahrs.getRotation2d();

        double yaw = ahrs.getYaw();
        double yawVelocity = Units.degreesToRadians(yaw - last_yaw) / 0.02;
        last_yaw = yaw;
        inputs.yawVelocityRadPerSec = yawVelocity;
    }

    @Override
    public void setYaw(Rotation2d yawAngle) {
        ahrs.setAngleAdjustment(yawAngle.getDegrees());
    }
}
