package org.firstinspires.ftc.teamcode;

/**
 * Created by l1581 on 12/15/2017.
 */

import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

        import com.qualcomm.robotcore.hardware.I2cAddr;
        import com.qualcomm.robotcore.hardware.I2cDevice;
        import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
        import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

        import static java.lang.Thread.sleep;

/**
 * Created by Administrator on 3/3/2017.
 */

public class UltraSonic {
    I2cDevice device;
    I2cDeviceSynch ultra_sonic2;
    private I2cAddr address;
    public UltraSonic(int address) {
        this.address = I2cAddr.create8bit(address);
    }
    public void initialize(I2cDevice device) {
        this.device = device;
        ultra_sonic2 = new I2cDeviceSynchImpl(device, this.address, false);
        ultra_sonic2.engage();
    }
    public double get() {
        byte a[];
        int b = 0;
        this.ultra_sonic2.write8(0x02, 0xb4);
        try {
            sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        a = this.ultra_sonic2.read(0x02, 0x02);
        try {
            sleep(20);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if ((int) a[1] < 0) {
            b = 256 + (a[1]);
        } else {
            b = (int) a[1];
        }
        return a[0] * 256 + b;
    }

}
