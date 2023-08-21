package frc.robot.utils;

public class CANDeviceID {
    public static final String kRioBusName = "rio";

    public String bus;
    public int id;

    /**
     * Create an object representing a device on the CAN bus
     * @param deviceID Device ID on the CAN bus
     * @param busName CAN bus name
     */
    public CANDeviceID(int deviceID, String busName) {
        this.id = deviceID;
        this.bus = busName;
    }

    /**
     * Create an object representing a device on the built-in (RIO) CAN bus
     * @param deviceID Device ID on the CAN bus
     */
    public CANDeviceID(int deviceID) {
        this(deviceID, kRioBusName);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        CANDeviceID that = (CANDeviceID) o;
        return this.id == that.id  && this.bus.equals(that.bus);
    }

}
