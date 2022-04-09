There are two proxy sensors.  One for either leg to prevent overtravel and to calibrate position.  A metal plate will come across the sensor to open the circuit providing a LOW signal to the MCU input pin.  We use Normally Closed signals in order to protect against power failure of the sensor.



Due to the desk design, there isn't a good way to protect against positive overtravel (too high).  One option would be to add pull switches.  However, we should be able to get high accuracy positioning, so we can add software safeties instead.  We will need to require recalibration every time we have a power cycle.