Header header

# Joint name
string[4] name

# The actual position by the encoder value (rad)
float64[4] position

# The actual velocity measured by the encoder as the actual RPM value (rad/s)
float64[4] velocity

# The current flowing through the motor (A)
float64[4] current

# The actual temperature of motor (C)
int16[4] temperature