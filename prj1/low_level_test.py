from pybricks.pupdevices import DCMotor
from pybricks.parameters import Port
from pybricks.tools import wait

# Initialize a motor without rotation sensors on Port A
motor = DCMotor(Port.A)

# Run at 50% power (duty cycle from -100 to 100)
motor.dc(50)
wait(2000)

# Stop the motor (coast)
motor.stop()