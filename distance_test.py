
import math

SCALE_FACTOR_CM = 1023.0 / 1300.48;

def pulseTOF(rawValue):
    return rawValue / 10000.0


def analogTOF(rawValue):
    return rawValue * 0.00735

temperature = 21.0

for i in range(0, 1023):
    timeOfFlight = analogTOF(i)
    distance = timeOfFlight * ((20.5 * math.sqrt(temperature + 273.15)) / 2.0)
    dist = i / SCALE_FACTOR_CM
    print 'Distance cm:', distance, dist

