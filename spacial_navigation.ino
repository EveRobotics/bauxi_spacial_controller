
#include <SPI.h>
#include "moving_average.h"


static inline int sign(int val) {
    if (val < 0) return -1;
    return 1;
}

static inline int sign(float val) {
    if (val < 0.0) return -1.0;
    return 1.0;
}


void(* resetFunc) (void) = 0; //declare reset function @ address 0

//==============================================================================
// PIN DECLARATIONS:
//==============================================================================

enum ProcessorPins {
    // Mux 1 Pins, Inputs to processor from analog distance sensors:
    MUX_ANALOG_IN_S0 = 18,
    MUX_ANALOG_IN_S1 = 19,
    MUX_ANALOG_IN_S2 = 20,
    MUX_ANALOG_IN_S3 = 21,
    MUX_ANALOG_IN_SIG = A8, // Analog input for distance sensors and power monitoring.
    // Mux 2 Pins, Outputs from processor to chip select pins and UI elements:
    MUX_DIGITAL_OUT_S0 = 1,
    MUX_DIGITAL_OUT_S1 = 0,
    MUX_DIGITAL_OUT_S2 = 2,
    // int MUX2_S3 = NA, pull low with pull-down resistor to access first eight channels.
    MUX_DIGITAL_OUT_SIG = 3, // Digital output signal for chip selects and UI indicators.
    // Left motor (motor 1) speed controller outputs:
    MOTOR_PWM_LEFT = 9,
    MOTOR_CTRL1_LEFT = 4,
    MOTOR_CTRL2_LEFT = 5,
    // Right motor (motor 2) speed controller outputs:
    MOTOR_PWM_RIGHT = 10,
    MOTOR_CTRL1_RIGHT = 6,
    MOTOR_CTRL2_RIGHT = 7,
    // SPI bus I/O:
    SPI_MOSI = 16,
    SPI_MISO = 14,
    SPI_SERIAL_CLK = 15,
};

// Used for addressing the channels of the digital output multiplexer (mux2):
enum DigitalMuxChannels {
    C0_ENCODER_CNT_LEFT_CS = 0, // Chip select for the first (left) encoder counter.
    C1_ENCODER_CNT_RIGHT_CS = 1, // Chip select for the right side encoder counter.
    // Sonar range-finder sonar transmit enable. Prevents interference...
    C2_SONAR_FRONT_ENABLE = 2,
    C3_SONAR_LEFT_ENABLE = 3,
    C4_SONAR_RIGHT_ENABLE = 4,
    C5_SONAR_BACK_ENABLE = 5,
    // Enumerate digital status outputs:
};

// Used for addressing the channels of the analog input multiplexer (mux1):
enum AnalogMuxChannels {
    C0_SONAR_FRONT = 0, // Front sonar
    C1_SONAR_LEFT = 1, // 
    C2_SONAR_RIGHT = 2,
    C3_SONAR_BACK = 3,
    C4_IR_DIST_FRONT = 4,
    C5_IR_DIST_LEFT = 5,
    C6_IR_DIST_RIGHT = 6,
    C7_IR_DIST_BACK = 7,
    // Enumerate battery power monitoring channels:
    // Mux analog resolution may not be sufficient for current sensing:
    // i.e. voltage drop across a 0.1 Ohm resistor, if so, free 2 inputs
    // for user inputs (switches or potentiometers, or bump switches).
    // Enumerate User input controls:
    C8_RADIO_A = 8,
    C9_RADIO_B = 9,
    C10_RADIO_C = 10,
    C11_RADIO_D = 11,
};

enum MotorDirections {
    FORWARD = 0,
    REVERSE = 1,
    SHORT_BRAKE = 2,
    STOP = 3,
};

//==============================================================================
// ENCODER COUNTER:
//==============================================================================

/**
32-BIT QUADRATURE COUNTER WITH SERIAL INTERFACE

Class Implementation for the LS7366R 32-bit CMOS counter, with direct interface 
for quadrature clocks from incremental encoders. It also interfaces with the 
index signals from incremental encoders to perform variety of marker functions.
*/
class EncoderCounter {

public:

    EncoderCounter(int mosi, int miso, int serialClock, int chipSelect) {
        // https://www.arduino.cc/en/Reference/SPI
        // Initialize Pins:
        // MOSI (RXD) (Pin 7)
        // MISO (TXD) (Pin 6)
        // SCK (Pin 5)
        _mosiRxdPin = mosi;
        _misoTxdPin = miso;
        _serialClockPin = serialClock; 
        _chipSelectPin = chipSelect;
        SPI.begin();
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        pinMode(_chipSelectPin, OUTPUT);
        // TODO: determine the correct states to assert and de-assert CS.
        digitalWrite(_chipSelectPin, HIGH);
        setOperatingMode();
    }

    /**
    TODO: make this member function general purpose by parameterizing it with 
    mode register initialization arguments.
    */
    void setOperatingMode() {
        // In our particular case, we want to setup the following parameters:
        // MDR0:
        // B1 B0 = 01: x1 quadrature count mode (one count per quadrature cycle). 
        // B3 B2 = 00: free-running count mode.
        // B5 B4 = 00: disable index.
        // B6 = 0: Asynchronous Index
        // B7 = 0: Filter clock division factor = 1 
        // 00000001
        _regMdr0Value = 1;
        // MDR1:
        // B1 B0 = 00:  = 00: 4-byte counter mode 
        // B2 = 0: Enable counting 
        // B3 =  : not used (set to 0)
        // B7-B7: NOP (set to 0)
        _regMdr1Value = 0;
        // Write the registers:
        // Write MDR0: Set IR to B2-B0 = 000, B5-B3 = 001: Select MDR0, 
        // and B7-B6: = 10: WR register: 0b10001000 = 136.
        _regIrValue = 136;
        digitalWrite(_chipSelectPin, LOW);
        SPI.transfer(_regIrValue);
        SPI.transfer(_regMdr0Value);
        // We don't need to set MDR1 to 0 since it is set to zero at power-up.
        digitalWrite(_chipSelectPin, HIGH);
        // We should set the initial counts to a non zero number say 100,000
        writeCounts(100000);
    }

    unsigned long readCounts() { 
        // Set the instruction register to load OTR with CNTR
        // 0b11101000
        _regIrValue = 0xe8; // dec: 232
        digitalWrite(_chipSelectPin, LOW);
        SPI.transfer(_regIrValue);
        digitalWrite(_chipSelectPin, HIGH);
        digitalWrite(_chipSelectPin, LOW);
        // Set the instruction register to read OTR
        // 0b01101000
        _regIrValue = 0x68; // dec: 104
        SPI.transfer(_regIrValue);
        // Read OTR (4 bytes).
        unsigned long counts = 0;
        unsigned char b0, b1, b2, b3;

        b0 = ~SPI.transfer(0);
        b1 = ~SPI.transfer(0);
        b2 = ~SPI.transfer(0);
        b3 = ~SPI.transfer(0);
        digitalWrite(_chipSelectPin, HIGH);

        counts = (b0 << 24) + (b1 << 16) + (b2 << 8) + b3;

        return counts; 
    }

    /*
     *
     */
    unsigned long writeCounts(unsigned long count) {

        unsigned char b0 = 0xFF & (count >> 24);
        unsigned char b1 = 0xFF & (count >> 16);
        unsigned char b2 = 0xFF & (count >> 8);
        unsigned char b3 = 0xFF & count;
        // Set the instruction register to write DTR
        // 0b10011000
        _regIrValue = 0x98;
        digitalWrite(_chipSelectPin, LOW);
        SPI.transfer(_regIrValue);
        //digitalWrite(_chipSelectPin, HIGH);
        //digitalWrite(_chipSelectPin, LOW);
        SPI.transfer(b0);
        SPI.transfer(b1);
        SPI.transfer(b2);
        SPI.transfer(b3);
        digitalWrite(_chipSelectPin, HIGH);
        // Set the instruction register to load DTR to CNTR
        // 0b11100000
        _regIrValue = 0xe0;
        digitalWrite(_chipSelectPin, LOW);
        SPI.transfer(_regIrValue);
        digitalWrite(_chipSelectPin, HIGH);
        return readCounts();
    }

    void clearCounts() { }

    void readStatus() { }

private:
    
    /**
     * MOSI (RXD) (Pin 7 on the LS7366R chip):
     * Input. Serial output data from the host processor is shifted into the 
     * LS7366R at this input.
     */
    int _mosiRxdPin = 0;

    /**
     * MISO (TXD) (Pin 6):
     * Output. Serial output data from the LS7366R is shifted out on the MISO 
     * (Master In Slave Out) pin. The MISO output goes into high impedance 
     * state when SS/ input is at logic high, providing multiple slave-unit 
     * serial outputs to be wire-ORed.
     */
    int _misoTxdPin = 0;

    /** 
     * SS/ (Pin 4):
     * A high to low transition at the SS/ (Slave Select) input selects the 
     * LS7366R for serial bi-directional data transfer; a low to high 
     * transition disables serial data transfer and brings the MISO output to 
     * high impedance state. This allows for the accommodation of multiple 
     * slave units on the serial I/O.
     */
    int _chipSelectPin = 0;

    /**
     * SCK (Pin 5):
     * Input. The SCK input serves as the shift clock input for transmit- ting 
     * data in and out of LS7366R on the MOSI and the MISO pins, respectively. 
     * Since the LS7366R can operate only in the slave mode, the SCK signal is 
     * provided by the host processor as a means for synchronizing the serial 
     * transmission between itself and the slave LS7366R.
     */
    int _serialClockPin = 0;

    /** REGISTERS:
     * The following is a list of LS7366R internal registers:
     * Upon power-up the registers DTR, CNTR, STR, MDR0 and MDR1 are reset to zero.
     *
     * DTR. The DTR is a software configurable 8, 16, 24 or 32-bit input data 
     * register which can be written into directly from MOSI, the serial input. 
     * The DTR data can be transferred into the 32-bit counter (CNTR) under 
     * program control or by hardware index signal. The DTR can be cleared to 
     * zero by software control. In certain count modes, such as modulo-n and 
     * range-limit, DTR holds the data for "n" and the count range, 
     * respectively. In compare operations, whereby compare flag is set, the 
     * DTR is compared with the CNTR.
     */
    unsigned int _regDtrValue = 0;

    /**
     * CNTR. The CNTR is a software configurable 8, 16, 24 or 32-bit up/down 
     * counter which counts the up/down pulses resulting from the quadrature 
     * clocks applied at the A and B inputs, or alternatively, in 
     * non-quadrature mode, pulses applied at the A input. By means of IR 
     * instructions the CNTR can be cleared, loaded from the DTR or in turn, can 
     * be transferred into the OTR.
     */
    unsigned int _regCntrValue = 0;
    
    /** OTR. The OTR is a software configuration 8, 16, 24 or 32-bit register 
     * which can be read back on the MISO output. Since instantaneous CNTR 
     * value is often needed to be read while the CNTR continues to count, the 
     * OTR serves as a convenient dump site for instantaneous CNTR data which 
     * can then be read without interfering with the counting process.
     */
    unsigned int _regOtrValue = 0;

    /**
     * STR. The STR is an 8-bit status register which stores count related 
     * status information.
     * CY:    Carry (CNTR overflow) latch
     * BW:    Borrow (CNTR underflow) latch
     * CMP:    Compare (CNTR = DTR) latch IDX: Index latch
     * CEN:    Count enable status: 0: counting disabled, 1: counting enabled
     * PLS:    Power loss indicator latch; set upon power up
     * U/D:    Count direction indicator: 0: count down, 1: count up
     * S:    Sign bit. 1: negative, 0: positive
     * -----------------------------------------
     * CY | BW | CMP | IDX | CEN | PLS | U/D | S
     * -----------------------------------------
     *  7 |  6 |   5 |   4 |   3 |   2 |   1 | 0
     */
    unsigned char _regStrValue = 0;

    /**
        * IR. The IR is an 8-bit register that fetches instruction bytes from the 
     * received data stream and executes them to perform such functions as 
     * setting up the operating mode for the chip (load the MDR) and data 
     * transfer among the various registers.
     * B2 B1 B0 = XXX (Don’t care) 
     * B5 B4 B3 = 000: Select none 
     *          = 001: Select MDR0
     *          = 010: Select MDR1 
     *          = 011: Select DTR
     *          = 100: Select CNTR
     *          = 101: Select OTR
     *          = 110: Select STR
     *          = 111: Select none
     * B7 B6 = 00: CLR register 
     *       = 01: RD register
     *       = 10: WR register
     *       = 11: LOAD register
     *
     * The actions of the four functions, CLR, RD, WR and LOAD are below:
     * Number of Bytes | OP Code | Register(s) | Operation
     * ------------------------------------------------------------------------
     *               1 |     CLR |        MDR0 | Clear MDR0 to zero
     *                 |         |        MRD1 | Clear MDR1 to zero
     *                 |         |        DTR  | None
     *                 |         |        CNTR | Clear CNTR to zero
     *                 |         |        OTR  | None
     *                 |         |        STR  | Clear STR to zero 
     * ------------------------------------------------------------------------
     *          2 to 5 |      RD |        MDR0 | Output MDR0 serially on TXD (MISO)
     *                 |         |        MDR1 | Output MDR1 serially on TXD (MISO)
     *                 |         |        DTR  | None
     *                 |         |        CNTR | Transfer CNTR to OTR, then output OTR serially on TXD (MISO)
     *                 |         |        OTR  | Output OTR serially on TXD (MISO)
     *                 |         |        STR  | Output STR serially on TXD (MISO)
     * ------------------------------------------------------------------------
     *          2 to 5 |      WR |        MDR0 | Write serial data at RXD (MOSI) into MDR0
     *                 |         |        MDR1 | Write serial data at RXD (MOSI) into MDR1
     *                 |         |        DTR  | Write serial data at RXD (MOSI) into DTR
     *                 |         |        CNTR | None
     *                 |         |        OTR  | None
     * ------------------------------------------------------------------------
     *               1 |    LOAD |        STR  | None
     *                 |         |        MDR0 | None
     *                 |         |        MDR1 | None
     *                 |         |        DTR  | None
     *                 |         |        CNTR | Transfer DTR to CNTR in "parallel"
     *                 |         |        OTR  | Transfer CNTR to OTR in "parallel"
     * ------------------------------------------------------------------------
     */
    unsigned char _regIrValue = 0;


    /** 
     * MDR0. The MDR0 (Mode Register 0) is an 8-bit read/write register that
     * sets up the operating mode for the LS7366R. The MDR0 is written into by
     * executing the "write-to-MDR0" instruction via the instruction register.
     * Upon power up MDR0 is cleared to zero. The following is a breakdown of 
     * the MDR bits:
     *
     * B1 B0 = 00: non-quadrature count mode. (A = clock, B = direction).
     *       = 01: x1 quadrature count mode (one count per quadrature cycle). 
     *       = 10: x2 quadrature count mode (two counts per quadrature cycle). 
     *       = 11: x4 quadrature count mode (four counts per quadrature cycle).
     * 
     * B3 B2 = 00: free-running count mode.
     *       = 01: single-cycle count mode (counter disabled with carry or 
     *             borrow, re-enabled with reset or load). 
     *       = 10: range-limit count mode (up and down count-ranges are limited
     *             between DTR and zero, respectively; counting freezes at 
     *             these limits but resumes when direction reverses). 
     *       = 11: modulo-n count mode (input count clock frequency is divided 
     *             by a factor of (n+1), where n = DTR, in both up and down directions).
     * B5 B4 = 00: disable index.
     *       = 01: configure index as the "load CNTR" input (transfers DTR to CNTR). 
     *       = 10: configure index as the "reset CNTR" input (clears CNTR to 0).
     *       = 11: configure index as the "load OTR" input (transfers CNTR to OTR).
     * B6 = 0: Asynchronous Index
     *    = 1: Synchronous Index (overridden in non-quadrature mode)
     * B7 = 0: Filter clock division factor = 1 
     *    = 1: Filter clock division factor = 2
     */
    unsigned char _regMdr0Value = 0;

    /**
     * MDR1. The MDR1 (Mode Register 1) is an 8-bit read/write register which 
     * is appended to MDR0 for additional modes. Upon power-up MDR1 is cleared 
     * to zero.
     *
     * B1 B0 = 00:  = 00: 4-byte counter mode 
     *       = 01: 3-byte counter mode 
     *       = 10: 2-byte counter mode. 
     *       = 11: 1-byte counter mode
     *
     * B2 = 0: Enable counting 
     *    = 1: Disable counting
     * B3 =  : not used 
     * B4 = 0: NOP
     *    = 1: FLAG on IDX (B4 of STR)------
     * B5 = 0: NOP                         |
     *    = 1: FLAG on CMP (B5 of STR)     | NOTE: Applicable to both
     * B6 = 0: NOP                         |       LFLAG/ and DFLAG/
     *    = 1: FLAG on BW (B6 of STR)      |
     * B7 = 0: NOP                         |
     *    = 1: FLAG on CY (B7 of STR) _____|
     */
    unsigned char _regMdr1Value = 0;

};

//==============================================================================
// MOTOR DRIVER:
//==============================================================================

/**

Toshiba Bi-CD Integrated Circuit: TB6612FNG 
Driver IC for Dual DC motor

In1    In2  | Out1  Out2  Mode
_________________________________________
high   High | Low   Low   Short Brake
Low    High | Low   High  Counter Clockwise
High   Low  | High  Low   Clockwise
Low    Low  | High  High  Stop

*/
class MotorDriver {

public:

    MotorDriver(int pwmPin, int control1Pin, int control2Pin) {
        _pwmPin = pwmPin;
        _control1Pin = control1Pin;
        _control2Pin = control2Pin;

        pinMode(_pwmPin, OUTPUT);
        pinMode(_control1Pin, OUTPUT);
        pinMode(_control2Pin, OUTPUT);
        // Set the initial state of the motor:
        setDirection(STOP);
        setSpeed(0);
    }

    /**
    Speed as duty cycle between 0 and 255.
    */
    void setSpeed(int speed) {
        // Set the duty cycle of the motor driver output.
        // https://www.arduino.cc/en/Tutorial/PWM
        analogWrite(_pwmPin, speed);
    }

    void setDirection(int direction) {
        // Set the direction of the motor driver output.
        if(direction == FORWARD) {
            digitalWrite(_control1Pin, HIGH);
            digitalWrite(_control2Pin, LOW);
        } else if(direction == REVERSE) {
            digitalWrite(_control1Pin, LOW);
            digitalWrite(_control2Pin, HIGH);
        } else if(direction == SHORT_BRAKE) {
            digitalWrite(_control1Pin, HIGH);
            digitalWrite(_control2Pin, HIGH);
        } else if(direction == STOP) {
            digitalWrite(_control1Pin, LOW);
            digitalWrite(_control2Pin, LOW);
        }
        // Else, do nothing, invalid direction argument.
    }

private:

    int _pwmPin = 0;
    int _control1Pin = 0;
    int _control2Pin = 0;

};

//==============================================================================
// MOTOR SPEED CONTROLLER:
//==============================================================================

/**
Motor speed controller, uses the motor driver and encoder counter as feedback
to implement a speed control feedback loop. Implements a PID loop.

TODO: set configurable constants 
*/
class SpeedController {

public:

    SpeedController(MotorDriver *driver, EncoderCounter *counter) {
        _driver = driver;
        _counter = counter;
    }

    void setSpeed(int speed) {
        // Set the speed we want to maintain, we use a feedback loop to make 
        // sure we stay at the desired speed.
        _setPoint = speed;
    }

    /*
     * Delta counts over delta time, delta time is in microseconds.
    */
    void updateControlLoop(int deltaCounts, unsigned long deltaTime) {
        
        _deltaCounts = deltaCounts;
        // Convert delta time to seconds.
        _deltaTime = (float)deltaTime / 100000.0;
        // The process variable is the counts per second.
        _processVariable = (float)deltaCounts / _deltaTime;
        // Calculate the current error:
        _currentError = _setPoint - _processVariable;

        if(_processVariable != REST_POINT_PROCESS 
            && _prevProcessVariable == REST_POINT_PROCESS) {
            // Adaptive static friction calibration:
            _staticFrictionTerm = _controlVariable;
            if(_staticFrictionTerm > MAXIMUM_CONTROL_INPUT / 3.0) {
                _staticFrictionTerm = MAXIMUM_CONTROL_INPUT / 3.0;
            } else if(_staticFrictionTerm < MINIMUM_CONTROL_INPUT / 3.0) {
                _staticFrictionTerm = MINIMUM_CONTROL_INPUT / 3.0;
            }

        }

        if(_setPoint != 0.0) {
            _staticFrictionTerm = abs(_staticFrictionTerm) * sign(_setPoint);
        }

        // Calculate the proportional term:
        _proportionalTerm = _proportionalGain * _currentError;
        
        _proportionalTermSum += _currentError * _deltaTime;

        // Prevent integral windup:
        if(_integralGain * _proportionalTermSum > MAXIMUM_CONTROL_INPUT) {
            _proportionalTermSum = MAXIMUM_CONTROL_INPUT / _integralGain;
        } else if(_integralGain * _proportionalTermSum < MINIMUM_CONTROL_INPUT) {
            _proportionalTermSum = MINIMUM_CONTROL_INPUT / _integralGain;
        }

        if(sign(_setPoint) != sign(_prevSetPoint)
            || (_processVariable == REST_POINT_PROCESS
            && sign(_setPoint) != sign(_proportionalTermSum))) {
            // The direction changed, set integral to zero. Or the process 
            // variable is at rest (got stuck). 
            _proportionalTermSum = REST_POINT_PROCESS;
        }

        _integralTerm = _integralGain * _proportionalTermSum;

        if(_processVariable != REST_POINT_PROCESS 
            && _prevProcessVariable == REST_POINT_PROCESS) {
            // Adaptive static friction calibration:
            _integralTerm -= _staticFrictionTerm;
        }

        // Calculate the derivative term:
        _derivativeTerm = _derivativeGain 
            * ((_currentError - _previousError) / _deltaTime);
        
        // Sum the terms and we have our control variable.
        _controlVariable = _staticFrictionTerm + _proportionalTerm 
            + _integralTerm + _derivativeTerm;

        if(_controlVariable > MAXIMUM_CONTROL_INPUT) {
            _controlVariable = MAXIMUM_CONTROL_INPUT;
        } else if(_controlVariable < MINIMUM_CONTROL_INPUT) {
            _controlVariable = MINIMUM_CONTROL_INPUT;
        }
        // There is nothing to be done.
        if((_setPoint == REST_POINT_PROCESS && _prevSetPoint != REST_POINT_PROCESS)
            || (_setPoint == REST_POINT_PROCESS && _processVariable == REST_POINT_PROCESS)) {
            // input a rest value to the controller.
            _controlVariable = REST_POINT_CONTROL;
            _proportionalTermSum = REST_POINT_PROCESS;
        }
        //printDebug();
        // Set direction:
        if(_setPoint == REST_POINT_PROCESS) {
            _driver->setDirection(STOP);
        } else if(_controlVariable < REST_POINT_CONTROL) {
            _driver->setDirection(REVERSE);
        } else if (_controlVariable > REST_POINT_CONTROL){
            _driver->setDirection(FORWARD);
        }

        // Scale the control variable?:

        
        _driver->setSpeed(abs((int)_controlVariable));
        _prevSetPoint = _setPoint;
        _previousError = _currentError;
        _prevProcessVariable = _processVariable;
    }

    int getProcessVariable() {
        return (int)_processVariable;
    }

    int getSetPoint() {
        return (int)_setPoint;
    }

    int getControlVariable() {
        return (int)_controlVariable;
    }

private:
    MotorDriver *_driver;
    EncoderCounter *_counter;

    const float REST_POINT_PROCESS = 0.0;
    const float REST_POINT_CONTROL = 0.0;

    const float MAXIMUM_CONTROL_INPUT = 255.0;
    const float MINIMUM_CONTROL_INPUT = -255.0;

    float _prevProcessVariable = 0.0;
    // A process variable, process value or process parameter is the current 
    // status of a process under control.
    float _processVariable = 0.0;
    
    float _prevSetPoint = 0.0; // The desired value.

    float _setPoint = 0.0; // The desired value.
    float _controlVariable = 0.0; // The output of the PID controller.
    float _currentError = 0.0; // Difference between process variable and the set point.
    // Used for computing the derivative term:
    float _previousError = 0.0;

    // PID Loop gains
    float _proportionalGain = 4.0;
    float _integralGain = 0.8;
    float _derivativeGain = 0.5;

    float _proportionalTermSum = 0.0;

    int _deltaCounts = 0;
    float _deltaTime = 0.0;

    float _staticFrictionTerm = 63.0;

    float _proportionalTerm = 0.0;
    float _integralTerm = 0.0;
    float _derivativeTerm = 0.0;

    
    void printDebug(void) {
        Serial.print("Delta Counts: ");
        Serial.println(_deltaCounts);


        Serial.print("Delta Time: ");
        Serial.println(_deltaTime);


        Serial.print("Set Point: ");
        Serial.println(_setPoint);


        Serial.print("Process Variable: ");
        Serial.println(_processVariable);


        Serial.print("Current Error: ");
        Serial.println(_currentError);

        
        Serial.print("StaticFriction Term: ");
        Serial.println(_staticFrictionTerm);


        Serial.print("Proportional Term: ");
        Serial.println(_proportionalTerm);


        Serial.print("Integral Term: ");
        Serial.println(_integralTerm);


        Serial.print("Derivative Term: ");
        Serial.println(_derivativeTerm);


        Serial.print("Control Variable: ");
        Serial.println(_controlVariable);
    }
};

//==============================================================================
// ANALOG MULTIPLEXER:
//==============================================================================

/**
Analog mux for reading sonar and IR and enabling the sonar transmit.
*/
class AnalogMultiplexer {

public:

    AnalogMultiplexer(int s0Pin, int s1Pin, int s2Pin, int s3Pin) {
        _s0Pin = s0Pin;
        _s1Pin = s1Pin;
        _s2Pin = s2Pin;
        _s3Pin = s3Pin;

        if(_s0Pin > -1) {
            pinMode(_s0Pin, OUTPUT);
        }
        if(_s1Pin > -1) {
            pinMode(_s1Pin, OUTPUT);
        }
        if(_s2Pin > -1) {
            pinMode(_s2Pin, OUTPUT);
        }
        if(_s3Pin > -1) {
            pinMode(_s3Pin, OUTPUT);
        }

        selectChannel(0);
    }
    

    void selectChannel(int channel) {
        _channel = channel;
        // Decode a nibble:
        int s0 = channel & 1;
        int s1 = (channel >> 1) & 1;
        int s2 = (channel >> 2) & 1;
        int s3 = (channel >> 3) & 1;

        //Serial.print("channel: ");
        //Serial.println(channel);
        
        if(_s0Pin > -1) {
            digitalWrite(_s0Pin, s0);
        }
        if(_s1Pin > -1) {
            digitalWrite(_s1Pin, s1);
        }
        if(_s2Pin > -1) {
            digitalWrite(_s2Pin, s2);
        }
        if(_s3Pin > -1) {
            digitalWrite(_s3Pin, s3);
        }
    }

private:

    int _s0Pin = 0;
    int _s1Pin = 0;
    int _s2Pin = 0;
    int _s3Pin = 0;

    int _channel = 0;
};


//==============================================================================
// RANGE FINDER (parent class):
//==============================================================================

/**
*/
class Rangefinder {

public:

    virtual float readDistance();

    virtual float getDistance();

    virtual int getRawDistance();

protected:
    int _signalPin = 0;

    int _rawValue = 0;
    float _distance = 0;

private:

};

//==============================================================================
// INFRARED RANGEFINDER:
//==============================================================================

class IrRangefinder : Rangefinder {

public:

    float readDistance() {
        // Read the analog input, and calculate the distance.
        // TODO: implement function for calculating distance .
        return _distance;
    }

    float getDistance() {
        // Get the distance without performing a measurement.
        return _distance;
    }

    int getRawDistance() {
        // Get the distance without performing a measurement.
        return _rawValue;
    }


private:

};


//==============================================================================
// SONAR RANGEFINDER:
//==============================================================================

/**
LV-MaxSonar-EZTM Series Sonar Rangefinder class implementation.

TODO: inherit from class Rangefinder and implement common functionality 
for both sonar and infrared range-finders in the base class.

Common functionality may include input buffering and averaging
*/
class SonarRangefinder : Rangefinder {

public:

    /**
    Initialize the sonar enable pin (Pin4 RX) and the analog signal output pin
    (Pin3 AN).

    Measures 0 - 254 inches. (645.16 cm) 
    Speed of sound is 340.29 m / s and 34029 cm / s

    34029 / 645.16 cm = 52.7450554901

    1 s / 52.7450554901 = 0.0189591231 seconds
    Double the time for the reflected signal.
    0.0189591231 * 2 = 0.0379182462 = 37.92 ms, 37918.2 us.
    */
    SonarRangefinder(int enablePin, int signalPin) {
        _enablePin = enablePin;
        _signalPin = signalPin;
        // https://www.arduino.cc/en/Tutorial/DigitalPins
        // https://www.arduino.cc/en/Reference/PinMode
        pinMode(_enablePin, OUTPUT);
        // This pin is internally pulled high. The LV-MaxSonar-EZ will 
        // continually measure range and output if RX data is left unconnected 
        // or held high. If held low the sensor will stop ranging. Bring high 
        // for 20uS or more to command a range reading.
        digitalWrite(_enablePin, LOW);
        // We do not need to initialize the HW for the analog input pin.
    }

    float readDistance() {
        // Assert the enable pin for 20 microseconds or more, then perform a
        // reading on the signal pin. Return the reading
        digitalWrite(_enablePin, HIGH);
        delayMicroseconds(SAMPLE_DELAY);
        // Enable reading for enough time for at least a single sample at
        // maximum range.
        _rawValue = 0;
        for(int i = 0; i < SAMPLE_COUNT; i++) {
            delayMicroseconds(SAMPLE_DELAY);
            // https://www.arduino.cc/en/Tutorial/AnalogInput
            _rawValue += analogRead(_signalPin);
        }
        digitalWrite(_enablePin, LOW);
        delayMicroseconds(SAMPLE_DELAY);
        _rawValue = (float)_rawValue / (float)SAMPLE_COUNT;

        // Convert the raw value into distance (centimeters).
        _distance = (float)_rawValue / SCALE_FACTOR_CM;
        return _distance;
    }

    float getDistance() {
        // Get the distance without performing a measurement.
        return _distance;
    }

    int getRawDistance() {
        // Get the distance without performing a measurement.
        return _rawValue;
    }

private:

    int _enablePin = 0;

    const int SAMPLE_COUNT = 3;
    
    // Outputs analog voltage with a scaling factor of (Vcc/512) per inch. 
    // A supply of 5V yields ~9.8mV/in. and 3.3V yields ~6.4mV/in.
    const float SCALE_FACTOR_IN = 1023 / 512;
    // 2.54 centimeters per inch.
    const float SCALE_FACTOR_CM = 1023 / 1300.48;
    // Delay enough time for sound to travel up to 645 cm to an object and 
    // be reflected back to the sensor.
    const int SAMPLE_DELAY = 37920; 
};


//==============================================================================
// PLATFORM CONTROLLER:
//==============================================================================

class PlatformController {

public:
    PlatformController(SonarRangefinder *sonarFront
        , SonarRangefinder *sonarLeft, SonarRangefinder *sonarRight
        , SonarRangefinder *sonarBack, AnalogMultiplexer *muxAnalogInput
        , AnalogMultiplexer *muxDigitalOutput, EncoderCounter *encoderLeft
        , EncoderCounter *encoderRight, MotorDriver *motorDriverLeft
        ,  MotorDriver *motorDriverRight) {

        _sonarFront = sonarFront;
        _sonarLeft = sonarLeft;
        _sonarRight = sonarRight;
        _sonarBack = sonarBack;

        _muxAnalogInput = muxAnalogInput;
        _muxDigitalOutput = muxDigitalOutput;

        // Used for distance measurements, so we will make them members.
        _encoderLeft = encoderLeft;
        _encoderRight = encoderRight;

        // Use the speed controllers to set the speed setpoint 
        _speedLeft = new SpeedController(motorDriverLeft, _encoderLeft);
        _speedRight = new SpeedController(motorDriverRight, encoderRight);

    }

    float getSonarDistance(int enablePin, int signalPin) {
        _muxDigitalOutput->selectChannel(enablePin);
        _muxAnalogInput->selectChannel(signalPin);
        return _sonarFront->readDistance();
    }

    float getSonarFront() {
        // Return the front sonar distance in centimeters.
        return getSonarDistance(C2_SONAR_FRONT_ENABLE, C0_SONAR_FRONT);
    }

    float getSonarLeft() {
        return getSonarDistance(C3_SONAR_LEFT_ENABLE, C1_SONAR_LEFT);
    }

    float getSonarRight() {
        return getSonarDistance(C4_SONAR_RIGHT_ENABLE, C2_SONAR_RIGHT);
    }

    float getSonarBack() {
        return getSonarDistance(C5_SONAR_BACK_ENABLE, C3_SONAR_BACK);
    }

    /******************************************************
    * Infrared range-finders
    ******************************************************/

    float getInfraredDistance(int signalPin) {
        _muxAnalogInput->selectChannel(signalPin);
        return _frontIr->readDistance();
    }

    float getInfraredFront() {
        return getInfraredDistance(C4_IR_DIST_FRONT);
    }

    float getInfraredLeft() {
        return getInfraredDistance(C5_IR_DIST_LEFT);
    }

    float getInfraredRight() {
        return getInfraredDistance(C6_IR_DIST_RIGHT);
    }

    float getInfraredBack() {
        return getInfraredDistance(C7_IR_DIST_BACK);
    }

    int getMuxInputRaw(int channel) {
        _muxAnalogInput->selectChannel(channel);
        return analogRead(MUX_ANALOG_IN_SIG);
    }


    unsigned long readEncoderCounterLeft() {
        // This is just experimental for now to see if we can write SPI.
        _muxDigitalOutput->selectChannel(C0_ENCODER_CNT_LEFT_CS);
        _prevEncoderCountsLeft = _encoderCountsLeft;
        _encoderCountsLeft = _encoderLeft->readCounts();
        return _encoderCountsLeft;
    }

    unsigned long readEncoderCounterRight() {
        // This is just experimental for now to see if we can write SPI.
        _muxDigitalOutput->selectChannel(C1_ENCODER_CNT_RIGHT_CS);
        _prevEncoderCountsRight = _encoderCountsRight;
        _encoderCountsRight = _encoderRight->readCounts();
        return _encoderCountsRight;
    }
    
    void setTimestamp(unsigned long milliseconds, unsigned int microseconds) {
        // Store the previous time-stamp:
        _prevMilliseconds = _milliseconds;
        _prevMicroseconds = _microseconds;
        // Store the current time-stamp:
        _milliseconds = milliseconds;
        _microseconds = microseconds;
        // Now we have delta T.

        _deltaTime = ((_milliseconds * 1000) + _microseconds) 
            - ((_prevMilliseconds * 1000) + _prevMicroseconds);

    }

    void setSpeedLeft(int speed) {
        if(speed != _speedLeft->getSetPoint()) {
            _speedLeft->setSpeed(speed);
        }
            
        int deltaCounts = _encoderCountsLeft - _prevEncoderCountsLeft;
        _speedLeft->updateControlLoop(deltaCounts, _deltaTime);
    }

    void setSpeedRight(int speed) {
        if(speed != _speedRight->getSetPoint()) {
            _speedRight->setSpeed(speed);
        }
        
        int deltaCounts = _encoderCountsRight - _prevEncoderCountsRight;
        _speedRight->updateControlLoop(deltaCounts, _deltaTime);
    }
    
    unsigned long getMotorStatusLeft(void) {
        return (_speedLeft->getProcessVariable() << 16)
            + (_speedLeft->getSetPoint() << 8)
            + _speedLeft->getControlVariable();
    }
    
    unsigned long getMotorStatusRight(void) {
        return (_speedRight->getProcessVariable() << 16)
            + (_speedRight->getSetPoint() << 8)
            + _speedRight->getControlVariable();
    }

private:

    SonarRangefinder *_sonarFront;
    SonarRangefinder *_sonarLeft;
    SonarRangefinder *_sonarRight;
    SonarRangefinder *_sonarBack;

    IrRangefinder *_frontIr;
    IrRangefinder *_leftIr;
    IrRangefinder *_rightIr;
    IrRangefinder *_backIr;

    SpeedController *_speedLeft;
    SpeedController *_speedRight;

    // Multiplexer for analog input signals such as distance sensors, etc..
    AnalogMultiplexer *_muxAnalogInput;
    // Multiplexer for analog outputs, but we are only using it for digital.
    AnalogMultiplexer *_muxDigitalOutput;

    EncoderCounter *_encoderLeft;
    EncoderCounter *_encoderRight;

    unsigned long _milliseconds;
    unsigned int _microseconds;

    unsigned long _prevMilliseconds;
    unsigned int _prevMicroseconds;

    unsigned long _deltaTime = 0; // Time in microseconds between now an previous.

    unsigned long _encoderCountsLeft;
    unsigned long _encoderCountsRight;

    long _prevEncoderCountsLeft;
    long _prevEncoderCountsRight;

};

// Process received serial data for configuration and control commands:
class CommandInterpreter {

public:
    CommandInterpreter() {
        // Process instructions from the system controller:
        // speed     - s:<speed value> 128 = stop
        // direction - d:<direction value>, turn rate between 0 and 255, 128 = no turn
        // Note: the turn rate is a differential, 1 = right+1, left+0, 2 = right+1, left+1
        // -1 = (right=0, left=1)
        // Implement commands to set user indicators (LEDs)
        // implement commands to set the PID coefficients.

    }

    void readCommands() {
        while(Serial.available() > 0) {
            // read the incoming bytes:
            unsigned char command = Serial.read();
            Serial.print("Received command: ");
            Serial.print(command);
            Serial.print(", param: ");
            if(Serial.read() == ':') {
                if(command == CMD_SPEED_LEFT) {
                    _speedLeft = Serial.parseInt();
                    Serial.println(_speedLeft);
                } else if(command == CMD_SPEED_RIGHT) {
                    _speedRight = Serial.parseInt();
                    Serial.println(_speedRight);
                } else if(command == CMD_RUN_MODE) {
                    _runMode = Serial.parseInt();
                    Serial.println(_runMode);
                } else if(command == CMD_RESET) {
                    Serial.println("Resetting spacial navigation controller.");    
                    resetFunc();  //call reset
                }
                // Parse user indicator commands and configuration commands.
                if(Serial.read() != ';') {
                    Serial.println("Received invalid end of message.");    
                }
            }
        }
    }

    int getSpeedLeft() {
        return _speedLeft;
    }

    int getSpeedRight() {
        return _speedRight;
    }
    
    int getRunMode() {
        return _runMode;
    }


private:

    enum InputCommands {
        CMD_RESET           = 0x61, // 'a' abort, reset the micro-controller.
        CMD_SPEED_LEFT      = 0x6c, // 'l' left, valid speeds: (-10, 10)
        CMD_SPEED_RIGHT     = 0x72, // 'r' right, valid speeds: (-10, 10)
        CMD_RUN_MODE        = 0x6d, // 'm' mode: 0 = AUTO_AVOID, 1 = EXT_CONTROL, 2 = REMOTE_CONTROL
        // ...
    };

    int _speedLeft;
    int _speedRight;
    int _runMode;

    unsigned char _userIndicator1;
    unsigned char _userIndicator2;
    unsigned char _userIndicator3;
    unsigned char _userIndicator4;

};

// Format sensor data and other platform data into messages to send to the PC:
class MessageFormatter {

public:
    MessageFormatter() {
        _message = String("s:");
    }

    void sendMessage(float sonarFront, float sonarLeft, float sonarRight
        , float sonarBack, float infraredDown, unsigned long encoderCountsLeft
        , unsigned long encoderCountsRight, unsigned long motorStatusLeft
        , unsigned long motorStatusRight, int buttonA, int buttonB, int buttonC
        , int buttonD, unsigned long milliseconds, unsigned int microseconds) {
                
        _message = "s:";
        _message = _message + sonarFront + ",";
        _message = _message + sonarLeft + ",";
        _message = _message + sonarRight + ",";
        _message = _message + sonarBack + ",";
        _message = _message + infraredDown + ",";
        _message = _message + encoderCountsLeft + ",";
        _message = _message + encoderCountsRight + ",";
        
        // Motor status is: byte 2: speed (process variable), byte 1: set-point
        // byte 0: control variable. if motor enable is true, low bit of byte 3 is set.
        _message = _message + motorStatusLeft + ",";
        // Same as above, but operating mode is in high byte.
        _message = _message + motorStatusRight + ",";
        
        _message = _message + buttonA + ",";
        _message = _message + buttonB + ",";
        _message = _message + buttonC + ",";
        _message = _message + buttonD + ",";

        _message = _message + milliseconds + ",";
        _message = _message + microseconds + ",";

        Serial.println(_message);
    }
    
private:
    String _message;
};


// Create global reference to the platform controller to use
// in the main loop for reading sensor data and performing motor control.
PlatformController *controller;

ExponentialMovingAverage *emaSonarFront;
ExponentialMovingAverage *emaSonarLeft;
ExponentialMovingAverage *emaSonarRight;
ExponentialMovingAverage *emaSonarBack;

// Initialize sensors and platform controller:
void setup() {

    Serial.begin(115200);

    AnalogMultiplexer *muxAnalogInput;
    AnalogMultiplexer *muxDigitalOutput;

    SonarRangefinder *sonarFront;
    SonarRangefinder *sonarLeft;
    SonarRangefinder *sonarRight;
    SonarRangefinder *sonarBack;
    // TODO: implement and initialize the IR range-finders.

    MotorDriver *motorDriverLeft;
    MotorDriver *motorDriverRight;

    EncoderCounter *encoderLeft;
    EncoderCounter *encoderRight;

    muxAnalogInput = new AnalogMultiplexer(MUX_ANALOG_IN_S0, MUX_ANALOG_IN_S1
        , MUX_ANALOG_IN_S2, MUX_ANALOG_IN_S3);

    muxDigitalOutput = new AnalogMultiplexer(MUX_DIGITAL_OUT_S0
        , MUX_DIGITAL_OUT_S1, MUX_DIGITAL_OUT_S2, -1);

    sonarFront = new SonarRangefinder(MUX_DIGITAL_OUT_SIG, MUX_ANALOG_IN_SIG);
    sonarLeft = new SonarRangefinder(MUX_DIGITAL_OUT_SIG, MUX_ANALOG_IN_SIG);
    sonarRight = new SonarRangefinder(MUX_DIGITAL_OUT_SIG, MUX_ANALOG_IN_SIG);
    sonarBack = new SonarRangefinder(MUX_DIGITAL_OUT_SIG, MUX_ANALOG_IN_SIG);
    // TODO: implement and initialize the IR range-finders.

    motorDriverLeft = new MotorDriver(MOTOR_PWM_LEFT, MOTOR_CTRL1_LEFT
        , MOTOR_CTRL2_LEFT);

    motorDriverRight = new MotorDriver(MOTOR_PWM_RIGHT, MOTOR_CTRL1_RIGHT
        , MOTOR_CTRL2_RIGHT);


    muxDigitalOutput->selectChannel(C0_ENCODER_CNT_LEFT_CS);
    encoderLeft = new EncoderCounter(SPI_MOSI, SPI_MISO, SPI_SERIAL_CLK
        , MUX_DIGITAL_OUT_SIG);

    muxDigitalOutput->selectChannel(C1_ENCODER_CNT_RIGHT_CS);
    encoderRight = new EncoderCounter(SPI_MOSI, SPI_MISO, SPI_SERIAL_CLK
        , MUX_DIGITAL_OUT_SIG);
    
    // Initialize the speed controllers and pass the motor drivers and encoders.
    
    // Initialize the platform controller and pass the sensors mux's and controllers.
    controller = new PlatformController(sonarFront, sonarLeft, sonarRight
        , sonarBack, muxAnalogInput, muxDigitalOutput, encoderLeft
        , encoderRight, motorDriverLeft, motorDriverRight);

    //motorDriverRight->setDirection(REVERSE);
    //motorDriverRight->setSpeed(100);
    randomSeed(analogRead(8));


    emaSonarFront = new ExponentialMovingAverage();
    emaSonarLeft = new ExponentialMovingAverage();
    emaSonarRight = new ExponentialMovingAverage();
    emaSonarBack = new ExponentialMovingAverage();
}


enum RunModes {
    MODE_AUTO_AVOID = 0,
    MODE_EXT_CONTROL = 1,
    MODE_REMOTE_CONTROL = 2,
};

enum ButtonDirections {
    DIR_NONE = -1,
    DIR_FORWARD = 0,
    DIR_LEFT = 1,
    DIR_RIGHT = 2,
    DIR_REVERSE = 3, 
};


float sonarFront = 0;
float sonarLeft = 0;
float sonarRight = 0;
float sonarBack = 0;

// Measure the distance to the ground. Detect ledges and steps.
float infraredDown = 0;

unsigned long encoderCountsLeft = 0;
unsigned long encoderCountsRight = 0;

unsigned long motorStatusLeft = 0;
unsigned long motorStatusRight = 0;

unsigned int microseconds = 0;
unsigned long milliseconds = 0;

int buttonA = 0;
int buttonB = 0;
int buttonC = 0;
int buttonD = 0;

int buttonDirection = DIR_NONE;

int runMode = MODE_AUTO_AVOID;

int debounceCounter = 0;
const int DEBOUNCE_LIMIT = 8;
bool motorEnable = false;

MessageFormatter messageFormatter; 
CommandInterpreter commandInterpreter;

int remoteSpeedLeft = 0;
int remoteSpeedRight = 0;

void processRemoteControlInput(void) {
    if(buttonA == 1 && buttonDirection != DIR_FORWARD) {
        debounceCounter++;
        if(debounceCounter == DEBOUNCE_LIMIT) {
            buttonDirection = DIR_FORWARD;
            remoteSpeedLeft = 4;
            remoteSpeedRight = 4;
            debounceCounter = 0;
        }
    } else if(buttonA == 1 && buttonDirection == DIR_FORWARD) {
        // Count up to the debounce limit, if we hit it then disable motion.
        debounceCounter++;
        if(debounceCounter == DEBOUNCE_LIMIT) {
            buttonDirection = DIR_NONE;
            remoteSpeedLeft = 0;
            remoteSpeedRight = 0;
            debounceCounter = 0;
        }    
    } else if(buttonD == 1 && buttonDirection != DIR_REVERSE) {
        debounceCounter++;
        if(debounceCounter == DEBOUNCE_LIMIT) {
            buttonDirection = DIR_REVERSE;
            remoteSpeedLeft = -4;
            remoteSpeedRight = -4;
            debounceCounter = 0;
        }
    } else if(buttonD == 1 && buttonDirection == DIR_REVERSE) {
        // Count up to the debounce limit, if we hit it then disable motion.
        debounceCounter++;
        if(debounceCounter == DEBOUNCE_LIMIT) {
            buttonDirection = DIR_NONE;
            remoteSpeedLeft = 0;
            remoteSpeedRight = 0;
            debounceCounter = 0;
        }
    } else if(buttonB == 1 && buttonDirection != DIR_LEFT) {
        debounceCounter++;
        if(debounceCounter == DEBOUNCE_LIMIT) {
            buttonDirection = DIR_LEFT;
            remoteSpeedLeft = 4;
            remoteSpeedRight = -4;
            debounceCounter = 0;
        }
    } else if(buttonB == 1 && buttonDirection == DIR_LEFT) {
        // Count up to the debounce limit, if we hit it then disable motion.
        debounceCounter++;
        if(debounceCounter == DEBOUNCE_LIMIT) {
            buttonDirection = DIR_NONE;
            remoteSpeedLeft = 4;
            remoteSpeedRight = 4;
            debounceCounter = 0;
        }    
    } else if(buttonC == 1 && buttonDirection != DIR_RIGHT) {
        debounceCounter++;
        if(debounceCounter == DEBOUNCE_LIMIT) {
            buttonDirection = DIR_RIGHT;
            remoteSpeedLeft = -4;
            remoteSpeedRight = 4;
            debounceCounter = 0;
        }
    } else if(buttonC == 1 && buttonDirection == DIR_RIGHT) {
        // Count up to the debounce limit, if we hit it then disable motion.
        debounceCounter++;
        if(debounceCounter == DEBOUNCE_LIMIT) {
            buttonDirection = DIR_NONE;
            remoteSpeedLeft = 0;
            remoteSpeedRight = 0;
            debounceCounter = 0;
        }
    }
    // Always update the speed.
    controller->setSpeedLeft(remoteSpeedLeft);
    controller->setSpeedRight(remoteSpeedRight);
    return;
}

int extSpeedLeft = 0;
int extSpeedRight = 0;

/* 
 * Process commands from the system controller.
 */
void processExternalControlInput(void) {
    if(buttonA == 1 && motorEnable == false) {
        debounceCounter++;
        if(debounceCounter == DEBOUNCE_LIMIT) {
            motorEnable = true;
            debounceCounter = 0;
        }
    } else if(buttonA == 1 && motorEnable == true) {
        // Count up to the debounce limit, if we hit it then disable motion.
        debounceCounter += 2;
        if(debounceCounter >= DEBOUNCE_LIMIT) {
            motorEnable = false;
            debounceCounter = 0;
        }
    }
    if(motorEnable) {
        extSpeedLeft = commandInterpreter.getSpeedLeft();
        extSpeedRight = commandInterpreter.getSpeedRight();
    } else {
        extSpeedLeft = 0;
        extSpeedRight = 0;
    }
    controller->setSpeedLeft(extSpeedLeft);
    controller->setSpeedRight(extSpeedRight);
    return;
}

const float DIST_THRESHOLD_FRONT_MAX = 75.0;
const float DIST_THRESHOLD_FRONT_MED = 50.0;
const float DIST_THRESHOLD_FRONT_MIN = 25.0;
const float DIST_THRESHOLD_SIDE = 25.0;
const float DIST_THRESHOLD_BACK = 25.0;
const float DIST_ABSOLUTE_MINIMUM = 18.0;

const int SPEED_SLOW = 4;
const int SPEED_MEDIUM = 7;
const int SPEED_FAST = 10;
// Ensures that an evasive movement is executed for longer than the minimum
// time required to get out of the condition that caused it. 
long int moveUntil = 0;

int autoSpeedLeft = 0;
int autoSpeedRight = 0;

void processAutoAvoidance(void) {
    if(buttonA == 1 && motorEnable == false) {
        debounceCounter++;
        if(debounceCounter == DEBOUNCE_LIMIT) {
            motorEnable = true;
            debounceCounter = 0;
        }
    } else if(buttonA == 1 && motorEnable == true) {
        // Count up to the debounce limit, if we hit it then disable motion.
        debounceCounter += 2;
        if(debounceCounter >= DEBOUNCE_LIMIT) {
            motorEnable = false;
            debounceCounter = 0;
        }
    }
    if(motorEnable == false) {
        controller->setSpeedLeft(0);
        controller->setSpeedRight(0);
        return;
    }
    
    // Also we need to be able to defeat the evasive maneuver if it gets us 
    // into a bad spot, and then choose a new one.

    // And not within absolute minimum thresholds. 
    if(moveUntil > millis()) {
        // We are executing some kind of evasive maneuver.
        // Move until the time is up.
        controller->setSpeedLeft(autoSpeedLeft);
        controller->setSpeedRight(autoSpeedRight);
        return;
    }
    
    // Use the distance sensors to determine the turn direction.
    if(sonarFront < DIST_THRESHOLD_FRONT_MAX
        && sonarLeft > DIST_THRESHOLD_SIDE
        && sonarRight > DIST_THRESHOLD_SIDE
        && sonarBack > DIST_THRESHOLD_BACK) {
        // If the robot's front sensor is closer than some threshold distance 
        // to an object, turn away from it.
        if(sonarFront < DIST_THRESHOLD_FRONT_MIN) {
            moveUntil = millis() + 2000;
            if(sonarLeft < sonarRight) {
                // Turn to the right, go in reverse.
                autoSpeedLeft = 0;
                autoSpeedRight = -SPEED_MEDIUM;
            } else if(sonarLeft >= sonarRight) {
                // Turn to the left, go in reverse.
                autoSpeedLeft = -SPEED_MEDIUM;
                autoSpeedRight = 0;
            }
        } else if(sonarFront < DIST_THRESHOLD_FRONT_MED) {
            moveUntil = millis() + 1000;
            if(sonarLeft < sonarRight) {
                // Turn to the right.
                autoSpeedLeft = SPEED_MEDIUM;
                autoSpeedRight = 0;
            } else if(sonarLeft >= sonarRight) {
                // Turn to the left.
                autoSpeedLeft = 0;
                autoSpeedRight = SPEED_MEDIUM;
            }
        } else {
            moveUntil = millis() + 500;
            if(sonarLeft < sonarRight) {
                // Turn to the right.
                autoSpeedLeft = SPEED_FAST;
                autoSpeedRight = SPEED_SLOW;
            } else if(sonarLeft >= sonarRight) {
                // Turn to the left.
                autoSpeedLeft = SPEED_SLOW;
                autoSpeedRight = SPEED_FAST;
            }
        }
    } else if(sonarLeft <= DIST_THRESHOLD_SIDE
        || sonarRight <= DIST_THRESHOLD_SIDE
        || sonarBack <= DIST_THRESHOLD_BACK) {
        if(sonarLeft <= DIST_THRESHOLD_SIDE
            && sonarRight <= DIST_THRESHOLD_SIDE
            && sonarBack <= DIST_THRESHOLD_BACK
            && sonarFront <= DIST_THRESHOLD_FRONT_MED) {
            // Do nothing. We cant go anywhere.
            moveUntil = millis() + 1000;
            autoSpeedLeft = 0;
            autoSpeedRight = 0;
        } else if(sonarLeft <= DIST_THRESHOLD_SIDE
            || sonarRight <= DIST_THRESHOLD_SIDE
            || (sonarBack <= DIST_THRESHOLD_BACK
            && sonarFront <= DIST_THRESHOLD_FRONT_MED)) {
            moveUntil = millis() + 2000;
            if(sonarLeft < sonarRight) {
                // Turn to the right, zero turning radius.
                autoSpeedLeft = SPEED_MEDIUM;
                autoSpeedRight = -SPEED_MEDIUM;
            } else if(sonarLeft >= sonarRight) {
                // Turn to the left, zero turning radius.
                autoSpeedLeft = -SPEED_MEDIUM;
                autoSpeedRight = SPEED_MEDIUM;
            }
        } else if(sonarFront > DIST_THRESHOLD_FRONT_MIN) {
            //moveUntil = millis() + (1000 * 1);
            autoSpeedLeft = SPEED_SLOW;
            autoSpeedRight = SPEED_SLOW;
        } else {
            // We should never get here... But set the speed to zero anyway.
            moveUntil = millis() + 1000;
            autoSpeedLeft = 0;
            autoSpeedRight = 0;
        }
    } else {
        //moveUntil = millis() + (1000 * 0.25);
        autoSpeedLeft = SPEED_FAST;
        autoSpeedRight = SPEED_FAST;
    }
    controller->setSpeedLeft(autoSpeedLeft);
    controller->setSpeedRight(autoSpeedRight);
    return;
}




// TODO: make platform model class that aggregates the sensors and controllers
// in order to implement low level sensor data processing for "headless" 
// operation. 
void loop() {
    // Poll the sonar range-finders:
    sonarFront = controller->getSonarFront();
    emaSonarFront->addSample(sonarFront);
    sonarFront = emaSonarFront->computeAverage();


    sonarLeft = controller->getSonarLeft();
    emaSonarLeft->addSample(sonarLeft);
    sonarLeft = emaSonarLeft->computeAverage();
    
    sonarRight = controller->getSonarRight();
    emaSonarRight->addSample(sonarRight);
    sonarRight = emaSonarRight->computeAverage();
    
    sonarBack = controller->getSonarBack();
    emaSonarBack->addSample(sonarBack);
    sonarBack = emaSonarBack->computeAverage();    
    
    // Poll the infrared range-finders:

    // Poll the encoder counters:
    // Calculate control parameters for motor control:
    encoderCountsLeft = controller->readEncoderCounterLeft();
    encoderCountsRight = controller->readEncoderCounterRight();
    
    encoderCountsLeft = controller->readEncoderCounterLeft();
    encoderCountsRight = controller->readEncoderCounterRight();
    
    motorStatusLeft = controller->getMotorStatusLeft();
    motorStatusRight = controller->getMotorStatusRight();
    if(motorEnable == true) {
        // set the low bit in the 4th byte in motor status left.
        motorStatusLeft += (1 << 24);
    }
    motorStatusRight += (runMode << 24);
    
    // Get the analog voltages for the motor battery and the platform battery:

    // Get the time since the program started:
    milliseconds = millis();
    microseconds = micros() % 1000;

    // Get the remote control button states:
    buttonA = (controller->getMuxInputRaw(C8_RADIO_A) < 100) ? 0 : 1;
    buttonB = (controller->getMuxInputRaw(C9_RADIO_B) < 100) ? 0 : 1;
    buttonC = (controller->getMuxInputRaw(C10_RADIO_C) < 100) ? 0 : 1;
    buttonD = (controller->getMuxInputRaw(C11_RADIO_D) < 100) ? 0 : 1;

    messageFormatter.sendMessage(sonarFront, sonarLeft, sonarRight, sonarBack
        , infraredDown, encoderCountsLeft, encoderCountsRight, motorStatusLeft
        , motorStatusRight,  buttonA, buttonB, buttonC, buttonD
        , milliseconds, microseconds);

    commandInterpreter.readCommands();
    runMode = commandInterpreter.getRunMode();
    controller->setTimestamp(milliseconds, microseconds);

    if(runMode == MODE_REMOTE_CONTROL) {
        processRemoteControlInput();
    } else if(runMode == MODE_EXT_CONTROL) {
        processExternalControlInput();
    } else if(runMode == MODE_AUTO_AVOID) {
        processAutoAvoidance();
    }
}




