package gopigo

import (
	"errors"
	"github.com/hybridgroup/gobot/platforms/raspi"
	"time"
)

// debug=0
// bwd_cmd				=[115]		#Move back with PID
// motor_bwd_cmd		=[107]		#Move back without PID
// stop_cmd			=[120]		#Stop the GoPiGo
// ispd_cmd			=[116]		#Increase the speed by 10
// dspd_cmd			=[103]		#Decrease the speed by 10
// read_motor_speed_cmd=[114]		#Get motor speed back
//
// volt_cmd			=[118]		#Read the voltage of the batteries
// us_cmd				=[117]		#Read the distance from the ultrasonic sensor
// led_cmd				=[108]		#Turn On/Off the LED's
// servo_cmd			=[101]		#Rotate the servo
// fw_ver_cmd			=[20]		#Read the firmware version
// en_enc_cmd			=[51]		#Enable the encoders
// dis_enc_cmd			=[52]		#Disable the encoders
// read_enc_status_cmd	=[53]		#Read encoder status
// en_servo_cmd		=[61]		#Enable the servo's
// dis_servo_cmd		=[60]		#Disable the servo's
// set_left_speed_cmd	=[70]		#Set the speed of the right motor
// set_right_speed_cmd	=[71]		#Set the speed of the left motor
// en_com_timeout_cmd	=[80]		#Enable communication timeout
// dis_com_timeout_cmd	=[81]		#Disable communication timeout
// timeout_status_cmd	=[82]		#Read the timeout status
// enc_read_cmd		=[53]		#Read encoder values
// trim_test_cmd		=[30]		#Test the trim values
// trim_write_cmd		=[31]		#Write the trim values
// trim_read_cmd		=[32]
//
// digital_write_cmd   =[12]      	#Digital write on a port
// digital_read_cmd    =[13]      	#Digital read on a port
// analog_read_cmd     =[14]      	#Analog read on a port
// analog_write_cmd    =[15]      	#Analog read on a port
// pin_mode_cmd        =[16]      	#Set up the pin mode on a port
//
// ir_read_cmd			=[21]
// ir_recv_pin_cmd		=[22]
// cpu_speed_cmd		=[25]

// #LED Pins
// #MAKE COMPATIBLE WITH OLD FIRMWARE
// # LED_L_PIN=17
// # LED_R_PIN=16
//
// #port definition
// analogPort=15
// digitalPort=10
//
// #LED setup
// LED_L=1
// LED_R=0
//
// # This allows us to be more specific about which commands contain unused bytes
// unused = 0
//
// v16_thresh=790

const (
	address = 0x08

	digitalWriteCmd = byte(12) // digital write on a port
	digitalReadCmd  = byte(13) // 	digital read on a port
	analogReadCmd   = byte(14) // analog read on a port
	analogWriteCmd  = byte(15) // analog read on a port
	pinModeCmd      = byte(16) // set up the pin mode on a port

	motor1Cmd = byte(111) // control motor 1
	motor2Cmd = byte(112) // control motor 2

	forwardCmd      = byte(119) // move forward with PID control
	motorForwardCmd = byte(105) // move forward without PID control

	turnLeftCmd    = byte(97)  // turn left by turning off one motor
	turnRightCmd   = byte(100) // turn right by turning off one motor
	rotateLeftCmd  = byte(98)  // rotate left by running both motors is opposite direction
	rotateRightCmd = byte(110) // rotate Right by running both motors is opposite direction

	encoderTargetCmd = byte(50) //set the encoder targeting
)

var pins = map[string]byte{
	"0":  0,
	"1":  1,
	"10": 10,
	"15": 15,
}

type GoPiGoAdaptor struct {
	name  string
	raspi *raspi.RaspiAdaptor
}

func NewGoPiPoAdaptor(name string) (*GoPiGoAdaptor, error) {
	g := &GoPiGoAdaptor{
		name:  name,
		raspi: raspi.NewRaspiAdaptor(name + "_raspi"),
	}
	err := g.raspi.I2cStart(address)
	if err != nil {

	}
	return g, nil
}

// DigitalRead reads the from pin
func (g *GoPiGoAdaptor) DigitalRead(pin string) (int, error) {
	p, err := translatePin(pin)
	if err != nil {
		return 0, err
	}
	err = g.raspi.I2cWrite(address, []byte{digitalReadCmd, p, 0, 0})
	if err != nil {
		return 0, err
	}
	time.Sleep(100 * time.Millisecond)
	d, err := g.raspi.I2cRead(address, 1)
	return int(d[0]), err
}

// translatePin looks up the pin in the pins map
func translatePin(pin string) (byte, error) {
	p, ok := pins[pin]
	if !ok {
		return 0, errors.New("no such pin")
	}
	return p, nil
}

func (g *GoPiGoAdaptor) Motor1(direction byte, speed byte) error {
	return g.raspi.I2cWrite(address, []byte{motor1Cmd, direction, speed, 0})
}

// Motor2 controlls moto 2
func (g *GoPiGoAdaptor) Motor2(direction byte, speed byte) error {
	return g.raspi.I2cWrite(address, []byte{motor2Cmd, direction, speed, 0})
}

// Forward drives the robot forward with PID control
func (g *GoPiGoAdaptor) Forward() error {
	return g.raspi.I2cWrite(address, []byte{forwardCmd, 0, 0, 0})
}

// MotorForward drives forward without PID control
func (g *GoPiGoAdaptor) MotorForward() error {
	return g.raspi.I2cWrite(address, []byte{motorForwardCmd, 0, 0, 0})
}

// TurnLeft by turning off one motor
func (g *GoPiGoAdaptor) TurnLeft() error {
	return g.raspi.I2cWrite(address, []byte{turnLeftCmd, 0, 0, 0})
}

// TurnLeft by turning off one motor
func (g *GoPiGoAdaptor) TurnRight() error {
	return g.raspi.I2cWrite(address, []byte{turnRightCmd, 0, 0, 0})
}

// RotateLeft by running both motors is opposite direction
func (g *GoPiGoAdaptor) RotateLeft() error {
	return g.raspi.I2cWrite(address, []byte{rotateLeftCmd, 0, 0, 0})
}

// RotateRight by running both motors is opposite direction
func (g *GoPiGoAdaptor) RotateRight() error {
	return g.raspi.I2cWrite(address, []byte{rotateRightCmd, 0, 0, 0})
}

// Set encoder targeting on for motor 1 and/or motor 2 with the number of encoder pulses
func (g *GoPiGoAdaptor) EncoderTarget(motor1 bool, motor2 bool, pulses int) error {
	motorSelect := byte(0)
	if motor1 {
		motorSelect += 2
	}
	if motor2 {
		motorSelect += 1
	}
	return g.raspi.I2cWrite(address, []byte{encoderTargetCmd, motorSelect, byte(pulses / 256), byte(pulses % 256)})
}
