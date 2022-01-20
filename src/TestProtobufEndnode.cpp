/*
 * TestProtobufEndnode
 *
 * - LoRaWan message serialization with ProtocolBuffer
 * - standby with EnergyController
 * - timer interrupt with ISRTimer
 */

#include <Arduino.h>
#include <arduino_lmic_hal_boards.h>
#include <Wire.h>

// leuville-arduino-easy-lmic
#include <LoraPBEndnode.h>
// leuville-arduino-utilities
#include <misc-util.h>
#include <ISRWrapper.h>
#include <ISRTimer.h>
#include <energy.h>
#include <StatusLed.h>

// nanopb (Protocol Buffer)
#include "message.pb.h"

// LoRaWAN configuration: see OTAAId in LMICWrapper.h
#include <lora-common-defs.h>

USBPrinter<Serial_> console(STDOUT);

namespace lstl = leuville::simple_template_library;
namespace lora = leuville::lora;

using namespace lstl;
using namespace lora;

/* ---------------------------------------------------------------------------------------
 * Application classes
 * ---------------------------------------------------------------------------------------
 */

/*
 * Template base class of our device
 *
 * Instanciated with nanopb generated types for uplink & downlink messages
 */
using Base = ProtobufEndnode<
	leuville_Uplink, leuville_Uplink_fields,
	leuville_Downlink, leuville_Downlink_fields
>;
using Button1 = ISRWrapper<DEVICE_BUTTON1_ISR>;

/*
 * LoraWan + ProtocolBuffer endnode with:
 * - a timer to trigger a PING message on regular basis
 * - a callback set on button connected to a pin, which triggers a BUTTON message
 * - standby mode capacity
 */
class EndNode : public Base, 
				private ISRTimer, private Button1, private EnergyController
{
	/*
	 * Jobs for event callbacks
	 */
	osjob_t _buttonJob;
	osjob_t _timeoutJob;
	osjob_t _joinJob;
	osjob_t _txCompleteJob;

	/*
	 * Message send retry mechanism
	 */
	uint8_t _nbRetries = 0;
	uint8_t _maxRetries = 1;

public:

	EndNode(RTCZero& rtc, const lmic_pinmap *pinmap): 
		Base(pinmap),
		ISRTimer(rtc, DEVICE_MEASURE_DELAY, true),
		Button1(INPUT_PULLUP, LOW),
		EnergyController(rtc)
	{
	}

	/*
	 * delegates begin() to each sub-component and send a PING message
	 */
	void begin(const OTAAId& id, u4_t network, bool adr = true) override {
		Wire.begin();
		Base::begin(id, network, adr);
		Button1::begin();
		EnergyController::begin();
		ISRTimer::begin(true);

		Button1::enable();
		ISRTimer::enable();
		EnergyController::enable();
		startJoining();
	}

	/*
	 * Configure LoRaWan network (channels, power, adr, ...)
	 */
	virtual void initLMIC(u4_t network, bool adr = true) override {
		configureNetwork(static_cast<Network>(network), adr);
	}

	/*
	 * Button ISR
	 * job done by sending a LMIC callback
	 * see performJob()
	 */
	virtual void ISR_callback(uint8_t pin) override {
		setCallback(_buttonJob);
	}

	/*
	 * Timer ISR
	 * job done by sending a LMIC callback
	 * see performJob()
	 */
	virtual void ISR_timeout() override {
		setCallback(_timeoutJob);
	}

	/*
	 * Job done on join/unjoin
	 * see completeJob()
	 */
	virtual void joined(boolean ok) override {
		if (ok) {
			setCallback(_joinJob);
		}
	}

	/*
	 * Implements a retry policy for important messages in case of unsuccessful transmission
	 */
	virtual bool isTxCompleted(const leuville_Uplink & payload, bool ackRequest, bool ack) override {
		setCallback(_txCompleteJob);
		console.print("isTxCompleted ", payload.battery);
		console.print(" ", payload.type);
		console.print(" ", ackRequest); 
		console.println(" ", ack);
		if (ack || !ackRequest) {
			_nbRetries = 0;
			return true;
		} else {
			if (++_nbRetries > _maxRetries) {
				_nbRetries = 0;
				return true;
			}
		}
		return false;
	};

	/*
	 * Updates the timer delay
	 */
	virtual void downlinkReceived(const leuville_Downlink & payload) override {
		console.println("downlink delay: ", payload.pingDelay);
		ISRTimer::setTimeout((uint32_t)payload.pingDelay);
	}

	/*
	 * Handle LMIC callbacks
	 */
	virtual void completeJob(osjob_t* job) override {
		if (job == &_buttonJob) {
			send(buildPayload(leuville_Type_BUTTON), true); 
		} else if (job == &_timeoutJob) {
			if (!hasMessageToSend()) {
				send(buildPayload(leuville_Type_PING), false);
			}
		} else if (job == &_joinJob) {
			LoRaWanSessionKeys keys = getSessionKeys();
			// https://www.thethingsnetwork.org/docs/lorawan/prefix-assignments.html
			console.println("netId: ", keys._netId, HEX);
			console.println("devAddr: ", keys._devAddr, HEX);
			console.print("nwkSKey: "); console.printHex(keys._nwkSKey, arrayCapacity(keys._nwkSKey));
			console.print("appSKey: "); console.printHex(keys._appSKey, arrayCapacity(keys._appSKey));
			postJoinSetup(keys._netId);
		} else if (job == &_txCompleteJob) {
			console.println("TX COMPLETE, FIFO size: ", _messages.size());
		}
	}

	/*
	 * Builds an uplink message
	 */
	leuville_Uplink buildPayload(leuville_Type uplinkType) {
		leuville_Uplink payload = leuville_Uplink_init_zero;
		payload.battery = getBatteryPower();
		payload.type = uplinkType;
		return payload;
	}  

	/*
	 * Post a message in the send queue
	 */
	void send(const leuville_Uplink & payload, bool ack = false) {
		console.print("send ", payload.battery); 
		console.println(" ", payload.type);
		Base::send(payload, ack);
	}

	/*
	 * Activates standby mode
	 */
	void standby() {
		EnergyController::standby();
	}

};

/* ---------------------------------------------------------------------------------------
 * GLOBAL OBJECTS
 * ---------------------------------------------------------------------------------------
 */
RTCZero 	rtc;
BlinkingLed statusLed(LED_BUILTIN, 500);

#if defined(LMIC_PINS)
EndNode 	endnode(rtc, LMIC_PINS); 
#else
EndNode 	endnode(rtc, Arduino_LMIC::GetPinmap_ThisBoard());
#endif

/* ---------------------------------------------------------------------------------------
 * ARDUINO FUNCTIONS
 * ---------------------------------------------------------------------------------------
 */
void setup()
{
	Serial.begin(115200);

	statusLed.begin();
	statusLed.on();

	while (!Serial.available()) { delay(10); }

	endnode.begin(	id[Config::DEVTEST1], 	Network::ORANGE,	ADR::ON		);

	delay(5000);
	statusLed.off();
}

void loop()
{
	endnode.runLoopOnce();
	if (endnode.isReadyForStandby()) {
		statusLed.off();
		#ifdef PRODUCTION_MODE
		endnode.standby(); 		
		#endif
	}
	else {
		statusLed.blink();
	}
}
