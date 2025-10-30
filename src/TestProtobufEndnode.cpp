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
#include <ProtobufEndnode.h>
// leuville-arduino-utilities
#include <misc-util.h>
#include <ISRWrapper.h>
#include <ISRTimer.h>
#include <EnergyController.h>
#include <StatusLed.h>
#include <JobRegister.h>

// nanopb (Protocol Buffer)
#include "message.pb.h"

// LoRaWAN configuration: see OTAAId in LMICWrapper.h
#include "lora-common-defs.h"

#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
USBPrinter<Serial_> console(LMIC_PRINTF_TO);
#endif

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
using Button1 = ISRWrapper<DEVICE_BUTTON1_PIN>;
using EnergyCtrl = EnergyController<VOLTAGE_MIN,VOLTAGE_MAX>;

/*
 * LoraWan + ProtocolBuffer endnode with:
 * - a timer to trigger a PING message on regular basis
 * - a callback set on button connected to a pin, which triggers a BUTTON message
 * - standby mode capacity
 */
class EndNode : public Base, 
				private ISRTimer, private Button1, private EnergyCtrl
{
	const Range<u1_t> _rangeLora {MCMD_DEVS_BATT_MIN, MCMD_DEVS_BATT_MAX};

	/*
	 * Redefines EnergyController::defineGetVoltage()
	 */
	#if defined(ARDUINO_SAMD_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0)
	virtual std::function<double(void)> defineGetVoltage() override {
		return []() -> double { return analogRead(A7) * 2 * 3.3 / 1.023; };
	}
	#endif

	/*
	 * Jobs for LMIC event callbacks
	 */
	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
	enum JOB { BUTTON, TIMEOUT, JOIN, TXCOMPLETE, _COUNT };
	#else
	enum JOB { BUTTON, TIMEOUT, JOIN, _COUNT };
	#endif
	JobRegister<EndNode,JOB::_COUNT> _callbacks;

public:

	EndNode(const lmic_pinmap &pinmap): EndNode(&pinmap) {}

	EndNode(const lmic_pinmap *pinmap): 
		Base(pinmap),
		ISRTimer(60 * DEVICE_MEASURE_DELAY, true),
		Button1(INPUT_PULLUP, LOW)
	{
		_callbacks.define(BUTTON, 		this, &EndNode::buttonJob);
		_callbacks.define(TIMEOUT, 		this, &EndNode::timeoutJob);
		_callbacks.define(JOIN, 		this, &EndNode::joinJob);
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 1
		_callbacks.define(TXCOMPLETE, 	this, &EndNode::txCompleteJob);
		#endif
	}

	/*
	 * delegates begin() to each sub-component and send a PING message
	 */
	void begin(const OTAAId& id, u4_t network, bool adr = true) override {
		Wire.begin();
		EnergyCtrl::begin();
		Base::begin(id, network, adr);
		Button1::begin();
		ISRTimer::begin(); 

		EnergyCtrl::setUnusedPins({A1, A2, A3, A4, A5});

		startJoining();
	}

	/*
	 * Configure LoRaWan network (channels, power, adr, ...)
	 */
	virtual void initLMIC(u4_t network = 0, bool adr = true) override {
		Base::initLMIC(network, adr);
		configureNetwork(static_cast<Network>(network), adr);
	}

	/*
	 * Button ISR
	 * job done by sending a LMIC callback
	 * see completeJob()
	 */
	virtual void ISR_callback(uint8_t pin) override {
		setCallback(_callbacks[BUTTON]);
	}

	/*
	 * Timer ISR
	 * job done by sending a LMIC callback
	 * see completeJob()
	 */
	virtual uint32_t ISR_timeout() override {
		setCallback(_callbacks[TIMEOUT]);
		return 60 * DEVICE_MEASURE_DELAY;
	}


	/*
	 * Job done on join/unjoin
	 * see completeJob()
	 */
	virtual void joined(bool ok) override {
		if (ok) {
			setCallback(_callbacks[JOIN]);
			Button1::enable();
			ISRTimer::enable();
		} else {
			for (auto & job : _callbacks) {
				unsetCallback(job);
			}
			Button1::disable();
			ISRTimer::disable();
		}
	}

		/*
	 * Updates the system time
	 */
	#if defined(LMIC_ENABLE_DeviceTimeReq)
	virtual void updateSystemTime(uint32_t newTime) override {
		ISRTimer::setEpoch(newTime);
    	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		RTCZero & rtc = ISRTimer::getRTC();
		console.println("----------------------------------------------------------");
		console.print("heure network=",	rtc.getHours()); 
		console.print(":", rtc.getMinutes());
		console.println(":", rtc.getSeconds());
		console.println("----------------------------------------------------------");
    	#endif
	}
	virtual bool isSystemTimeSynced() override {
    	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		console.println("----------------------------------------------------------");
		console.println("system time age=", systemTimeAge()); 
		console.println("----------------------------------------------------------");
		#endif
		return systemTimeAge() < SYSTEM_TIME_MAX_AGE;
	}
	#endif

		/*
	 * Updates the timer delay
	 */
	virtual void downlinkReceived(const leuville_Downlink & payload, const DownstreamMessage & rawMessage) override {
    	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		console.println("----------------------------------------------------------");
		console.println("downlink delay: ", payload.pingDelay);
		console.println("----------------------------------------------------------");
		#endif
		ISRTimer::setTimeout((uint32_t)payload.pingDelay);
	}

	/*
	 * Handle LMIC callbacks
	 */
	virtual void completeJob(osjob_t* job) override {
		_callbacks[job]();
	}

	virtual void buttonJob() {
		send(buildPayload(leuville_Type_BUTTON), true); 
	}

	virtual void timeoutJob() {
		if (!hasMessageToSend()) {
				send(buildPayload(leuville_Type_PING), false);
			}
	}

	virtual void joinJob() {
		LoRaWanSessionKeys keys = getSessionKeys();
		// https://www.thethingsnetwork.org/docs/lorawan/prefix-assignments.html
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		console.println("----------------------------------------------------------");
		console.println("netId: ", keys._netId, HEX);
		console.println("devAddr: ", keys._devAddr, HEX);
		console.print("nwkSKey: "); console.printHex(keys._nwkSKey, arrayCapacity(keys._nwkSKey));
		console.print("appSKey: "); console.printHex(keys._appSKey, arrayCapacity(keys._appSKey));
		console.println("----------------------------------------------------------");
		#endif
		postJoinSetup(keys._netId);
	}

	/*
	 * Set batteryLevel and builds an uplink message
	 */
	leuville_Uplink buildPayload(leuville_Type uplinkType) {
		auto batt = getBatteryPower(_rangeLora);	
		setLoraBatteryLevel(batt);

		leuville_Uplink payload = leuville_Uplink_init_zero;
		payload.battery = scaleValue(batt, EnergyCtrl::_range100);
		payload.type = uplinkType;
		return payload;
	}  

	/*
	 * Post a message in the send queue
	 */
	void send(const leuville_Uplink & payload, bool ack = false) {
		Base::send(payload, ack);
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		console.println("----------------------------------------------------------");
		console.print("send ", payload.battery); 
		console.print(" ", payload.type);
		console.println(", FIFO size: ", _messages.size());
		console.println("----------------------------------------------------------");
		#endif
	}

	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 1

	virtual void txCompleteJob() {
		console.println("----------------------------------------------------------");
		console.println("FIFO size: ", _messages.size());
		console.println("----------------------------------------------------------");
	}

	/*
	 * LMIC callback called on TX_COMPLETE event
	 */
	virtual bool isTxCompleted(const leuville_Uplink & payload, const UpstreamMessage & rawMessage) override {
		setCallback(_callbacks[TXCOMPLETE]);
		console.println("----------------------------------------------------------");
		console.print("isTxCompleted "); 
		console.print(" / ackRequest: ", 	rawMessage._ackRequested);
		console.print(" / ack: ", 			rawMessage.isAcknowledged());
		console.print(" / lmicError: ", 	(int32_t)rawMessage._lmicTxError);
		console.print(" / _txrxFlags: ", 	rawMessage._txrxFlags);
		console.println("----------------------------------------------------------");
		return Base::isTxCompleted(message, rawMessage);
	}

	#endif

	/*
	 * Activates standby mode
	 */
	void standby() {
		ISRTimer::standbyMode();
	}

};

/* ---------------------------------------------------------------------------------------
 * GLOBAL OBJECTS
 * ---------------------------------------------------------------------------------------
 */
BlinkingLed statusLed(LED_BUILTIN, 500);

#if defined(LMIC_PINS)
EndNode 	endnode(LMIC_PINS); 
#else
EndNode 	endnode(Arduino_LMIC::GetPinmap_ThisBoard());
#endif

/* ---------------------------------------------------------------------------------------
 * ARDUINO FUNCTIONS
 * ---------------------------------------------------------------------------------------
 */
void setup()
{
	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
	console.begin(115200);
	#endif

	statusLed.begin();
	statusLed.on();

	endnode.begin(id[DEVICE_CONFIG], DEVICE_NETWORK, ADR::ON);

	delay(5000);
	statusLed.off();
}

void loop()
{
	endnode.runLoopOnce();
	if (endnode.isReadyForStandby()) {
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		statusLed.off();
		#else
		endnode.standby(); 
		#endif
	}
	else {
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		statusLed.blink();
		#endif
	}
}
