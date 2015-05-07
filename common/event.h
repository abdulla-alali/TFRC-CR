#ifndef SWITCH_EVENT_H
#define SWITCH_EVENT_H

// CRAHNs Model START
// @author: Marco Di Felice

// EventSwitch: Sent by the MAC layer to ask for a new packet on a given channel
class EventSwitch: public Event {

	public:
		EventSwitch() {};
		int channel;
};

// CRAHNs Model END

#endif
