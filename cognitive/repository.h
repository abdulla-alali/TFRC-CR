// Switchable Interface Implementation START
// @author:  Marco Di Felice	

#ifndef repository_H
#define repository_H


// Max number of nodes in the simulation
#define MAX_NODES		 	200


// Defines the time a node spends on each queue
#define QUEUE_UTILIZATION_INTERVAL 	1.0
// Defines the channel switching delay
#define SWITCHING_DELAY			0.001
// Defines the TIMEOUT_ALIVE to check wheter a node is active on a given channel or not
#define TIMEOUT_ALIVE			2


// Multi-radio multi-channel specification
// Channels/Radio Definition. DO NOT MODIFY HERE!


// Interface Classification
#define CONTROL_RADIO 		0
#define TRANSMITTER_RADIO 	1
#define RECEIVER_RADIO  	2

// Channe/Radio Information 
#define MAX_RADIO	3
#define	MAX_CHANNELS 	11
#define CONTROL_CHANNEL 0


#define SENSING_VERBOSE_MODE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <random.h>
#include "object.h"


// Channel Entry for receiver nodes
struct repository_entry_recv {
	// receiving channel
	int recv_channel;
};



// Channel Entry for sender nodes
struct repository_entry_send {
	//Flag indicating wheter the channel is used for transmitting
	bool active;
	//Last time the channel was used
	double time;
};





// Cross-Layer Repository class
class Repository : public NsObject {

        public:

		 // Initializer
                 Repository();
                 int command(int argc, const char*const* argv);
                 void recv(Packet*, Handler*);
		  
	 		 
		 // Set/Get Function for the Receiver Channel Table
		 int get_recv_channel(int node);
		 void set_recv_channel(int node, int channel);
		
		 // Set/Get Function for the Sender Channel Table
		 void update_send_channel(int node, int channel, double time);
		 bool is_channel_used_for_sending(int node, int channel, double timeNow);

	
	private:

		// Receiver Channel table: repository_table[i] contains the channels used for receiving by node i
		repository_entry_recv repository_table[MAX_NODES];
		// Sender Channel table: repository_table_sender[i][j] contains the information (active/time) for sending node i and channel j
		 repository_entry_send repository_table_sender[MAX_NODES][MAX_CHANNELS];
		// Returns a random channel between 1 and MAX_CHANNELS
		int get_random_channel();
		
};

// Switchable Interface Implementation END

#endif


