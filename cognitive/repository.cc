// Switchable Interface Implementation START
// @author:  Marco Di Felice	

// Class Repository 
// Cross-Layer Repository to enable channel information sharing between MAC and routing protocols

#include "repository.h"


static class Repositoryclass : public TclClass {
public:
        Repositoryclass() : TclClass("CrossLayerRepository") {}
        TclObject* create(int argc, const char*const* argv) {
          return (new Repository());
        }
} class_repository;




// Initializer
Repository::Repository() {
	
	// Set randomly the receiver channel for each node	
	for (int i=0; i<MAX_NODES; i++) {
		int channel=get_random_channel();
		repository_table[i].recv_channel= channel;
	}

	// Initialize each sending channel as NOT active for each node
	for (int node=0; node<MAX_NODES; node++) 
		for (int channel=0; channel< MAX_CHANNELS; channel++) 
			repository_table_sender[node][channel].active=false;



}




//get_recv_channel: Return the receiving channel for a node
int 
Repository::get_recv_channel(int node) {
	if (node < MAX_NODES)
		return repository_table[node].recv_channel;
	else
		return -1;

}
		 

//set_recv_channel: Set the receiving channel for a node
void 
Repository::set_recv_channel(int node, int channel) {
	if (node < MAX_NODES)
		repository_table[node].recv_channel=channel;

}

		
// update_send_channel: Set the sending channel as active, at the current time
void 
Repository::update_send_channel(int node, int channel, double time) {

	if (node < MAX_NODES)  {
		
		repository_table_sender[node][channel].active=true;
		repository_table_sender[node][channel].time=time;
	
	 }

}
		 

//is_channel_used_for_sending: Check wheter a given sending channel is active for a given node
bool 
Repository::is_channel_used_for_sending(int node, int channel, double timeNow) {

	if (repository_table_sender[node][channel].active) {
		if (timeNow - repository_table_sender[node][channel].time > TIMEOUT_ALIVE)
			repository_table_sender[node][channel].active=false;
	}
	
	return repository_table_sender[node][channel].active;
	
}



//get_random_channel: Return a random channel between 1 and MAX_CHANNELS
int 
Repository::get_random_channel() {
	
	int channel=((int)(Random::uniform()*MAX_CHANNELS))+1;		
	if (channel >= MAX_CHANNELS)
		channel = MAX_CHANNELS-1;
	return channel;
}


// recv: Empty method
void
Repository::recv(Packet*, Handler* = 0) {

}

// command: Empty method
int
Repository::command(int argc, const char*const* argv) {
}


// Switchable Interface Implementation END

