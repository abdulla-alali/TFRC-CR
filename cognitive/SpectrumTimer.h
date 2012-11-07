

#ifndef SPECTRUM_TIMER_H
#define SPECTRUM_TIMER_H

#include <delay.h>
#include <connector.h>
#include <packet.h>
#include <random.h>


class SpectrumManager;
class SpectrumMobility;

class SenseTimer: public Handler {

	public:

        	SenseTimer(SpectrumManager *s); 

	        void handle(Event *e);

		void start(double time);

	private:
		Event           intr;		
		SpectrumManager *handler_;
};





class TransmitTimer: public Handler {

	public:

		 TransmitTimer(SpectrumManager *s);

                 void handle(Event *e);

		 void start(double time);

	private:
		Event           intr;
		SpectrumManager *handler_;

};




class HandoffTimer: public Handler {

	public:

		HandoffTimer(SpectrumMobility *s);

                void  handle(Event *e);
		
		void  start (double time);

	private:
		Event           intr;
		SpectrumMobility *handler_;

};


#endif
