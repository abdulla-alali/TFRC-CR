/* -*-  Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- */
/*
 * Copyright (c) 1999  International Computer Science Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by ACIRI, the AT&T 
 *      Center for Internet Research at ICSI (the International Computer
 *      Science Institute).
 * 4. Neither the name of ACIRI nor of ICSI may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ICSI AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL ICSI OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <stdlib.h>
#include <sys/types.h>
#include <math.h>

#include "tfrc-cr.h"
#include "formula.h"
#include "flags.h"

int hdr_tfrc_cr::offset_;
int hdr_tfrc_cr_ack::offset_;

static class TFRC_CR_HeaderClass : public PacketHeaderClass {
public:
	TFRC_CR_HeaderClass() : PacketHeaderClass("PacketHeader/TFRC_CR",
			sizeof(hdr_tfrc_cr)) {
		bind_offset(&hdr_tfrc_cr::offset_);
	}
} class_tfrc_cr_hdr;

static class TFRC_CR_ACKHeaderClass : public PacketHeaderClass {
public:
	TFRC_CR_ACKHeaderClass() : PacketHeaderClass("PacketHeader/TFRC_CR_ACK",
			sizeof(hdr_tfrc_cr_ack)) {
		bind_offset(&hdr_tfrc_cr_ack::offset_);
	}
} class_tfrc_cr_ackhdr;

static class Tfrc_CR_Class : public TclClass {
public:
	Tfrc_CR_Class() : TclClass("Agent/TFRC_CR") {}
	TclObject* create(int, const char*const*) {
		return (new Tfrc_CR_Agent());
	}
} class_tfrc_cr;


Tfrc_CR_Agent::Tfrc_CR_Agent() : Agent(PT_TFRC_CR), send_timer_(this), puoff_timer_(this), check_pu_off_timer_(this),
		NoFeedbacktimer_(this), rate_(0), oldrate_(0), maxrate_(0)
{
	bind("packetSize_", &size_);
	bind("rate_", &rate_);
	bind("df_", &df_);
	bind("tcp_tick_", &tcp_tick_);
	bind("ndatapack_", &ndatapack_);
	bind("ndatabytes_", &ndatabytes_);
	bind("true_loss_rate_", &true_loss_rate_);
	bind("srtt_init_", &srtt_init_);
	bind("rttvar_init_", &rttvar_init_);
	bind("rtxcur_init_", &rtxcur_init_);
	bind("rttvar_exp_", &rttvar_exp_);
	bind("T_SRTT_BITS", &T_SRTT_BITS);
	bind("T_RTTVAR_BITS", &T_RTTVAR_BITS);
	bind("InitRate_", &InitRate_);
	bind("overhead_", &overhead_);
	bind("ssmult_", &ssmult_);
	bind("bval_", &bval_);
	bind("ca_", &ca_);
	bind_bool("printStatus_", &printStatus_);
	bind_bool("conservative_", &conservative_);
	bind_bool("ecn_", &ecn_);
	bind("minrto_", &minrto_);
	bind("maxHeavyRounds_", &maxHeavyRounds_);
	bind("SndrType_", &SndrType_); 
	bind("scmult_", &scmult_);
	bind_bool("oldCode_", &oldCode_);
	bind("rate_init_", &rate_init_);
	bind("rate_init_option_", &rate_init_option_);
	bind_bool("slow_increase_", &slow_increase_); 
	bind_bool("voip_", &voip_);
	bind("voip_max_pkt_rate_", &voip_max_pkt_rate_);
	bind("fsize_", &fsize_);
	bind_bool("useHeaders_", &useHeaders_);
	bind_bool("idleFix_", &idleFix_);
	bind("headersize_", &headersize_);
	bind("flost_", &flost_);
	bind("mysendrate_", &mysendrate_);
	bind("rcvrate_", &rcvrate);
	bind("algo_", &algo);
	seqno_ = -1;
	maxseq_ = 0;
	datalimited_ = 0;
	lastlimited_ = 0.0;
	last_pkt_time_ = 0.0;
	bind("maxqueue_", &maxqueue_);
	maxqueue_ = MAXSEQ;
}

/*
 * Must convert bytes into packets. 
 * If nbytes == -1, this corresponds to infinite send.  We approximate
 * infinite by a very large number (MAXSEQ).
 * For simplicity, when bytes are converted to packets, fractional packets 
 * are always rounded up.  
 */
void Tfrc_CR_Agent::sendmsg(int nbytes, const char* /*flags*/)
{
	if (nbytes == -1 && maxseq_ < MAXSEQ)
		advanceby(MAXSEQ - maxseq_);
	else if (size_ > 0) {
		int npkts = int(nbytes/size_);
		npkts += (nbytes%size_ ? 1 : 0);
		// maxqueue was added by Tom Phelan, to control the
		//   transmit queue from the application.
		if ((maxseq_ - seqno_) < maxqueue_) {
			advanceby(npkts);
		}
	}
}


void Tfrc_CR_Agent::advanceby(int delta)
{
	maxseq_ += delta;

	if (seqno_ == -1) {
		// if no packets hve been sent so far, call start. 
		start();
	} else if (datalimited_ && maxseq_ > seqno_) {
		// We were data-limited - send a packet now!
		// The old code always waited for a timer to expire!!
		datalimited_ = 0;
		lastlimited_ = Scheduler::instance().clock();
		all_idle_ = 0;
		if (!oldCode_) {
			sendpkt();
		}
	}
} 

int Tfrc_CR_Agent::command(int argc, const char*const* argv)
{
	if (argc==2) {
		// are we an infinite sender?
		if ( (strcmp(argv[1],"start")==0) && (SndrType_ == 0)) {
			start();
			return TCL_OK;
		}
		if (strcmp(argv[1],"stop")==0) {
			stop();
			return TCL_OK;
		}
	}
	if ((argc == 3) && (SndrType_ == 1)) {
		// or do we need an FTP type app? 
		if (strcmp(argv[1], "advance") == 0) {
			int newseq = atoi(argv[2]);
			// THIS ISN"T USED.
			// newseq: new sequence
			// seqno_: next sequence to be sent
			// maxseq_: max seq_  produced by app so far.
			if (newseq > maxseq_)
				advanceby(newseq - maxseq_);
			return (TCL_OK);
		}
		if (strcmp(argv[1], "advanceby") == 0) {
			advanceby(atoi(argv[2]));
			return (TCL_OK);
		}
		if (strcmp(argv[1], "set-pu-model") == 0) {
			pumodel_ = (PUmodel *) TclObject::lookup(argv[2]);
			return (TCL_OK);
		}
		if (strcmp(argv[1], "set-repository") == 0) {
			repository_ = (Repository *) TclObject::lookup(argv[2]);
			return (TCL_OK);
		}
	}
	return (Agent::command(argc, argv));
}

void Tfrc_CR_Agent::start()
{

	printStatus_=1;
	infoDumpDebug_ = 0; /*Abdulla*/
	/*Abdulla*/
	pFile = fopen("senderdump.txt", "w");
	rate_at_pause_time_=0;
	pausedPU_ = false;
	puDoubleCheckAllowed_ = true;
	pendingPUCheck_=false;
	numHopsToDest_=-1;
	pausedPU_at_time_=0;
	issued_check_time_=0;
	avgPUACKIntervalIndex_=0;
	avgPUACKInterval_=0;
	avgACKIntervalIndex_=0;
	avgACKInterval_=0;
	timeLastACKrcv_=0;
	timeLastPUACKrcv_=0;
	inPUArea_=false;
	is_slow_start = SLOW_START_NO;
	nextPUOffTime_ = 0;
	avgRTT_ = 0;
	m2RTT_=0;
	deltaRTT_=0;
	indexRTT_ = 0;
	seqno_=0;				
	rate_ = InitRate_;
	delta_ = 0;
	oldrate_ = rate_;  
	rate_change_ = SLOW_START;
	UrgentFlag = 1;
	rtt_=0;	 
	sqrtrtt_=1;
	rttcur_=1;
	tzero_ = 0;
	last_change_=0;
	maxrate_ = 0; 
	ss_maxrate_ = 0;
	ndatapack_=0;
	ndatabytes_ = 0;
	true_loss_rate_ = 0;
	active_ = 1; 
	round_id = 0;
	heavyrounds_ = 0;
	t_srtt_ = int(srtt_init_/tcp_tick_) << T_SRTT_BITS;
	t_rttvar_ = int(rttvar_init_/tcp_tick_) << T_RTTVAR_BITS;
	t_rtxcur_ = rtxcur_init_;
	rcvrate = 0 ;
	all_idle_ = 0;
	flost_=0;
	mysendrate_=0;

	first_pkt_rcvd = 0 ;
	// send the first packet
	sendpkt();
	// ... at initial rate
	send_timer_.resched(size_/rate_);
	// ... and start timer so we can cut rate 
	// in half if we do not get feedback
	if (algo!=WALI2)
		NoFeedbacktimer_.resched(2*size_/rate_);
	else NoFeedbacktimer_.resched(5);
}

void Tfrc_CR_Agent::resetVars() {
	send_timer_.force_cancel();
	printStatus_=1;
	infoDumpDebug_ = 0; /*Abdulla*/
	/*Abdulla*/
	//pFile = fopen("senderdump.txt", "w");
	pausedPU_ = false;
	puDoubleCheckAllowed_ = true;
	pendingPUCheck_=false;
	rate_at_pause_time_=0;
	pausedPU_at_time_=0;
	issued_check_time_=0;
	numHopsToDest_=-1;
	inPUArea_=false;
	avgPUACKInterval_=0;
	timeLastPUACKrcv_=0;
	avgPUACKIntervalIndex_=0;
	avgACKInterval_=0;
	timeLastACKrcv_=0;
	avgACKIntervalIndex_=0;
	is_slow_start = SLOW_START_REQ;
	nextPUOffTime_ = 0;
	//avgRTT_ = 0;
	//indexRTT_ = 0;
	seqno_=0;
	rate_ = InitRate_;
	delta_ = 0;
	oldrate_ = rate_;
	rate_change_ = SLOW_START;
	UrgentFlag = 1;
	rtt_=0;
	sqrtrtt_=1;
	rttcur_=1;
	tzero_ = 0;
	last_change_=0;
	maxrate_ = 0;
	ss_maxrate_ = 0;
	ndatapack_=0;
	ndatabytes_ = 0;
	true_loss_rate_ = 0;
	active_ = 1;
	round_id = 0;
	heavyrounds_ = 0;
	t_srtt_ = int(srtt_init_/tcp_tick_) << T_SRTT_BITS;
	t_rttvar_ = int(rttvar_init_/tcp_tick_) << T_RTTVAR_BITS;
	t_rtxcur_ = rtxcur_init_;
	rcvrate = 0 ;
	all_idle_ = 0;

	first_pkt_rcvd = 0 ;
	// send the first packet
	sendpkt();
	// ... at initial rate
	mysendrate_=0;
	send_timer_.resched(size_/rate_);
	// ... and start timer so we can cut rate
	// in half if we do not get feedback
	//NoFeedbacktimer_.resched(2*size_/rate_);
	if (indexRTT_!=0) {
		NoFeedbacktimer_.resched(avgRTT_+(4*sqrt(m2RTT_/indexRTT_)));
	} else {
		if (algo!=WALI2) NoFeedbacktimer_.resched(2*size_/rate_);
		else NoFeedbacktimer_.resched(5);
	}
}

void Tfrc_CR_Agent::stop()
{
	active_ = 0;
	if (idleFix_) 
		datalimited_ = 1;
	send_timer_.force_cancel();

	/*Abdulla*/
	fclose(pFile);
}

void Tfrc_CR_Agent::nextpkt()
{

	double next = -1;
	double xrate = -1; 

	if (SndrType_ == 0) {
		sendpkt();
	}
	else {
		if (maxseq_ > seqno_) {
			sendpkt(); //sends here, then reschedules down there
		} else
			datalimited_ = 1;
		if (debug_) {
			double now = Scheduler::instance().clock();
			printf("Time: %5.2f Datalimited now. maxseq = %i seqno = %i\n", now, maxseq_, seqno_);
		}
	}

	// If slow_increase_ is set, then during slow start, we increase rate
	// slowly - by amount delta per packet 
	// SALLY
	//    double now = Scheduler::instance().clock(); //notused
	// SALLY
	if (slow_increase_ && round_id > 2 && (rate_change_ == SLOW_START) //abdulla: not using this
			&& (oldrate_+SMALLFLOAT< rate_)) {
		oldrate_ = oldrate_ + delta_;
		xrate = oldrate_;
	} else {
		if (ca_) {
			if (debug_) printf("SQRT: now: %5.2f factor: %5.2f\n", Scheduler::instance().clock(), sqrtrtt_/sqrt(rttcur_));
			xrate = rate_ * sqrtrtt_/sqrt(rttcur_);
		} else //abdulla: this is our case
			xrate = rate_;
	}
	printf("%f - sender_ xrate = %f\n", Scheduler::instance().clock(), xrate);
	if (xrate > SMALLFLOAT) { //this thing checks if rate is bigger than 0
		next = size_/xrate; //next is the time interval
		if (voip_) {
			double min_interval = 1.0/voip_max_pkt_rate_;
			if (next < min_interval)
				next = min_interval;
		}
		//
		// randomize between next*(1 +/- woverhead_) 
		//
		next = next*(2*overhead_*Random::uniform()-overhead_+1);
		double now = Scheduler::instance().clock();

		if (next > SMALLFLOAT) {
			if (!pausedPU_) {
				double now = Scheduler::instance().clock();
				double scheduledTime = now + next;
				if (scheduledTime>(nextPUOffTime_-avgRTT_) && scheduledTime<nextPUOffTime_+TIME_TO_WAIT_AFTER_PU) {
					//don't schedule !
					//if (debug_) printf("canceling schedule at time: %f for event at time %f\n", now, scheduledTime);
				} else {
					send_timer_.resched(next);
				}
			} else {
				send_timer_.resched(next); //will call itself.
			}
		}
		else
			send_timer_.resched(SMALLFLOAT);
	}
}

void Tfrc_CR_Agent::update_rtt (double tao, double now) //too complex to understand now for abdulla :(
{
	/* the TCP update */
	t_rtt_ = int((now-tao) /tcp_tick_ + 0.5);
	if (t_rtt_==0) t_rtt_=1;
	if (t_srtt_ != 0) {
		register short rtt_delta;
		rtt_delta = t_rtt_ - (t_srtt_ >> T_SRTT_BITS);    
		if ((t_srtt_ += rtt_delta) <= 0)    
			t_srtt_ = 1;
		if (rtt_delta < 0)
			rtt_delta = -rtt_delta;
		rtt_delta -= (t_rttvar_ >> T_RTTVAR_BITS);
		if ((t_rttvar_ += rtt_delta) <= 0)
			t_rttvar_ = 1;
	} else {
		t_srtt_ = t_rtt_ << T_SRTT_BITS;		
		t_rttvar_ = t_rtt_ << (T_RTTVAR_BITS-1);	
	}
	t_rtxcur_ = (((t_rttvar_ << (rttvar_exp_ + (T_SRTT_BITS - T_RTTVAR_BITS))) + t_srtt_)  >> T_SRTT_BITS ) * tcp_tick_;
	tzero_=t_rtxcur_;
	if (tzero_ < minrto_)
		tzero_ = minrto_;

	/* fine grained RTT estimate for use in the equation */
	if (rtt_ > 0) {
		rtt_ = df_*rtt_ + ((1-df_)*(now - tao));
		sqrtrtt_ = df_*sqrtrtt_ + ((1-df_)*sqrt(now - tao));
	} else {
		rtt_ = now - tao;
		sqrtrtt_ = sqrt(now - tao);
	}
	rttcur_ = now - tao;
}

/*
 * Receive a status report from the receiver.
 */
void Tfrc_CR_Agent::recv(Packet *pkt, Handler *)
{
	/*
	 * currently canceling slow-start at pu_off time. this assumes that an ACK is an indication that intermediate
	 * nodes have switched channels around the PU
	 */
	/*if (pausedPU_) {
		//printf("%f - received packet while paused\n", Scheduler::instance().clock());
		pausedPU_ = false; //TODO: might have to reconsider this. what if ACK is coming from nodes after PU? will nofeedbacktimeout even get triggered and pausedPU get set?
		puDoubleCheckAllowed_=false; //this is already set to false if we're paused.
		if (rate_at_pause_time_!=0) {
			//printf("%f - restoring rate to %f\n", Scheduler::instance().clock(), rate_at_pause_time_);
			rate_=rate_at_pause_time_;
		}
	}*/
	if (pendingPUCheck_) {
		//if (Scheduler::instance().clock()>issued_check_time_+(avgRTT_+4*sqrt(m2RTT_/indexRTT_))) {
			printf("%f - canceling double checking \n", Scheduler::instance().clock());
			check_pu_off_timer_.force_cancel();
		//}
		pendingPUCheck_=false;
	}
	double now = Scheduler::instance().clock();
	hdr_tfrc_cr_ack *nck = hdr_tfrc_cr_ack::access(pkt);
	if (nck->slow_start_acked==1) {
		printf("%f - slow start acknowledged at sender\n", now);
		if (is_slow_start==SLOW_START_NO) { printf("%f - acked twice\n"); return; }
		is_slow_start=SLOW_START_NO;
		avgRTT_ = 0;
		indexRTT_ = 0;
		deltaRTT_=0;
		m2RTT_=0;
	}
	if (is_slow_start==SLOW_START_REQ && nck->slow_start_acked!=1) {
		printf("%f - received unwanted ACK\n", now);
		return;
	}

	if (pausedPU_) return;

	//double ts = nck->timestamp_echo;
	double ts = nck->timestamp_echo + nck->timestamp_offset;
	double rate_since_last_report = nck->rate_since_last_report;
	// double NumFeedback_ = nck->NumFeedback_;
	double flost = nck->flost;
	flost_ = flost;
	int losses = nck->losses;
	true_loss_rate_ = nck->true_loss;



	//printf("current rate=%f true_loss rate=%f losses=%i flost=%f rate since last report %f and maxrate=%5.2f\n", rate_, nck->true_loss, losses, flost, rate_since_last_report, maxrate_);

	round_id ++ ;
	UrgentFlag = 0;
	if (round_id > 1 && rate_since_last_report > 0) {
		/* compute the max rate for slow-start as two times rcv rate */ 
		ss_maxrate_ = 2*rate_since_last_report*size_;
		if (conservative_) { 
			if (losses >= 1) {
				/* there was a loss in the most recent RTT */
				if (debug_) printf("time: %5.2f losses: %d rate %5.2f\n", 
						now, losses, rate_since_last_report);
				maxrate_ = rate_since_last_report*size_;
			} else {
				/* there was no loss in the most recent RTT */
				maxrate_ = scmult_*rate_since_last_report*size_;
			}
			if (debug_) printf("time: %5.2f losses: %d rate %5.2f maxrate: %5.2f\n", now, losses, rate_since_last_report, maxrate_);
		} else {
			//this affects INC_RATE directly
			if (algo!=WALI2) maxrate_ = 2*rate_since_last_report*size_;
			else maxrate_ = 20000*rate_since_last_report*size_; //effectively disables it
			printf("%f which13 maxrate_=%f last_report %f size %i\n", now, maxrate_, rate_since_last_report, size_);
		}
	} else {
		ss_maxrate_ = 0;
		maxrate_ = 0; 
	}

	/* update the round trip time */
	update_rtt (ts, now);

	printf("%f - RTT equals = %f\n", now, now-ts);

	indexRTT_+=1;
	deltaRTT_=rttcur_-avgRTT_;
	avgRTT_=avgRTT_+(deltaRTT_/indexRTT_);
	m2RTT_=m2RTT_+(deltaRTT_*(rttcur_-avgRTT_));
	//printf("%f - cur: %f average RTT: %f and std dev=%f\n", now, rttcur_, avgRTT_, sqrt(m2RTT_/indexRTT_));


	if (inPUArea_) {
		if (timeLastPUACKrcv_==0) {
			timeLastPUACKrcv_=now;
		} else {
			avgPUACKIntervalIndex_++;
			avgPUACKInterval_ = avgPUACKInterval_ * (avgPUACKIntervalIndex_-1);
			avgPUACKInterval_ = avgPUACKInterval_ + (now-timeLastPUACKrcv_);
			avgPUACKInterval_ = avgPUACKInterval_/avgPUACKIntervalIndex_;
			timeLastPUACKrcv_=now;
		}
	} else {
		if (timeLastACKrcv_==0) {
			timeLastACKrcv_=now;
		} else {
			avgACKIntervalIndex_++;
			avgACKInterval_ = avgACKInterval_ * (avgACKIntervalIndex_ - 1 );
			avgACKInterval_ = avgACKInterval_ + (now-timeLastACKrcv_);
			avgACKInterval_ = avgACKInterval_/avgACKIntervalIndex_;
			timeLastACKrcv_=now;
		}
	}

	/* .. and estimate of fair rate */
	if (voip_ != 1) {
		// From RFC 3714:
		// The voip flow gets to send at the same rate as
		//  a TCP flow with 1460-byte packets.
		fsize_ = size_;
	}
	//flost=0.000085;
	//flost_=0.000085;
	//flost = flost_ = 0.00009;
	rcvrate = p_to_b(flost, rtt_, tzero_, fsize_, bval_);
	printf("%f - sender_ rcvrate_ %f rtt_ %f flost %f bval %i fsize %i tzero %f\n", now, rcvrate, rtt_ , flost, bval_, fsize_, tzero_);
	// rcvrate is in bytes per second, based on fairness with a    
	// TCP connection with the same packet size size_.	      
	if (voip_) {
		// Subtract the bandwidth used by headers.
		double temp = rcvrate*(size_/(1.0*headersize_+size_));
		rcvrate = temp;
	}

	/* if we get no more feedback for some time, cut rate in half */
	double t = 2*rtt_ ;
	if (t < 2*size_/rate_)
		t = 2*size_/rate_ ;
	if (algo!=WALI2) NoFeedbacktimer_.resched(t);
	else NoFeedbacktimer_.resched(5);
	/* if we are in slow start and we just saw a loss */
	/* then come out of slow start */
	if (first_pkt_rcvd == 0) {
		first_pkt_rcvd = 1 ; 
		slowstart();
		nextpkt();
	}
	else {
		if (rate_change_ == SLOW_START) {
			if (flost > 0) {
				rate_change_ = OUT_OF_SLOW_START;
				oldrate_ = rate_ = rcvrate;
			}
			else {
				slowstart();
				nextpkt();
			}
		}
		else {
			printf("%f rate = %f and rcvrate = %f flost = %f rtt = %f\n", now, rate_, rcvrate, flost, rtt_);
			if (rcvrate>rate_) {
				increase_rate(flost);
			}
			else {
				decrease_rate ();
			}
		}
	}
	mysendrate_ = rate_;
	if (printStatus_) {
		printf("Received a packet: size: %i\n", size_);
		printf("time: %5.2f rate: %5.2f\n", now, rate_);
		double packetrate = rate_ * rtt_ / size_;
		printf("time: %5.2f packetrate: %5.2f\n", now, packetrate);
		double maxrate = maxrate_ * rtt_ / size_;
		printf("time: %5.2f maxrate: %5.2f\n", now, maxrate);
	}
	Packet::free(pkt);
}

/*
 * Calculate initial sending rate from RFC 3390.
 */
double Tfrc_CR_Agent::rfc3390(int size)
{
	if (size_ <= 1095) {
		return (4.0);
	} else if (size_ < 2190) {
		return (3.0);
	} else {
		return (2.0);
	}
}

/*
 * Used in setting the initial rate.
 * This is from TcpAgent::initial_wnd().
 */
double Tfrc_CR_Agent::initial_rate()
{
	if (rate_init_option_ == 1) {
		// init_option = 1: static initial rate of rate_init_
		return (rate_init_);
	}
	else if (rate_init_option_ == 2) {
		// do initial rate according to RFC 3390.
		return (rfc3390(size_));
	}
	// XXX what should we return here???
	fprintf(stderr, "Wrong number of rate_init_option_ %d\n",
			rate_init_option_);
	abort();
	return (2.0); // XXX make msvc happy.
}


// ss_maxrate_ = 2*rate_since_last_report*size_;
// rate_: the rate set from the last pass through slowstart()
void Tfrc_CR_Agent::slowstart ()
{
	double now = Scheduler::instance().clock(); 
	double initrate = initial_rate()*size_/rtt_;
	// If slow_increase_ is set to true, delta is used so that 
	//  the rate increases slowly to new value over an RTT. 
	if (debug_) printf("SlowStart: round_id: %d rate: %5.2f ss_maxrate_: %5.2f\n", round_id, rate_, ss_maxrate_);
	if (round_id <=1 || (round_id == 2 && initial_rate() > 1)) {
		// We don't have a good rate report yet, so keep to  
		//   the initial rate.				     
		oldrate_ = rate_;
		if (rate_ < initrate) rate_ = initrate;
		delta_ = (rate_ - oldrate_)/(rate_*rtt_/size_);
		last_change_=now;
	} else if (ss_maxrate_ > 0) {
		if (idleFix_ && (datalimited_ || lastlimited_ > now - 1.5*rtt_)
				&& ss_maxrate_ < initrate) {
			// Datalimited recently, and maxrate is small.
			// Don't be limited by maxrate to less that initrate.
			oldrate_ = rate_;
			if (rate_ < initrate) rate_ = initrate;
			delta_ = (rate_ - oldrate_)/(rate_*rtt_/size_);
			last_change_=now;
		} else if (rate_ < ss_maxrate_ && 
				now - last_change_ > rtt_) {
			// Not limited by maxrate, and time to increase.
			// Multiply the rate by ssmult_, if maxrate allows.
			oldrate_ = rate_;
			if (ssmult_*rate_ > ss_maxrate_) 
				rate_ = ss_maxrate_;
			else rate_ = ssmult_*rate_;
			if (rate_ < size_/rtt_) 
				rate_ = size_/rtt_; 
			delta_ = (rate_ - oldrate_)/(rate_*rtt_/size_);
			last_change_=now;
		} else if (rate_ > ss_maxrate_) {
			// Limited by maxrate.  
			rate_ = oldrate_ = ss_maxrate_/2.0;
			delta_ = 0;
			last_change_=now;
		} 
	} else {
		// If we get here, ss_maxrate <= 0, so the receive rate is 0.
		// We should go back to a very small sending rate!!!
		oldrate_ = rate_;
		rate_ = size_/rtt_; 
		delta_ = 0;
		last_change_=now;
	}
	if (debug_) printf("SlowStart: now: %5.2f rate: %5.2f delta: %5.2f\n", now, rate_, delta_);
	if (printStatus_) {
		double rate = rate_ * rtt_ / size_;
		printf("SlowStart: now: %5.2f rate: %5.2f ss_maxrate: %5.2f delta: %5.2f\n", now, rate, ss_maxrate_, delta_);
	}
}

void Tfrc_CR_Agent::increase_rate (double p)
{
	double now = Scheduler::instance().clock();
	double maximumrate;
	double mult = (now-last_change_)/rtt_ ;
	//if (mult > 2) mult = 2 ;
	rate_ = rate_ + (size_/rtt_)*mult ;
	if (datalimited_ || lastlimited_ > now - 1.5*rtt_) {
		// Modified by Sally on 3/10/2006
		// If the sender has been datalimited, rate should be
		//   at least the initial rate, when increasing rate.
		double init_rate = initial_rate()*size_/rtt_;
		maximumrate = (maxrate_>init_rate)?maxrate_:init_rate ;
	} else {
		maximumrate = (maxrate_>size_/rtt_)?maxrate_:size_/rtt_ ;
	}
	//printf("INC rcvrate = %f maximumrate = %f rate_ = %f\n", rcvrate, maximumrate, rate_);
	maximumrate = (maximumrate>rcvrate)?rcvrate:maximumrate;
	rate_ = (rate_ > maximumrate)?maximumrate:rate_ ;
	rate_ = rcvrate;
	rate_change_ = CONG_AVOID;
	last_change_ = now;
	heavyrounds_ = 0;
	//if (debug_) printf("Increase: now: %5.2f rate: %5.2f lastlimited: %5.2f rtt: %5.2f\n", now, rate_, lastlimited_, rtt_);
	//if (printStatus_) {
	//	double rate = rate_ * rtt_ / size_;
	//	printf("Increase: now: %5.2f rate: %5.2f lastlimited: %5.2f rtt: %5.2f size:%i\n", now, rate, lastlimited_, rtt_, size_);
	//}
}       

void Tfrc_CR_Agent::decrease_rate ()
{
	double now = Scheduler::instance().clock(); 
	rate_ = rcvrate;
	double maximumrate = (maxrate_>size_/rtt_)?maxrate_:size_/rtt_ ;

	// Allow sending rate to be greater than maximumrate
	//   (which is by default twice the receiving rate)
	//   for at most maxHeavyRounds_ rounds.
	if (rate_ > maximumrate)
		heavyrounds_++;
	else
		heavyrounds_ = 0;
	if (heavyrounds_ > maxHeavyRounds_) {
		rate_ = (rate_ > maximumrate)?maximumrate:rate_ ;
	}

	rate_change_ = RATE_DECREASE;
	last_change_ = now;
	if (debug_) printf("Decrease: now: %5.2f rate: %5.2f rtt: %5.2f\n", now, rate_, rtt_);
	if (printStatus_) {
		double rate = rate_ * rtt_ / size_;
		printf("Decrease: now: %5.2f rate: %5.2f rtt: %5.2f\n", now, rate, rtt_);
	}

}

void Tfrc_CR_Agent::sendpkt()
{
	if (active_) {

		double now = Scheduler::instance().clock();
		Packet* p = allocpkt();
		hdr_tfrc_cr *tfrch = hdr_tfrc_cr::access(p);
		hdr_flags* hf = hdr_flags::access(p);
		struct hdr_ip *ih = HDR_IP(p);
		//printf("destination ip=%i and number of hops to it %i\n", ih->dst_.addr_, repository_->get_number_hops(ih->dst_.addr_));
		//numHopsToDest_ = repository_->get_number_hops(ih->dst_.addr_);

		if (ecn_) {
			hf->ect() = 1;  // ECN-capable transport
		}
		tfrch->seqno=seqno_++;
		tfrch->timestamp=Scheduler::instance().clock();
		tfrch->rtt=rtt_;
		tfrch->tzero=tzero_;
		tfrch->rate=rate_;
		tfrch->psize=size_;
		tfrch->fsize=fsize_;
		tfrch->UrgentFlag=UrgentFlag;
		tfrch->round_id=round_id;
		if (is_slow_start==SLOW_START_REQ) {
			printf("%f - requesting slow start at sender\n", now);
			tfrch->is_slow_start = 1;
		} else {
			tfrch->is_slow_start = 0;
		}
		ndatapack_++;
		ndatabytes_ += size_;
		if (useHeaders_ == true) {
			hdr_cmn::access(p)->size() += headersize_;
		}
		last_pkt_time_ = now;
		send(p, 0);
	} else {
		//printf("sending packet fail due to it being inactive\n");
	}
}


/*
 * RFC 3448:
 * "If the sender has been idle since this nofeedback timer was set and
 * X_recv is less than four packets per round-trip time, then X_recv
 * should not be halved in response to the timer expiration.  This
 * ensures that the allowed sending rate is never reduced to less than
 * two packets per round-trip time as a result of an idle period."
 */

void Tfrc_CR_Agent::reduce_rate_on_no_feedback()
{

	bool is_pu_active = pumodel_->check_active(Scheduler::instance().clock(), 0.001f);
	printf("%f - no feedback and PU is %s rate is %f # of hops to it %i\n", Scheduler::instance().clock(),
			(is_pu_active)?"true":"false",
					rate_, numHopsToDest_);
	if (is_pu_active && puDoubleCheckAllowed_ && !pendingPUCheck_) {
		double recheck_time = (avgACKInterval_<(2*avgRTT_))?(2*avgRTT_):avgACKInterval_;
		printf("%f - no feedback is double checking in %f\n", Scheduler::instance().clock(), recheck_time);
		pendingPUCheck_=true;
		issued_check_time_=Scheduler::instance().clock();
		check_pu_off_timer_.resched(recheck_time);
	} else {
		issued_check_time_=0;
	}
	double now = Scheduler::instance().clock();
	if (rate_change_ != SLOW_START)
		// "if" statement added by Sally on 9/25/06,
		// so that an idle report doesn't kick us out of
		// slow-start!
		rate_change_ = RATE_DECREASE; 
	if (oldCode_ || (!all_idle_ && !datalimited_ && !pausedPU_ && is_slow_start!=SLOW_START_REQ)) {
		// if we are not datalimited
		rate_*=0.5;
		printf("half\n");
	} else if ((datalimited_ || all_idle_) && rate_init_option_ == 1) { 
		// all_idle_: the sender has been datalimited since the 
		//    timer was set
		//  Don't reduce rate below rate_init_ * size_/rtt_.
		if (rate_ > 2.0 * rate_init_ * size_/rtt_ ) {
			rate_*=0.5;
		}
	} else if ((datalimited_ || all_idle_) && rate_init_option_ == 2) {
		// Don't reduce rate below the RFC3390 rate.
		if (rate_ > 2.0 * rfc3390(size_) * size_/rtt_ ) {
			rate_*=0.5;
		} else if ( rate_ > rfc3390(size_) * size_/rtt_ ) {
			rate_ = rfc3390(size_) * size_/rtt_;
		}
	}
	if (debug_) printf("NO FEEDBACK: time: %5.2f rate: %5.2f all_idle: %d\n", now, rate_, all_idle_);
	if (algo!=WALI2) UrgentFlag = 1;
	round_id ++ ;
	double t = 2*rtt_ ; 
	// Set the nofeedback timer.
	if (t < 2*size_/rate_) 
		t = 2*size_/rate_ ; 
	//if we're in slow start. change recheck value
	if (is_slow_start==SLOW_START_REQ && !is_pu_active && indexRTT_!=0) {
		t=avgRTT_+4*sqrt(m2RTT_/indexRTT_);
		//printf("%f - slow start - 2rescheduling feedback timeout at %f\n", Scheduler::instance().clock(), Scheduler::instance().clock()+(avgRTT_+(4*sqrt(m2RTT_/indexRTT_))));
	} else {
		//printf("%f - no slow start - rescheduling feedback\n", Scheduler::instance().clock());
	}
	if (algo!=WALI2) NoFeedbacktimer_.resched(t);
	else NoFeedbacktimer_.resched(5);
	//printf("%f - no feedback timer reschedule: %f\n", now, now+t );
	if (datalimited_) {
		all_idle_ = 1;
		if (debug_) printf("Time: %5.2f Datalimited now.\n", now);
	}
	if (now>nextPUOffTime_-avgRTT_ && now<nextPUOffTime_+TIME_TO_WAIT_AFTER_PU && puoff_timer_.status()==TIMER_PENDING) { } //dont send anything in that sensitive time period
	else nextpkt();
}

void Tfrc_CR_SendTimer::expire(Event *) {
	a_->nextpkt();
}

void Tfrc_CR_PUOffTimer::expire(Event *) {
	//printf("%f - fired pu expire event\n", Scheduler::instance().clock());
	//if (a_->avgPUACKIntervalIndex_!=0)
	//	printf("%f - fired off PU expire - rate now %f pudoublecheckallowed = %s ackInterval %f puAckInterval %f pupacks/time %f\n", Scheduler::instance().clock(), a_->rate_,
	//			(a_->puDoubleCheckAllowed_)?"true":"false", a_->avgACKInterval_, a_->avgPUACKInterval_, ((Scheduler::instance().clock()-a_->pausedPU_at_time_)/a_->avgPUACKIntervalIndex_));
	//else
	//	printf("%f - fired off PU expire - rate now %f pudoublecheckallowed = %s ackInterval %f puAckInterval %f paused_pu_at_time=%f last ack recv %f\n", Scheduler::instance().clock(), a_->rate_,
	//				(a_->puDoubleCheckAllowed_)?"true":"false", a_->avgACKInterval_, a_->avgPUACKInterval_, a_->pausedPU_at_time_, a_->timeLastACKrcv_);

	a_->inPUArea_=false; //TODO: might wanna run a DB check to see if there is an overlap PU.

	if (!a_->puDoubleCheckAllowed_) { //PU double check is not allowed. i.e. nodes supposedly got around the blocked spectrum
		//the following if makes sure we slow start if we received a "slipped" ACK.
		double now = Scheduler::instance().clock();
		//if it has been more than twice the avg ACK interarrival time and no acks were received meanwhile
		if ( ((now-a_->timeLastACKrcv_)>(2*a_->avgACKInterval_) && a_->avgPUACKInterval_==0) ||
				//or if we have a PU ack avg, and its bigger than 2*avg normal acks (make sure that it hasn't been long since it was last recorded
				((a_->avgPUACKInterval_!=0) && (a_->avgPUACKInterval_>2*a_->avgACKInterval_) && (now-a_->timeLastPUACKrcv_<2*a_->avgPUACKInterval_)) ||
				//or if we have a a small PU ack avg, but time since last rcv is very big. TODO: what if we receive at the end two consecutive packets?
				(a_->avgPUACKInterval_!=0 && now-a_->timeLastPUACKrcv_>3*a_->avgACKInterval_)) {
			printf("%f - we're enforcing a slow start avgACKInterval = %f avgPUACKInterval = %f now-lastACK=%f now-lastPUACK=%f\n", now,
					a_->avgACKInterval_, a_->avgPUACKInterval_, (now-a_->timeLastACKrcv_), (now-a_->timeLastPUACKrcv_));
			a_->send_timer_.force_cancel();
			a_->resetVars();
		}
		else if (a_->avgPUACKIntervalIndex_!=0) {
			if ( ((now-a_->pausedPU_at_time_)/a_->avgPUACKIntervalIndex_) > 2*a_->avgPUACKInterval_ ) { //if average received acks during pu period is bigger than calculated PU ack interval
				printf("%f - we're enforcing a slow start avgACKInterval = %f avgPUACKInterval = %f now-lastACK=%f now-lastPUACK=%f\n", now,
									a_->avgACKInterval_, a_->avgPUACKInterval_, (now-a_->timeLastACKrcv_), (now-a_->timeLastPUACKrcv_));
				a_->send_timer_.force_cancel();
				a_->resetVars();
			}
		}
		a_->pausedPU_=false;
		a_->puDoubleCheckAllowed_=true;
		a_->nextPUOffTime_=0;
	}
	else if (a_->pausedPU_) { //just in case
		//printf("slow start after PU exist at: %f\n", Scheduler::instance().clock());
		a_->resetVars();
	}
	//we have to reset these values regardless of outcome
	a_->timeLastPUACKrcv_=0;
	a_->avgPUACKIntervalIndex_=0;
	a_->avgPUACKInterval_=0;
	//a_->nextpkt();
	//a_->pausedPU_=false;
	//a_->NoFeedbacktimer_.resched(0.11);
}

void Tfrc_CR_CheckPUOff::expire(Event *) {
	bool is_pu_active = a_->pumodel_->check_active(Scheduler::instance().clock(), 0.001f);
	printf("%f - double checked no feedback and PU is %s rate is %f # of hops to it %i\n", Scheduler::instance().clock(),
			(is_pu_active)?"true":"false",
					a_->rate_, a_->numHopsToDest_);
	a_->pendingPUCheck_=false;
	//a_->puDoubleCheckAllowed_=false; //we can double check again only after pu exists
	//printf("%f - pudoublecheckallowed will always be %s\n", Scheduler::instance().clock(), (a_->puDoubleCheckAllowed_)?"true":"false");
	if (is_pu_active) {
		if (!a_->pausedPU_) { //schedule only once
			a_->nextPUOffTime_ = a_->pumodel_->get_next_off_time(Scheduler::instance().clock());
			double time_PU_off = a_->nextPUOffTime_-Scheduler::instance().clock()+TIME_TO_WAIT_AFTER_PU;
			printf("%f - time to fire off PU event %f\n", Scheduler::instance().clock(), time_PU_off);
			if (time_PU_off>0) {
				a_->puoff_timer_.resched(time_PU_off);
				a_->puDoubleCheckAllowed_=false;
			}
			a_->rate_at_pause_time_=a_->rate_;
			a_->inPUArea_=true;
			a_->pausedPU_at_time_=Scheduler::instance().clock();
			a_->pausedPU_=true;
			if (a_->avgRTT_==0) a_->rate_=100;
			else a_->rate_=a_->size_/(6*a_->avgRTT_);
			printf("%f - reduced rate to %f\n", a_->pausedPU_at_time_, a_->rate_);

		}

	}
}

void Tfrc_CR_NoFeedbackTimer::expire(Event *) {
	a_->reduce_rate_on_no_feedback ();
	// TODO: if the first (SYN) packet was dropped, then don't use
	//   the larger initial rates from RFC 3390:
	// if (highest_ack_ == -1 && wnd_init_option_ == 2)
	//     wnd_init_option_ = 1;

}
