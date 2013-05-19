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

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <math.h>
 
#include "tfrc-cr-sink.h"
//#include "formula-with-inverse.h"
#include "flags.h"

static class Tfrc_CR_SinkClass : public TclClass {
public:
  	Tfrc_CR_SinkClass() : TclClass("Agent/TFRC_CR_Sink") {}
  	TclObject* create(int, const char*const*) {
     		return (new Tfrc_CR_SinkAgent());
  	}
} class_tfrcSink; 


Tfrc_CR_SinkAgent::Tfrc_CR_SinkAgent() : Agent(PT_TFRC_CR_ACK), nack_timer_(this)
{
	bind("packetSize_", &size_);
	bind("InitHistorySize_", &hsz);
	bind("NumFeedback_", &NumFeedback_);
	bind ("AdjustHistoryAfterSS_", &adjust_history_after_ss);
	bind ("printLoss_", &printLoss_);
	bind ("algo_", &algo); // algo for loss estimation //this is set to 1 by default. (aka. WALI)
	bind ("PreciseLoss_", &PreciseLoss_);
	bind ("numPkts_", &numPkts_);

	// for WALI ONLY
	bind ("NumSamples_", &numsamples);
	bind ("discount_", &discount);
	bind ("smooth_", &smooth_);
	bind ("ShortIntervals_", &ShortIntervals_);
	bind ("ShortRtts_", &ShortRtts_);

	//expose samples standard deviation to tcl
	bind ("SamplesStdDev_", &samplesStdDev_);
	bind ("mostRecentSample_", &mostRecentSample_);
	bind ("binMultiplier", &bin_multiplier_);
	//expose floss to TCL
	bind ("flost_", &flost_);

	// EWMA use only
	bind ("history_", &history); // EWMA history

	// for RBPH use only
	bind("minlc_", &minlc); 

	bind("bytes_", &bytes_);

	bind("rtt_", &rtt_);

	bind("droppedpercent_", &percentageDropped);

	rtt_ =  0; 
	tzero_ = 0;
	last_timestamp_ = 0;
	last_arrival_ = 0;
	last_report_sent=0;
	total_received_ = 0;
	total_losses_ = 0;
	total_dropped_ = 0;

	maxseq = -1;
	maxseqList = -1;
	rcvd_since_last_report  = 0;
	losses_since_last_report = 0;
	loss_seen_yet = 0;
	lastloss = 0;
	lastloss_round_id = -1 ;
	numPktsSoFar_ = 0;

	rtvec_ = NULL;
	tsvec_ = NULL;
	lossvec_ = NULL;

	// used by WALI and EWMA
	last_sample = 0;

	// used only for WALI 
	false_sample = 0;
	sample = NULL ; 
	weights = NULL ;
	mult = NULL ;
        losses = NULL ;
	count_losses = NULL ;
        num_rtts = NULL ;
	sample_count = 1 ;
	mult_factor_ = 1.0;
	init_WALI_flag = 0;

	// used only for EWMA
	avg_loss_int = -1 ;
	loss_int = 0 ;

	// used only bu RBPH
	sendrate = 0 ; // current send rate

	//pFile_ = fopen("receiverdump.txt", "w");
	last_arrived = 0;
	last_ott = 0;
	is_slow_start = 0;
	//Abdulla
	sample_index=0;
	avg_samples=0;
	delta_samples=0;
	m2_samples=0;
	samplesStdDev_ = 0;
	mostRecentSample_=0;
	flost_=0;
	lastinterval=0;
	percentageDropped=0;

}

void Tfrc_CR_SinkAgent::resetVars() {
	rtt_ =  0;
	tzero_ = 0;
	last_timestamp_ = 0;
	last_arrival_ = 0;
	last_report_sent=0;
	total_received_ = 0;
	total_losses_ = 0;
	total_dropped_ = 0;

	maxseq = -1;
	maxseqList = -1;
	rcvd_since_last_report  = 0;
	losses_since_last_report = 0;
	loss_seen_yet = 0;
	lastloss = 0;
	lastloss_round_id = -1 ;
	numPktsSoFar_ = 0;

	rtvec_ = NULL;
	tsvec_ = NULL;
	lossvec_ = NULL;

	// used by WALI and EWMA
	last_sample = 0;

	// used only for WALI
	false_sample = 0;
	sample = NULL ;
	weights = NULL ;
	mult = NULL ;
        losses = NULL ;
	count_losses = NULL ;
        num_rtts = NULL ;
	sample_count = 1 ;
	mult_factor_ = 1.0;
	init_WALI_flag = 0;

	// used only for EWMA
	avg_loss_int = -1 ;
	loss_int = 0 ;

	// used only bu RBPH
	sendrate = 0 ; // current send rate

	//Abdulla do we reset here?
	//sample_index=0;
	//avg_samples=0;
	//delta_samples=0;
	//m2_samples=0;

	//lastinterval=0;

}

/*
 * This is a new loss event if it is at least an RTT after the beginning
 *   of the last one.
 * If PreciseLoss_ is set, the new_loss also checks that there is a
 *     new round_id.
 * The sender updates the round_id when it receives a new report from
 *   the receiver, and when it reduces its rate after no feedback.
 * Sometimes the rtt estimates can be less than the actual RTT, and
 *   the round_id will catch this.  This can be useful if the actual
 *   RTT increases dramatically.
 */
int Tfrc_CR_SinkAgent::new_loss(int i, double tstamp)
{
	double time_since_last_loss_interval = tsvec_[i%hsz]-lastloss;
	if ((time_since_last_loss_interval > rtt_)
	     && (PreciseLoss_ == 0 || (round_id > lastloss_round_id))) {
		lastloss = tstamp;
		lastloss_round_id = round_id ;
                if (time_since_last_loss_interval < ShortRtts_ * rtt_ &&
				(algo == WALI || algo == WALI2)) {
                        count_losses[0] = 1;
                }
                if (rtt_ > 0 && (algo == WALI || algo == WALI2)) {
                        num_rtts[0] = (int) ceil(time_since_last_loss_interval / rtt_);
                        if (num_rtts[0] < 1) num_rtts[0] = 1;
                }
		return TRUE;
	} else return FALSE;
}

double Tfrc_CR_SinkAgent::estimate_tstamp(int before, int after, int i)
{
	double delta = (tsvec_[after%hsz]-tsvec_[before%hsz])/(after-before) ; 
	double tstamp = tsvec_[before%hsz]+(i-before)*delta ;
	return tstamp;
}

/*
 * Receive new data packet.  If appropriate, generate a new report.
 */
void Tfrc_CR_SinkAgent::recv(Packet *pkt, Handler *)
{
	hdr_tfrc_cr *tfrch = hdr_tfrc_cr::access(pkt);
	hdr_flags* hf = hdr_flags::access(pkt);
	double now = Scheduler::instance().clock();
	double p = -1;
	int ecnEvent = 0;
	int congestionEvent = 0;
	int UrgentFlag = 0;	// send loss report immediately
	//int newdata = 0;	// a new data packet received

	//is source requesting slow-start?
	if (tfrch->is_slow_start==1) {
			printf("%f - slow start at receiver\n", now);
			resetVars();
			is_slow_start = 1;
			UrgentFlag = 1;
	}

	if ((algo == WALI || algo == WALI2) && !init_WALI_flag) {
		init_WALI () ;
	}
	printf("%f - std_ recv packet seq: %i\n", Scheduler::instance().clock(), tfrch->seqno);
	rcvd_since_last_report ++;
	total_received_ ++;
	// bytes_ was added by Tom Phelan, for reporting bytes received.
	bytes_ += hdr_cmn::access(pkt)->size();

	if (last_arrived!=0 && (now-last_arrived<20)) {
		printf("%5.2f inter-arrival %f\n", now, now-last_arrived);
	}
	last_arrived=now;
	double ott = now-tfrch->timestamp;
	printf("%5.2f timesent: %f timenow: %f RTO: %f\n", now, tfrch->timestamp, now, ott-last_ott);
	printf("received packet at receiver: %5.2f\n", now);
	last_ott = ott;


	if (maxseq < 0) {
		// This is the first data packet.
		//newdata = 1;
		maxseq = tfrch->seqno - 1 ;
		maxseqList = tfrch->seqno;
		rtvec_=(double *)malloc(sizeof(double)*hsz);
		tsvec_=(double *)malloc(sizeof(double)*hsz);
		lossvec_=(char *)malloc(sizeof(double)*hsz);
		if (rtvec_ && lossvec_) {
			int i;
			for (i = 0; i < hsz ; i ++) {
				lossvec_[i] = UNKNOWN;
				rtvec_[i] = -1; 
				tsvec_[i] = -1; 
			}
		}
		else {
			printf ("error allocating memory for packet buffers\n");
			abort (); 
		}
	}
	/* for the time being, we will ignore out of order and duplicate 
	   packets etc. */
	int seqno = tfrch->seqno ;
	fsize_ = tfrch->fsize;
	int oldmaxseq = maxseq;
	// if this is the highest packet yet, or an unknown packet
	//   between maxseqList and maxseq  
	if ((seqno > maxseq) || 
	  (seqno > maxseqList && lossvec_[seqno%hsz] == UNKNOWN )) {
		if (seqno > maxseqList + 1)
			++ numPktsSoFar_;
		UrgentFlag = tfrch->UrgentFlag;
		round_id = tfrch->round_id ;
		rtt_=tfrch->rtt;
		tzero_=tfrch->tzero;
		psize_=tfrch->psize;
		sendrate = tfrch->rate;
		last_arrival_=now;
		last_timestamp_=tfrch->timestamp;
		rtvec_[seqno%hsz]=now;	
		tsvec_[seqno%hsz]=last_timestamp_;	
		if (hf->ect() == 1 && hf->ce() == 1) {
			// ECN action
			lossvec_[seqno%hsz] = ECN_RCVD;
			++ total_losses_;
			losses_since_last_report++;
			if (new_loss(seqno, tsvec_[seqno%hsz])) {
				ecnEvent = 1;
				lossvec_[seqno%hsz] = ECNLOST;
			} 
			if (algo == WALI || algo == WALI2) {
                       		++ losses[0];
			}
		} else lossvec_[seqno%hsz] = RCVD;
	}
	printf("%f - urg = %i\n", UrgentFlag);
	if (seqno > maxseq) {
		int i = maxseq + 1;
		while (i < seqno) {
			// Added 3/1/05 in case we have wrapped around
			//   in packet sequence space.
			lossvec_[i%hsz] = UNKNOWN;
			++ i;
			++ total_losses_;
			++ total_dropped_;
		}
	}
	if (seqno > maxseqList && 
	  (ecnEvent || numPktsSoFar_ >= numPkts_ ||
	     tsvec_[seqno%hsz] - tsvec_[maxseqList%hsz] > rtt_)) {
		// numPktsSoFar_ >= numPkts_:
		// Number of pkts since we last entered this procedure
		//   at least equal numPkts_, the number of non-sequential 
		//   packets that must be seen before inferring loss.
		// maxseqList: max seq number checked for dropped packets
		// Decide which losses begin new loss events.
		int i = maxseqList ;
		while(i < seqno) {
			if (lossvec_[i%hsz] == UNKNOWN) {
				rtvec_[i%hsz]=now;	
				tsvec_[i%hsz]=estimate_tstamp(oldmaxseq, seqno, i);	
				if (new_loss(i, tsvec_[i%hsz])) {
					if (algo!=WALI2) {
						congestionEvent = 1;
						lossvec_[i%hsz] = LOST;
					} else
						lossvec_[i%hsz] = NOT_RCVD;
				} else {
					// This lost packet is marked "NOT_RCVD"
					// as it does not begin a loss event.
					printf("%f std_ - %i is not rcvd\n", Scheduler::instance().clock(), i);
					lossvec_[i%hsz] = NOT_RCVD; 
				}
				if (algo == WALI || algo == WALI2) {
			    		++ losses[0];
				}
				losses_since_last_report++;
			}
			i++;
		}
		maxseqList = seqno;
		numPktsSoFar_ = 0;
	} else if (seqno == maxseqList + 1) {
		maxseqList = seqno;
		numPktsSoFar_ = 0;
	} 
	if (seqno > maxseq) {
		maxseq = tfrch->seqno ;
		// if we are in slow start (i.e. (loss_seen_yet ==0)), 
		// and if we saw a loss, report it immediately
		if ((algo == WALI || algo == WALI2) && (loss_seen_yet ==0) &&
		  (tfrch->seqno - oldmaxseq > 1 || ecnEvent )) {
			UrgentFlag = 1 ; 
			loss_seen_yet = 1;
			if (adjust_history_after_ss) {
				p = adjust_history(tfrch->timestamp);
			}

		}
		if ((rtt_ > SMALLFLOAT) && 
			(now - last_report_sent >= rtt_/NumFeedback_)) {
			if (algo!=WALI2) UrgentFlag = 1 ;
		}
	}
	if (UrgentFlag || ecnEvent || congestionEvent) {
		printf("%f - urg = %i ecn = %i cong = %i\n", now, UrgentFlag, ecnEvent, congestionEvent);
		nextpkt(p);
	}

	if (total_received_>0) {
		printf("totaldropped %i and total received %i  | current seqno. %i\n", total_dropped_, total_received_, seqno);
		percentageDropped=100.0*total_dropped_/(total_dropped_+total_received_);
	}

	Packet::free(pkt);
}

double Tfrc_CR_SinkAgent::est_loss ()
{	
	double p = 0 ;
	switch (algo) {
		case WALI:
			p = est_loss_WALI () ;
			break;
		case EWMA:
			p = est_loss_EWMA () ;
			break;
		case RBPH:
			p = est_loss_RBPH () ;
			break;
		case EBPH:
			p = est_loss_EBPH () ;
			break;
		case WALI2:
			p = est_loss_WALI2 () ;
			break;
		default:
			printf ("invalid algo specified\n");
			abort();
			break ; 
	}
	return p;
}

/*
 * compute estimated throughput in packets per RTT for report.
 */
double Tfrc_CR_SinkAgent::est_thput ()
{
	double time_for_rcv_rate;
	double now = Scheduler::instance().clock();
	double thput = 1 ;

	if ((rtt_ > 0) && ((now - last_report_sent) >= rtt_)) {
		// more than an RTT since the last report
		time_for_rcv_rate = (now - last_report_sent);
		if (rcvd_since_last_report > 0) {
			thput = rcvd_since_last_report/time_for_rcv_rate;
			printf("%f - est_throughput %f\n", now, thput);
		}
	}
	else {
		// count number of packets received in the last RTT
		if (rtt_ > 0){
			double last = rtvec_[maxseq%hsz]; 
			int rcvd = 0;
			int i = maxseq;
			while (i > 0) {
				if (lossvec_[i%hsz] == RCVD) {
					if ((rtvec_[i%hsz] + rtt_) > last) 
						rcvd++; 
					else
						break ;
				}
				i--; 
			}
			if (rcvd > 0)
				thput = rcvd/rtt_; 
		}
	}
	return thput ;
}

/*
 * Schedule sending this report, and set timer for the next one.
 */
void Tfrc_CR_SinkAgent::nextpkt(double p) {

	sendpkt(p);

	/* schedule next report rtt/NumFeedback_ later */
	/* note from Sally: why is this 1.5 instead of 1.0? */
	if (rtt_ > 0.0 && NumFeedback_ > 0) {
		if (algo==WALI2) nack_timer_.resched(2*1.5*rtt_/NumFeedback_);
		else nack_timer_.resched(1.5*rtt_/NumFeedback_);
	}
	else {
		nack_timer_.force_cancel();
	}
}

/*
 * Create report message, and send it.
 */
void Tfrc_CR_SinkAgent::sendpkt(double p)
{
	double now = Scheduler::instance().clock();

	printf("%f - std_ send packet and rtt_ %f p = %f\n", now, rtt_, p);

	/*don't send an ACK unless we've received new data*/
	/*if we're sending slower than one packet per RTT, don't need*/
	/*multiple responses per data packet.*/
        /*
	 * Do we want to send a report even if we have not received
	 * any new data?
         */ 

	if (last_arrival_ >= last_report_sent) {

		Packet* pkt = allocpkt();
		if (pkt == NULL) {
			printf ("error allocating packet\n");
			abort(); 
		}
	
		hdr_tfrc_cr_ack *tfrc_ackh = hdr_tfrc_cr_ack::access(pkt);
	
		tfrc_ackh->seqno=maxseq;
		tfrc_ackh->timestamp_echo=last_timestamp_;
		tfrc_ackh->timestamp_offset=now-last_arrival_;
		tfrc_ackh->timestamp=now;
		tfrc_ackh->NumFeedback_ = NumFeedback_;
		if (is_slow_start) {
			tfrc_ackh->slow_start_acked = 1;
			is_slow_start = 0;
		} else {
			tfrc_ackh->slow_start_acked = 0;
		}
		if (p < 0) 
			tfrc_ackh->flost = est_loss (); //if this event is triggered by expiry, this gets set. (or even in recv in some cases)
		else
			tfrc_ackh->flost = p; //no expiry. this is sent at (recv).
		//tfrc_ackh->flost = 0.0002;
		tfrc_ackh->rate_since_last_report = est_thput ();
		tfrc_ackh->losses = losses_since_last_report;
		//variable to expose flost to TCL
		flost_ = tfrc_ackh->flost;
		if (total_received_ <= 0) 
			tfrc_ackh->true_loss = 0.0;
		else 
			tfrc_ackh->true_loss = 1.0 * 
			    total_losses_/(total_received_+total_dropped_);
		//fprintf (pFile_, "%5.2f frequency_of_loss_indication: %f rate_since_last_report: %f NumFeedback: %f losses: %i\n", now, tfrc_ackh->flost, tfrc_ackh->rate_since_last_report, tfrc_ackh->NumFeedback_, losses_since_last_report);
		last_report_sent = now; 
		rcvd_since_last_report = 0;
		losses_since_last_report = 0;
		send(pkt, 0);
	}
}

int Tfrc_CR_SinkAgent::command(int argc, const char*const* argv)
{
	if (argc == 3) {
		if (strcmp(argv[1], "weights") == 0) {
			/* 
			 * weights is a string of numbers, seperated by + signs
			 * the firs number is the total number of weights.
			 * the rest of them are the actual weights
			 * this overrides the defaults
			 */
			char *w ;
			w = (char *)calloc(strlen(argv[2])+1, sizeof(char)) ;
			if (w == NULL) {
				printf ("error allocating w\n");
				abort();
			}
			strcpy(w, (char *)argv[2]);
			numsamples = atoi(strtok(w,"+"));
			sample = (int *)malloc((numsamples+1)*sizeof(int));
			losses = (int *)malloc((numsamples+1)*sizeof(int));
                        count_losses = (int *)malloc((numsamples+1)*sizeof(int));
                        num_rtts = (int *)malloc((numsamples+1)*sizeof(int));
			weights = (double *)malloc((numsamples+1)*sizeof(double));
			mult = (double *)malloc((numsamples+1)*sizeof(double));
			fflush(stdout);
			if (sample && weights) {
				int count = 0 ;
				while (count < numsamples) {
					sample[count] = 0;
					losses[count] = 1;
					count_losses[count] = 0;
                                        num_rtts[count] = 0;
					mult[count] = 1;
					char *w;
					w = strtok(NULL, "+");
					if (w == NULL)
						break ; 
					else {
						weights[count++] = atof(w);
					}	
				}
				if (count < numsamples) {
					printf ("error in weights string %s\n", argv[2]);
					abort();
				}
				sample[count] = 0;
				losses[count] = 1;
				count_losses[count] = 0;
				num_rtts[count] = 0;
				weights[count] = 0;
				mult[count] = 1;
				free(w);
				return (TCL_OK);
			}
			else {
				printf ("error allocating memory for smaple and weights:2\n");
				abort();
			}
		}
	}
	return (Agent::command(argc, argv));
}

void Tfrc_CR_NackTimer::expire(Event *) {
	a_->nextpkt(-1);
}

void Tfrc_CR_SinkAgent::print_loss(int sample, double ave_interval)
{
	double now = Scheduler::instance().clock();
	double drops = 1/ave_interval;
	// This is ugly to include this twice, but the first one is
	//   for backward compatibility with earlier scripts. 
	printf ("time: %7.5f loss_rate: %7.5f \n", now, drops);
	printf ("time: %7.5f sample 0: %5d loss_rate: %7.5f \n", 
		now, sample, drops);
	//printf ("time: %7.5f send_rate: %7.5f\n", now, sendrate);
	//printf ("time: %7.5f maxseq: %d\n", now, maxseq);
}

void Tfrc_CR_SinkAgent::print_loss_all(int *sample)
{
	double now = Scheduler::instance().clock();
	printf ("%f: sample 0: %5d 1: %5d 2: %5d 3: %5d 4: %5d\n", 
		now, sample[0], sample[1], sample[2], sample[3], sample[4]); 
}

void Tfrc_CR_SinkAgent::print_losses_all(int *losses)
{
	double now = Scheduler::instance().clock();
	printf ("%f: losses 0: %5d 1: %5d 2: %5d 3: %5d 4: %5d\n", 
		now, losses[0], losses[1], losses[2], losses[3], losses[4]); 
}

void Tfrc_CR_SinkAgent::print_count_losses_all(int *count_losses)
{
	double now = Scheduler::instance().clock();
	printf ("%f: count? 0: %5d 1: %5d 2: %5d 3: %5d 4: %5d\n", 
		now, count_losses[0], count_losses[1], count_losses[2], count_losses[3], count_losses[4]); 
}

void Tfrc_CR_SinkAgent::print_num_rtts_all(int *count_losses)
{
	double now = Scheduler::instance().clock();
	printf ("%f: rtts 0: %5d 1: %5d 2: %5d 3: %5d 4: %5d\n", 
 	   now, num_rtts[0], num_rtts[1], num_rtts[2], num_rtts[3], num_rtts[4]); 
}

void Tfrc_CR_SinkAgent::calculateStdDev(void) {
	//we need to get out sample [1];
	sample_index+=1;
	delta_samples=sample[1]-avg_samples;
	avg_samples=avg_samples+(delta_samples/sample_index);
	m2_samples=m2_samples+(delta_samples*(sample[1]-avg_samples));
	samplesStdDev_ = sqrt(m2_samples/sample_index);
	mostRecentSample_=sample[1];
	printf("std_ %f this sample %i - samples standard deviation: %f\n", Scheduler::instance().clock(), sample[0], sqrt(m2_samples/sample_index));
}

////////////////////////////////////////
// algo specific code /////////////////
///////////////////////////////////////


/*
 * this gets set at sendpkt() only if we're sending every 1 rtt. then set new interval to start every 2*rtt
 */
double Tfrc_CR_SinkAgent::est_loss_WALI2 ()
{

	int i;
	double ave_interval1, ave_interval2;
	int ds ;
	double now = Scheduler::instance().clock();
	printf("%f - last report sent = %f\n", now, last_report_sent);
	//double last_interval = last_report_sent;
	//int setonce = 0;
	if (!init_WALI_flag) {
		init_WALI () ;
	}
	// sample[i] counts the number of packets in the i-th loss interval
	// sample[0] contains the most recent sample.
        // losses[i] contains the number of losses in the i-th loss interval
        // count_losses[i] is 1 if the i-th loss interval is short.
        // num_rtts[i] contains the number of rtts in the i-th loss interval
	int num_consecutive_losses=0;
	for (i = last_sample; i <= maxseq ; i ++) {
		sample[0]++;
		printf("%f - wali2 last sample = %i rtvec_[%i]=%f lastinterval=%f rtt=%f sample[0]=%i\n", now, last_sample, i, rtvec_[i], lastinterval, rtt_, sample[0]);
		double time = 3;
		//if (percentageDropped>5) TODO: try to use dropped percentage
		//include droppedPercentage here !
		//1.5s works good with BWINC and NONE
		//1.0 works quite nicely now.
		if (rtvec_[i]-lastinterval>=1.0) {//6*rtt_) {
		        //  new loss event
			printf("%f - wali2 new loss event\n", now);
			//if (!setonce) {
				lastinterval=rtvec_[i];
				//setonce = 1;
			//}
			num_consecutive_losses++;
			sample[0]=sample[0]*bin_multiplier_;
			sample_count ++;
			shift_array (sample, numsamples+1, 0);
			shift_array (losses, numsamples+1, 1);
			shift_array (count_losses, numsamples+1, 1);
			shift_array (num_rtts, numsamples+1, 0);
			multiply_array(mult, numsamples+1, mult_factor_);
			shift_array (mult, numsamples+1, 1.0);
			mult_factor_ = 1.0;
			calculateStdDev();
		}
		mostRecentSample_=sample[0];
	}
	if (num_consecutive_losses!=0)
	printf("%f - std_ number of consecutive losses: %i\n", Scheduler::instance().clock(), num_consecutive_losses);
	last_sample = maxseq+1 ;
        //if (ShortIntervals_ > 0 && printLoss_ > 0) {
        //    printf ("now: %5.2f lastloss: %5.2f ShortRtts_: %d rtt_: %5.2f\n",
        //         now, lastloss, ShortRtts_, rtt_);
        //}
        if (ShortIntervals_ > 0 &&
            now - lastloss > ShortRtts_ * rtt_) {
              // Check if the current loss interval is short.
              count_losses[0] = 0;
        }
        if (ShortIntervals_ > 0 && rtt_ > 0) {
              // Count number of rtts in current loss interval.
              num_rtts[0] = (int) ceil((now - lastloss) / rtt_);
              if (num_rtts[0] < 1) num_rtts[0] = 1;
        }
	if (sample_count>numsamples+1)
		// The array of loss intervals is full.
		ds=numsamples+1;
    	else
		ds=sample_count;

	if (sample_count == 1 && false_sample == 0)
		// no losses yet
		return 0;
	/* do we need to discount weights? */
	if (sample_count > 1 && discount && sample[0] > 0) {
                //double ave = weighted_average1(1, ds, 1.0, mult, weights, sample, ShortIntervals_, losses, count_losses);
                double ave = weighted_average1(1, ds, 1.0, mult, weights, sample, ShortIntervals_, losses, count_losses, num_rtts);
		int factor = 2;
		double ratio = (factor*ave)/sample[0];
		double min_ratio = 0.5;
		if ( ratio < 1.0) {
			// the most recent loss interval is very large
			mult_factor_ = ratio;
			if (mult_factor_ < min_ratio)
				mult_factor_ = min_ratio;
		}
		/*double ave_first_two = weighted_average1(1, 2, 1.0, mult, weights, sample, ShortIntervals_, losses, count_losses, num_rtts);
		double ave_last_few = weighted_average1(3, ds, 1.0, mult, weights, sample, ShortIntervals_, losses, count_losses, num_rtts);
		ratio = ave_last_few/(1.5*ave_first_two);
		//if the two latest are very small.
		if (ratio > 1.0) {
			//this will discount the weights of the previous guys
			mult_factor_=1/ratio;
			printf("%f multiplying !!!\n", now);
		}*/

	}
	// Calculations including the most recent loss interval.
        ave_interval1 = weighted_average1(0, ds, mult_factor_, mult, weights, sample, ShortIntervals_, losses, count_losses, num_rtts);
        // Calculations not including the most recent loss interval.
        ave_interval2 = weighted_average1(1, ds, mult_factor_, mult, weights, sample, ShortIntervals_, losses, count_losses, num_rtts);
	// The most recent loss interval does not end in a loss
	// event.  Include the most recent interval in the
	// calculations only if this increases the estimated loss
	// interval.
        // If ShortIntervals is less than 10, do not count the most
        //   recent interval if it is a short interval.
        //   Values of ShortIntervals greater than 10 are only for
        //   validation purposes, and for backwards compatibility.
        //
	if (ave_interval2 > ave_interval1 ||
             (ShortIntervals_ > 1 && ShortIntervals_ < 10
                     && count_losses[0] == 1))
                // The second condition is to check if the first interval
                //  is a short interval.  If so, we must use ave_interval2.
		ave_interval1 = ave_interval2;
	if (ave_interval1 > 0) {
		if (printLoss_ > 0) {
			print_loss(sample[0], ave_interval1);
			print_loss_all(sample);
			if (ShortIntervals_ > 0) {
				print_losses_all(losses);
				print_count_losses_all(count_losses);
                                print_num_rtts_all(num_rtts);
			}
		}
		return 1/ave_interval1;
	} else return 999;
}



////
/// WALI Code
////
double Tfrc_CR_SinkAgent::est_loss_WALI ()
{

	int i;
	double ave_interval1, ave_interval2; 
	int ds ; 

	printf("%f - std_ calculating wali\n", Scheduler::instance().clock());

	if (!init_WALI_flag) {
		init_WALI () ;
	}
	// sample[i] counts the number of packets in the i-th loss interval
	// sample[0] contains the most recent sample.
        // losses[i] contains the number of losses in the i-th loss interval
        // count_losses[i] is 1 if the i-th loss interval is short.
        // num_rtts[i] contains the number of rtts in the i-th loss interval
	int num_consecutive_losses=0;
	for (i = last_sample; i <= maxseq ; i ++) {
		sample[0]++;
		if (lossvec_[i%hsz] == LOST || lossvec_[i%hsz] == ECNLOST) {
		        //  new loss event
			num_consecutive_losses++;//it'll always be 1 ? maybe bigger if we're calculating WALI loss (sending feedback) every more than one rtt while samples are advancing every rtt
			sample_count ++;
			shift_array (sample, numsamples+1, 0); 
			shift_array (losses, numsamples+1, 1); 
			shift_array (count_losses, numsamples+1, 1); 
			shift_array (num_rtts, numsamples+1, 0); 
			multiply_array(mult, numsamples+1, mult_factor_);
			shift_array (mult, numsamples+1, 1.0); 
			mult_factor_ = 1.0;
			//after shifting, we take sample[1]
			calculateStdDev();
		}
		mostRecentSample_=sample[0];
	}
	if (num_consecutive_losses!=0)
	printf("%f - std_ number of consecutive losses: %i\n", Scheduler::instance().clock(), num_consecutive_losses);
	last_sample = maxseq+1 ; 
	double now = Scheduler::instance().clock();
        //if (ShortIntervals_ > 0 && printLoss_ > 0) {
        //    printf ("now: %5.2f lastloss: %5.2f ShortRtts_: %d rtt_: %5.2f\n",
        //         now, lastloss, ShortRtts_, rtt_);
        //}
        if (ShortIntervals_ > 0 && 
            now - lastloss > ShortRtts_ * rtt_) {
              // Check if the current loss interval is short.
              count_losses[0] = 0;
        }
        if (ShortIntervals_ > 0 && rtt_ > 0) {
              // Count number of rtts in current loss interval.
              num_rtts[0] = (int) ceil((now - lastloss) / rtt_);
              if (num_rtts[0] < 1) num_rtts[0] = 1;
        }
	if (sample_count>numsamples+1)
		// The array of loss intervals is full.
		ds=numsamples+1;
    	else
		ds=sample_count;

	if (sample_count == 1 && false_sample == 0) 
		// no losses yet
		return 0; 
	/* do we need to discount weights? */
	if (sample_count > 1 && discount && sample[0] > 0) {
                //double ave = weighted_average1(1, ds, 1.0, mult, weights, sample, ShortIntervals_, losses, count_losses);
                double ave = weighted_average1(1, ds, 1.0, mult, weights, sample, ShortIntervals_, losses, count_losses, num_rtts);
		int factor = 2;
		double ratio = (factor*ave)/sample[0];
		double min_ratio = 0.5;
		if ( ratio < 1.0) {
			// the most recent loss interval is very large
			mult_factor_ = ratio;
			if (mult_factor_ < min_ratio) 
				mult_factor_ = min_ratio;
		}
	}
	// Calculations including the most recent loss interval.
        ave_interval1 = weighted_average1(0, ds, mult_factor_, mult, weights, sample, ShortIntervals_, losses, count_losses, num_rtts);
        // Calculations not including the most recent loss interval.
        ave_interval2 = weighted_average1(1, ds, mult_factor_, mult, weights, sample, ShortIntervals_, losses, count_losses, num_rtts);
	// The most recent loss interval does not end in a loss
	// event.  Include the most recent interval in the 
	// calculations only if this increases the estimated loss
	// interval. 
        // If ShortIntervals is less than 10, do not count the most
        //   recent interval if it is a short interval.
        //   Values of ShortIntervals greater than 10 are only for
        //   validation purposes, and for backwards compatibility.
        //
	if (ave_interval2 > ave_interval1 ||
             (ShortIntervals_ > 1 && ShortIntervals_ < 10 
                     && count_losses[0] == 1))
                // The second condition is to check if the first interval
                //  is a short interval.  If so, we must use ave_interval2.
		ave_interval1 = ave_interval2;
	if (ave_interval1 > 0) { 
		if (printLoss_ > 0) {
			print_loss(sample[0], ave_interval1);
			print_loss_all(sample);
			if (ShortIntervals_ > 0) {
				print_losses_all(losses);
				print_count_losses_all(count_losses);
                                print_num_rtts_all(num_rtts);
			}
		}
		return 1/ave_interval1; 
	} else return 999;     
}

// Calculate the weighted average.
double Tfrc_CR_SinkAgent::weighted_average(int start, int end, double factor, double *m, double *w, int *sample)
{
	int i; 
	double wsum = 0;
	double answer = 0;
	if (smooth_ == 1 && start == 0) {
		if (end == numsamples+1) {
			// the array is full, but we don't want to uses
			//  the last loss interval in the array
			end = end-1;
		} 
		// effectively shift the weight arrays 
		for (i = start ; i < end; i++) 
			if (i==0)
				wsum += m[i]*w[i+1];
			else 
				wsum += factor*m[i]*w[i+1];
		for (i = start ; i < end; i++)  
			if (i==0)
			 	answer += m[i]*w[i+1]*sample[i]/wsum;
			else 
				answer += factor*m[i]*w[i+1]*sample[i]/wsum;
	        return answer;

	} else {
		for (i = start ; i < end; i++) 
			if (i==0)
				wsum += m[i]*w[i];
			else 
				wsum += factor*m[i]*w[i];
		for (i = start ; i < end; i++)  
			if (i==0)
			 	answer += m[i]*w[i]*sample[i]/wsum;
			else 
				answer += factor*m[i]*w[i]*sample[i]/wsum;
	        return answer;
	}
}

int Tfrc_CR_SinkAgent::get_sample(int oldSample, int numLosses)
{
	int newSample;
	if (numLosses == 0) {
		newSample = oldSample;
	} else {
		newSample = oldSample / numLosses;
	}
	return newSample;
}

int Tfrc_CR_SinkAgent::get_sample_rtts(int oldSample, int numLosses, int rtts)
{
	int newSample;
	if (numLosses == 0) {
		newSample = oldSample;
                //printf ("sample: %d numLosses: %d\n", oldSample, numLosses);
	} else {
                double fraction;
                if (ShortRtts_ != 0)
                     fraction = (ShortRtts_ + 1.0 - rtts) / ShortRtts_;
                else fraction = 1.0;
		int numLoss = (int) (floor(fraction * numLosses ));
                if (numLoss != 0)
		  newSample = oldSample / numLoss;
                else newSample = oldSample;
                //printf ("sample: %d rtts: %d numLosses: %d newSample: %d fraction: %5.2f numLoss %d\n",
                //  oldSample, rtts, numLosses, newSample, fraction, numLoss);
	}
	return newSample;
}

// Calculate the weighted average, factor*m[i]*w[i]*sample[i]/wsum.
// "factor" is "mult_factor_", for weighting the most recent interval
//    when it is very large
// "m[i]" is "mult[]", for old values of "mult_factor_".
//
// When ShortIntervals_%10 is 1, the length of a loss interval is
//   "sample[i]/losses[i]" for short intervals, not just "sample[i]".
//   This is equivalent to a loss event rate of "losses[i]/sample[i]",
//   instead of "1/sample[i]".
//
// When ShortIntervals_%10 is 2, it is like ShortIntervals_ of 1,
//   except that the number of losses per loss interval is at
//   most 1460/byte-size-of-small-packets.
//
// When ShortIntervals_%10 is 3, short intervals are up to three RTTs,
//   and the number of losses counted is a function of the interval size.
//
double Tfrc_CR_SinkAgent::weighted_average1(int start, int end, double factor, double *m, double *w, int *sample, int ShortIntervals, int *losses, int *count_losses, int *num_rtts)
{
        int i;
        int ThisSample;
        double wsum = 0;
        double answer = 0;
        if (smooth_ == 1 && start == 0) {
                if (end == numsamples+1) {
                        // the array is full, but we don't want to use
                        //  the last loss interval in the array
                        end = end-1;
                }
                // effectively shift the weight arrays
                for (i = start ; i < end; i++)
                        if (i==0)
                                wsum += m[i]*w[i+1];
                        else
                                wsum += factor*m[i]*w[i+1];
                for (i = start ; i < end; i++) {
                        ThisSample = sample[i];
                        if (ShortIntervals%10 == 1 && count_losses[i] == 1) {
			       ThisSample = get_sample(sample[i], losses[i]);
                        }
                        if (ShortIntervals%10 == 2 && count_losses[i] == 1) {
			       int adjusted_losses = int(fsize_/size_);
			       if (losses[i] < adjusted_losses) {
					adjusted_losses = losses[i];
			       }
			       ThisSample = get_sample(sample[i], adjusted_losses);
                        }
                        if (ShortIntervals%10 == 3 && count_losses[i] == 1) {
			       ThisSample = get_sample_rtts(sample[i], losses[i], num_rtts[i]);
                        }
                        if (i==0)
                                answer += m[i]*w[i+1]*ThisSample/wsum;
                                //answer += m[i]*w[i+1]*sample[i]/wsum;
                        else
                                answer += factor*m[i]*w[i+1]*ThisSample/wsum;
                                //answer += factor*m[i]*w[i+1]*sample[i]/wsum;
		}
                return answer;

        } else {
                for (i = start ; i < end; i++)
                        if (i==0)
                                wsum += m[i]*w[i];
                        else
                                wsum += factor*m[i]*w[i];
                for (i = start ; i < end; i++) {
                       ThisSample = sample[i];
                       if (ShortIntervals%10 == 1 && count_losses[i] == 1) {
			       ThisSample = get_sample(sample[i], losses[i]);
                       }
                       if (ShortIntervals%10 == 2 && count_losses[i] == 1) {
			       ThisSample = get_sample(sample[i], 7);
			       // Replace 7 by 1460/packet size.
                               // NOT FINISHED.
                       }
                        if (ShortIntervals%10 == 3 && count_losses[i] == 1) {
			       ThisSample = get_sample_rtts(sample[i], losses[i], (int) num_rtts[i]);
                        }
                       if (i==0)
                                answer += m[i]*w[i]*ThisSample/wsum;
                                //answer += m[i]*w[i]*sample[i]/wsum;
                        else
                                answer += factor*m[i]*w[i]*ThisSample/wsum;
                                //answer += factor*m[i]*w[i]*sample[i]/wsum;
		}
                return answer;
        }
}

// Shift array a[] up, starting with a[sz-2] -> a[sz-1].
void Tfrc_CR_SinkAgent::shift_array(int *a, int sz, int defval)
{
	int i ;
	for (i = sz-2 ; i >= 0 ; i--) {
		a[i+1] = a[i] ;
	}
	a[0] = defval;
}
void Tfrc_CR_SinkAgent::shift_array(double *a, int sz, double defval) {
	int i ;
	for (i = sz-2 ; i >= 0 ; i--) {
		a[i+1] = a[i] ;
	}
	a[0] = defval;
}

// Multiply array by value, starting with array index 1.
// Array index 0 of the unshifted array contains the most recent interval.
void Tfrc_CR_SinkAgent::multiply_array(double *a, int sz, double multiplier) {
	int i ;
	for (i = 1; i <= sz-1; i++) {
		double old = a[i];
		a[i] = old * multiplier ;
	}
}

/*
 * We just received our first loss, and need to adjust our history.
 */
double Tfrc_CR_SinkAgent::adjust_history (double ts)
{
	int i;
	double p;
	for (i = maxseq; i >= 0 ; i --) {
		if (lossvec_[i%hsz] == LOST || lossvec_[i%hsz] == ECNLOST ) {
			lossvec_[i%hsz] = NOT_RCVD; 
		}
	}
	lastloss = ts; 
	lastloss_round_id = round_id ;
	p=b_to_p3(est_thput()*psize_, rtt_, tzero_, fsize_, 1);
	false_sample = (int)(1.0/p);
	sample[1] = false_sample;
	sample[0] = 0;
	losses[1] = 0;
	losses[0] = 1;
	count_losses[1] = 0;
	count_losses[0] = 0;
        num_rtts[0]=0;
        num_rtts[1]=0;
	sample_count++; 
	if (printLoss_) {
		print_loss_all (sample);
		if (ShortIntervals_ > 0) {
			print_losses_all(losses);
			print_count_losses_all(count_losses);
			print_num_rtts_all(num_rtts);
		}
	}
	false_sample = -1 ; 
	return p;
}


/*
 * Initialize data structures for weights.
 */
void Tfrc_CR_SinkAgent::init_WALI () {
	int i;
	if (numsamples < 0)
		numsamples = DEFAULT_NUMSAMPLES ;	
	if (smooth_ == 1) {
		numsamples = numsamples + 1;
	}
	sample = (int *)malloc((numsamples+1)*sizeof(int));
        losses = (int *)malloc((numsamples+1)*sizeof(int));
        count_losses = (int *)malloc((numsamples+1)*sizeof(int));
        num_rtts = (int *)malloc((numsamples+1)*sizeof(int));
	weights = (double *)malloc((numsamples+1)*sizeof(double));
	mult = (double *)malloc((numsamples+1)*sizeof(double));
	for (i = 0 ; i < numsamples+1 ; i ++) {
		sample[i] = 0 ; 
	}
	if (smooth_ == 1) {
		int mid = int(numsamples/2);
		for (i = 0; i < mid; i ++) {
			weights[i] = 1.0;
		}
		for (i = mid; i <= numsamples; i ++){
			weights[i] = 1.0 - (i-mid)/(mid + 1.0);
		}
	} else {
		int mid = int(numsamples/2);
		for (i = 0; i < mid; i ++) {
			weights[i] = 1.0;
		}
		for (i = mid; i <= numsamples; i ++){
			weights[i] = 1.0 - (i+1-mid)/(mid + 1.0);
		}
	}
	for (i = 0; i < numsamples+1; i ++) {
		mult[i] = 1.0 ; 
	}
	init_WALI_flag = 1;  /* initialization done */
}

///////////////////////////
// EWMA //////////////////
//////////////////////////

double Tfrc_CR_SinkAgent::est_loss_EWMA () {
	double p1, p2 ;
	for (int i = last_sample; i <= maxseq ; i ++) {
		loss_int++; 
		if (lossvec_[i%hsz] == LOST || lossvec_[i%hsz] == ECNLOST ) {
			if (avg_loss_int < 0) {
				avg_loss_int = loss_int ; 
			} else {
				avg_loss_int = history*avg_loss_int + (1-history)*loss_int ;
			}
			loss_int = 0 ;
		}
	}
	last_sample = maxseq+1 ; 

	if (avg_loss_int < 0) { 
		p1 = 0;
	} else {
		p1 = 1.0/avg_loss_int ; 
	}
	if (loss_int == 0 
	    || avg_loss_int < 0){ //XXX this last check was added by a
				  //person who knows nothing of this
				  //code just to stop FP div by zero.
				  //Values were history=.75,
				  //avg_loss_int=-1, loss_int=3.  If
				  //you know what should be here,
				  //please cleanup and remove this
				  //comment.

		p2 = p1 ; 
	} else {
		p2 = 1.0/(history*avg_loss_int + (1-history)*loss_int) ;
	}
	if (p2 < p1) {
		p1 = p2 ; 
	}
	if (printLoss_ > 0) {
		if (p1 > 0) 
			print_loss(loss_int, 1.0/p1);
		else
			print_loss(loss_int, 0.00001);
		print_loss_all(sample);
	}
	return p1 ;
}

///////////////////////////
// RBPH //////////////////
//////////////////////////
double Tfrc_CR_SinkAgent::est_loss_RBPH () {

	double numpkts = hsz ;
	double p ; 

	// how many pkts we should go back?
	if (sendrate > 0 && rtt_ > 0) {
		double x = b_to_p3(sendrate, rtt_, tzero_, psize_, 1);
		if (x > 0) 
			numpkts = minlc/x ; 
		else
			numpkts = hsz ;
	}

	// that number must be below maxseq and hsz 
	if (numpkts > maxseq)
		numpkts = maxseq ;
	if (numpkts > hsz)
		numpkts = hsz ;

	int lc = 0;
	int pc = 0;
	int i = maxseq ;

	// first see if how many lc's we find in numpkts 
	while (pc < numpkts) {
		pc ++ ;
		if (lossvec_[i%hsz] == LOST || lossvec_[i%hsz] == ECNLOST )
			lc ++ ; 
		i -- ;
	}

	// if not enough lsos events, keep going back ...
	if (lc < minlc) {

		// but only as far as the history allows ...
		numpkts = maxseq ;
		if (numpkts > hsz)
			numpkts = hsz ;

		while ((lc < minlc) && (pc < numpkts)) {
			pc ++ ;
			if (lossvec_[i%hsz] == LOST || lossvec_[i%hsz] == ECNLOST )
				lc ++ ;
			i -- ;
		
		}
	}

	if (pc == 0) 
		p = 0; 
	else
		p = (double)lc/(double)pc ; 
	if (printLoss_ > 0) {
		if (p > 0) 
			print_loss(0, 1.0/p);
		else
			print_loss(0, 0.00001);
		print_loss_all(sample);
	}
	return p ;
}

///////////////////////////
// EBPH //////////////////
//////////////////////////
double Tfrc_CR_SinkAgent::est_loss_EBPH () {

	double numpkts = hsz ;
	double p ; 

	int lc = 0;
	int pc = 0;
	int i = maxseq ;

	numpkts = maxseq ;
	if (numpkts > hsz)
		numpkts = hsz ;

	while ((lc < minlc) && (pc < numpkts)) {
		pc ++ ;
		if (lossvec_[i%hsz] == LOST || lossvec_[i%hsz] == ECNLOST)
			lc ++ ;
		i -- ;
	}

	if (pc == 0) 
		p = 0; 
	else
		p = (double)lc/(double)pc ; 
	if (printLoss_ > 0) {
		if (p > 0) 
			print_loss(0, 1.0/p);
		else
			print_loss(0, 0.00001);
		print_loss_all(sample);
	}
	return p ;
}
