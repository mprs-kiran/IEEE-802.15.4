/*
This simulation code models the behaviour of IEEE 802.15.4 Slotted CSMA/CA with limited on-node storage and connected in star topology.

We assume ideal channel conditions and only the CSMA/CA is modeled without any PHY layer.

All the delays are measured in slots (1 slot = 320 us)

It enables easy analysis.

Any bugs, please report them to ee12m1021@iith.ac.in. Please read the comments to understand the code.

For more details on the explanation of the code, please refer to the paper: https://ieeexplore.ieee.org/document/8085176

Please cite this paper for usage of the code: https://ieeexplore.ieee.org/document/8085176
*/

#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define t 10000000 // Simulation duration in backoff slots
#define node_num 10 // No. of nodes in the network
#define pkt 6 // Packet size in slots
#define ack 1 // Ack duration in slots
#define qLim 5 // The queue size of each node in no. of packets

int h = 1, macMinBE = 2, macMaxBE = 8, m_b = 5, n_b = 2; // macMinBE - Min. Backoff Exponent, macMaxBE - Max. Backoff Exponent, m_b - Max. Backoff Stages, n_b - Maximum no. of retransmissions allowed

int qStateInd[node_num], csmaStateInd[node_num]; // Used for indicating whether the node is in idle state or csma state
int wakeCount[node_num], sleepDur[node_num];
int qSlot[node_num], qEmpty[node_num], qEmptySucc[node_num];
int w[node_num], m[node_num], n[node_num];
int inB000[node_num], channel[node_num][pkt + ack + 2];
int in_alpha[node_num], in_beta[node_num], fail_alpha[node_num],
		fail_beta[node_num], succ_beta[node_num];
int out_fail_csma_backoff[node_num], out_fail_csma_retry[node_num],
		out_succ[node_num], in[node_num];
int delayCsma[node_num], delay[node_num],ccaDelay[node_num],ccaDelayAcc[node_num], delayBE[node_num], delayQueueCsma[node_num],
		delayRetry[node_num], delayBack[node_num], delayAvg[node_num];
int delCount[node_num];
int queue[node_num], queueFail[node_num], totalArr[node_num];
int qFill[node_num], qFillCount[node_num], qFillCountRetry[node_num], qFillCountBack[node_num], qFillCountSucc[node_num];
int qReq = 0;
int varMax = 1, iterMax = 1; // Set varMax=1 always. iterMax - No. of iterations to conduct for averaging
int queueValidate[node_num];
int tEvent[node_num];
float lambda = 6; // Packet arrival rate per second - follows poisson distribution 
int sfTimer = 0, sfDur = 48;
float actPeriod = 0.256; // Active period of the node in a beacon interval
int actMode = 1;
int sDel[node_num], serveCount[node_num], avgIS[node_num], iS[node_num],
		avgQueue[node_num], qSat[node_num];
float Psleep  = 0.00026, Pidle=0.16, Prx=0.17, Ptx=0.16; // Power consumption in different states of the node
int dSleep[node_num], dBackoff[node_num], dCCA[node_num], dTx[node_num], dRx[node_num];
int queueDelayAcc[node_num][qLim];
FILE *fp_r, *fp_al, *fp_be, *fp_booo, *fp_T, *fp_delay, *fp_queue, *fp_energy; // The results are logged into seperate files

void initIter() {
	int i = 0, j = 0;
	srand(time(0));
	for (i = 0; i < node_num; i++) {
		dSleep[i]=0;
		dBackoff[i]=0;
		dCCA[i]=0;
		dTx[i]=0;
		dRx[i]=0;
		qFill[i] = 0;
		qFillCount[i] = 0;
		qFillCountBack[i] = 0;
		qFillCountSucc[i] = 0;
		qFillCountRetry[i] = 0;
		qEmptySucc[i] = 0;
		delCount[i] = 0;
		delayBE[i] = 0;
		delayRetry[i] = 0;
		delayBack[i] = 0;
		delayAvg[i] = 0;
		sDel[i] = 0;
		sleepDur[i] = 0;
		serveCount[i] = 0;
		avgIS[i] = 0;
		iS[i] = 0;
		queueValidate[i] = 0;
		tEvent[i] = 1;
		queue[i] = 0;
		queueFail[i] = 0;
		qSat[i] = 0;
		totalArr[i] = 0;
		qStateInd[i] = 1;
		qSlot[i] = 0;
		csmaStateInd[i] = 0;
		avgQueue[i] = 0;
		wakeCount[i] = 0;
		m[i] = 0;
		n[i] = 0;
		w[i] = 0;
		in[i] = 0;
		inB000[i] = 0;

		in_alpha[i] = 0;
		in_beta[i] = 0;
		fail_alpha[i] = 0;
		fail_beta[i] = 0;
		succ_beta[i] = 0;
		out_fail_csma_backoff[i] = 0;
		out_fail_csma_retry[i] = 0;
		out_succ[i] = 0;
		delayCsma[i] = 0;
        delayQueueCsma[i] = 0;
		ccaDelayAcc[i] = 0;
		delay[i] = 0;
		ccaDelay[i] = 0;
		for (j = 0; j <= pkt + ack + 1; j++) {
			channel[i][j] = 0;
		}
        for(j-0;j<qLim;j++) {
            queueDelayAcc[i][j] = 0;
        }
	}

}
int csma_sense() {

	int i, j = 0;

	for (i = 0; i < node_num; i++) {
		j = channel[i][0] + j;
		if (j > 0) {
			return (0);
		}
	}
	return (1);
}
float randNum_0_1() {
	float randNum = (float) rand() / (float) RAND_MAX;
	if (randNum == 0) {
		return ((float) 2);
	}
	return randNum;
}

int power(int a, int b) {
	int j;
	int c = 1;
	for (j = 0; j < b; j++)
		c = c * a;

	return (c);
}
int MIN(int a, int b) {
	int c;
	c = (((a) < (b)) ? (a) : (b));
	return (c);
}
int randBackOffSlot(int m) {
	return (power(2, MIN(macMaxBE, macMinBE + m)) * ((double) rand() / (double) ((unsigned) RAND_MAX + 1)));

}
void leafNode(int node) {
    for(int i=0; i<queue[node]; i++) queueDelayAcc[node][i]++;
	if (qStateInd[node] == 1) {
		dSleep[node]++;
		if (qSlot[node] < h) {
			qSlot[node] = qSlot[node] + 1;
			if (qSlot[node] == h) {
				if(qFill[node] == qReq) {
					qFillCount[node] = qFillCount[node]+1;
				}
				sleepDur[node] = sleepDur[node] + 1;
				wakeCount[node] = wakeCount[node] + 1;
				serveCount[node] = serveCount[node] + 1;
				avgIS[node] = avgIS[node] + sDel[node];
				sDel[node] = 0;
				qFill[node] = 0;

				if (queue[node] > 0) {
					csmaStateInd[node] = 1;
					m[node] = 0;
					w[node] = randBackOffSlot(m[node]);
					n[node] = 0;
					qStateInd[node] = 0;
					in[node] = in[node] + 1;
					delay[node] = 0;
					ccaDelay[node] = 0;
					delayBack[node] = 0;
				} else {
					qEmpty[node] = qEmpty[node] + 1;
					qSlot[node] = 0;
				}
			}
		}

	} else if (csmaStateInd[node] == 1) {
		delay[node] = delay[node] + 1;
		// CCA Contention slots
		if ((w[node] == 0) && (m[node] == 0) && (n[node] == 0)) {
			inB000[node] = inB000[node] + 1;
		}
		if ((w[node] == 0) && (m[node] != -1) && (m[node] != -2)) {
			dCCA[node]++;
			delayBack[node] = delayBack[node] + 1;
			in_alpha[node] = in_alpha[node] + 1;
			if (csma_sense() > 0) {
				w[node] = -1;
			} else {
				fail_alpha[node] = fail_alpha[node] + 1;
				if (m[node] < m_b) {
					m[node] = m[node] + 1;
					w[node] = randBackOffSlot(m[node]);

				} else {
					queue[node] = queue[node] - 1;
for(int i=1; i<qLim; i++) queueDelayAcc[node][i-1] = queueDelayAcc[node][i];
                queueDelayAcc[node][qLim-1] = 0;
					wakeCount[node] = wakeCount[node] + 1;
					serveCount[node] = serveCount[node] + 1;
					avgIS[node] = avgIS[node] + sDel[node];
					sDel[node] = 0;
					delayBE[node] = delayBE[node] + delay[node];
					if(qFill[node] == qReq) {
						qFillCountBack[node] = qFillCountBack[node] + 1;
					}
					qFill[node] = 0;
					if (queue[node] > 0) {

						csmaStateInd[node] = 1;
						m[node] = 0;
						w[node] = randBackOffSlot(m[node]);
						n[node] = 0;
						qStateInd[node] = 0;
						in[node] = in[node] + 1;
						delay[node] = 0;
						ccaDelay[node] = 0;
						delayBack[node] = 0;
					} else {
						qFill[node] = 0;
						qEmpty[node] = qEmpty[node] + 1;
						qSlot[node] = 0;
						qStateInd[node] = 1;
						m[node] = 0;
						w[node] = 0;
						n[node] = 0;
						csmaStateInd[node] = 0;
					}
					out_fail_csma_backoff[node] = out_fail_csma_backoff[node]
							+ 1;
				}
			}
		}

		else if ((w[node] == -1) && (m[node] != -1) && (m[node] != -2)) {
			dCCA[node]++;
			delayBack[node] = delayBack[node] + 1;
			ccaDelay[node] = ccaDelay[node] + 1;
			in_beta[node] = in_beta[node] + 1;
			if (csma_sense()) {
				succ_beta[node] = succ_beta[node] + 1;
				channel[node][1] = 1;
			} else {

				fail_beta[node] = fail_beta[node] + 1;
				if (m[node] < m_b) {
					m[node] = m[node] + 1;
					w[node] = randBackOffSlot(m[node]);
				} else {
					queue[node] = queue[node] - 1;
for(int i=1; i<qLim; i++) queueDelayAcc[node][i-1] = queueDelayAcc[node][i];
                queueDelayAcc[node][qLim-1] = 0;
					wakeCount[node] = wakeCount[node] + 1;
					serveCount[node] = serveCount[node] + 1;
					avgIS[node] = avgIS[node] + sDel[node];
					sDel[node] = 0;
					delayBE[node] = delayBE[node] + delay[node];
					if(qFill[node] == qReq) {
						qFillCountBack[node] = qFillCountBack[node] + 1;
					}
					qFill[node] = 0;
					if (queue[node] > 0) {

						csmaStateInd[node] = 1;
						m[node] = 0;
						w[node] = randBackOffSlot(m[node]);
						n[node] = 0;
						qStateInd[node] = 0;
						in[node] = in[node] + 1;
						delay[node] = 0;
						ccaDelay[node] = 0;
						delayBack[node] = 0;

					} else {
						qFill[node] = 0;
						qEmpty[node] = qEmpty[node] + 1;
						qSlot[node] = 0;
						qStateInd[node] = 1;
						csmaStateInd[node] = 0;
						m[node] = 0;
						w[node] = 0;
						n[node] = 0;

					}
					out_fail_csma_backoff[node] = out_fail_csma_backoff[node]
							+ 1;
				}
			}
		} else if ((w[node] > 0) && (m[node] != -1) && (m[node] != -2)) {
			dBackoff[node]++;
			delayBack[node] = delayBack[node] + 1;
			w[node] = w[node] - 1;
		} else if (m[node] == -2) {
			dTx[node]++;
			if (w[node] > 1)
				w[node] = w[node] - 1;
			else {
				if (n[node] < n_b) {
					delayBack[node] = 0;
					m[node] = 0;
					w[node] = randBackOffSlot(m[node]);
					n[node] = n[node] + 1;
				} else {
					dTx[node] = dTx[node]-2;
					dBackoff[node] = dBackoff[node]+3;
					queue[node] = queue[node] - 1;
for(int i=1; i<qLim; i++) queueDelayAcc[node][i-1] = queueDelayAcc[node][i];
                queueDelayAcc[node][qLim-1] = 0;
					wakeCount[node] = wakeCount[node] + 1;
					serveCount[node] = serveCount[node] + 1;
					avgIS[node] = avgIS[node] + sDel[node];
					sDel[node] = 0;
					delayRetry[node] = delayRetry[node] + delay[node];
					if(qFill[node] == qReq) {
											qFillCountRetry[node] = qFillCountRetry[node] + 1;
										}
										qFill[node] = 0;
					if (queue[node] > 0) {

						csmaStateInd[node] = 1;
						m[node] = 0;
						n[node] = 0;
						w[node] = randBackOffSlot(m[node]);
						qStateInd[node] = 0;
						in[node] = in[node] + 1;
						delay[node] = 0;
						ccaDelay[node] = 0;
						delayBack[node] = 0;

					} else {
						qFill[node] = 0;
						qEmpty[node] = qEmpty[node] + 1;
						qSlot[node] = 0;
						qStateInd[node] = 1;
						m[node] = 0;
						w[node] = 0;
						n[node] = 0;
						csmaStateInd[node] = 0;
					}
					out_fail_csma_retry[node] = out_fail_csma_retry[node] + 1;
				}

			}
		} else if (m[node] == -1) {
			dTx[node]++;
			if (w[node] > 1)
				w[node] = w[node] - 1;
			else {
				dTx[node] = dTx[node]-2;
				dBackoff[node]++;
				dRx[node] = dRx[node]+2;
				queue[node] = queue[node] - 1;

				wakeCount[node] = wakeCount[node] + 1;
				delayCsma[node] = delayCsma[node] + delay[node];
                delayQueueCsma[node] = delayQueueCsma[node] + queueDelayAcc[node][0];
                for(int i=1; i<qLim; i++) queueDelayAcc[node][i-1] = queueDelayAcc[node][i];
                queueDelayAcc[node][qLim-1] = 0;
				ccaDelayAcc[node] = ccaDelayAcc[node] + ccaDelay[node];
				if (n[node] == 0) {
					delayAvg[node] = delayAvg[node] + delayBack[node];
					delCount[node] = delCount[node] + 1;
				}
				if(qFill[node] == qReq) {
										qFillCountSucc[node] = qFillCountSucc[node] + 1;
									}
									qFill[node] = 0;
				serveCount[node] = serveCount[node] + 1;
				avgIS[node] = avgIS[node] + sDel[node];
				sDel[node] = 0;
				if (queue[node] > 0) {
					ccaDelay[node] = 0;
					delay[node] = 0;
					delayBack[node] = 0;

					csmaStateInd[node] = 1;
					m[node] = 0;
					n[node] = 0;
					w[node] = randBackOffSlot(m[node]);
					qStateInd[node] = 0;
					in[node] = in[node] + 1;
				} else {
					qFill[node] = 0;
					qEmpty[node] = qEmpty[node] + 1;
					qEmptySucc[node] = qEmptySucc[node]+1;
					qSlot[node] = 0;
					qStateInd[node] = 1;
					m[node] = 0;
					w[node] = 0;
					n[node] = 0;
					csmaStateInd[node] = 0;
				}
				out_succ[node] = out_succ[node] + 1;
			}
		}

	}

}

void sense_collission() {
	int i, j = 0, k, l, n_u;

	for (i = 0; i < node_num; i++) {
		j = channel[i][1] + j;
		if (channel[i][1])
			n_u = i;
	}
	if (j > 1) {
		for (k = 0; k < node_num; k++) {
			if (channel[k][1] && m[k] != -2 && csmaStateInd[k] == 1
					&& m[k] != -1) {
				m[k] = -2;
				w[k] = pkt + ack + 1;
				for (l = 1; l <= pkt; l++)
					channel[k][l] = 1;
				channel[k][pkt + 1] = 0;
				for (l = pkt + 2; l <= pkt + ack + 1; l++)
					channel[k][l] = 0;

			}
		}
	} else if ((j == 1)) {
		if ((w[n_u] == -1) && (csmaStateInd[n_u] == 1) && m[n_u] != -1
				&& m[n_u] != -2) {
			m[n_u] = -1;
			w[n_u] = pkt + ack + 1;
			for (l = 1; l <= pkt; l++)
				channel[n_u][l] = 1;
			channel[n_u][pkt + 1] = 0;
			for (l = pkt + 2; l <= pkt + ack + 1; l++)
				channel[n_u][l] = 1;

		}

	}

}

void shifter() {
	int j, k;

	for (j = 0; j < node_num; j++) {
		for (k = 0; k <= pkt + ack; k++) {
			channel[j][k] = channel[j][k + 1];
		}
		channel[j][pkt + ack + 1] = 0;
	}

}
void logResult() {
	int j;
	int sum = 0;
	int sumIn = 0;
    printf("Logging results");
	for (j = 0; j < node_num; j++) {
		sum = sum + queueValidate[j];
		sumIn = sumIn + in[j];
        
		fprintf(fp_r, "Node: %d Stats - \t Effective Reliability: %f\t CSMA Reliability: %f\t CSMA Failure Due to Exceeded Retries: %f\t CSMA Failures Due to Exceeded Backoffs: %f\n", j+1, (float) out_succ[j] / totalArr[j], (float) out_succ[j] / in[j],(float) out_fail_csma_retry[j] / in[j],(float) out_fail_csma_backoff[j] / in[j]);
		
		fprintf(fp_al, "Node: %d Stats - CCA1 Failure/Busy Probability: %f\n", j+1, (float) fail_alpha[j] / in_alpha[j]);
        fprintf(fp_be, "Node: %d Stats - CCA2 Failure/Busy Probability: %f\n", j+1, (float) fail_beta[j] / in_beta[j]);
	
		fprintf(fp_queue,
				"Node: %d Stats - Probability of queue being empty: %f\t Blocking Probability: %f\t Avg Queue Size: %f\n", j+1, (float)qEmptySucc[j]/out_succ[j],
				(float) queueFail[j] / totalArr[j], (float)avgQueue[j]/t);
		fprintf(fp_booo, "Node: %d Stats - Probability of node residing in state (0,0,0): %f\n", j+1, (float) inB000[j] / (t));
		fprintf(fp_T, "Node: %d Stats - Probability of node attempting CCA1 (i,0,j): %f\n", j+1, (float) in_alpha[j] / (t));
		fprintf(fp_delay, "Node: %d Stats - Delay for successful packet transmission with queue waiting time: %f\t Delay for successful packet transmission: %f\t Delay for packet discarded due to exceeded retries: %f\t Delay for packet discarded due to exceeded backoffs: %f\n", j+1, (float) delayQueueCsma[j] / out_succ[j], (float) delayCsma[j] / out_succ[j],
				(float) delayRetry[j] / out_fail_csma_retry[j],
				(float) delayBE[j] / out_fail_csma_backoff[j]);
		fprintf(fp_energy, "Node: %d Stats - Average Power Consumption (W): %f\n", j+1, (float) (Psleep*dSleep[j] + Pidle*dBackoff[j] + Ptx*dTx[j] + Prx*(dRx[j] + dCCA[j]))/t);

	}
}

void queueUpdater(int time) {
	int i = 0;

	for (i = 0; i < node_num; i++) {
		avgQueue[i] = avgQueue[i] + queue[i];
		if (queue[i] == 0){
			qSat[i] = qSat[i] + 1;
		}
		tEvent[i] = tEvent[i] - 1;
		if (tEvent[i] == 0) {
			qFill[i] = qFill[i]+1;
			totalArr[i] = totalArr[i] + 1;
			if (queue[i] < qLim) {
				queue[i] = queue[i] + 1;
                queueDelayAcc[i][queue[i]-1] = 0;
			} else {
				queueFail[i] = queueFail[i] + 1;
			}
			queueValidate[i] = queueValidate[i] + 1;
			tEvent[i] = (float) (-log(randNum_0_1()) * 3125) / (float) lambda;
			tEvent[i] = (tEvent[i] < 1) ? 1 : tEvent[i];
		}
	}
}
int main() {
	int i, j, iter, var;
    
	// File pointer initializations
	fp_r = fopen("/results/Reliability.txt", "w");
	fp_al = fopen("/results/CCA1 Fail Probabilities.txt", "w");
	fp_be = fopen("/results/CCA2 Fail Probabilities.txt", "w");
	fp_booo = fopen("/results/b000 Probabilities.txt", "w");
	fp_T = fopen("/results/Tau Probabilities.txt", "w");
	fp_delay = fopen("/results/Packet Delay.txt", "w");
	fp_queue = fopen("/results/Queue Statistics.txt", "w");
	fp_energy = fopen("/results/Power Consumption.txt", "w");
	setvbuf(stdout, NULL, _IONBF, 0);

	printf("Starting Simulation...\n");
    printf("Simulation Time: %d\n", t);

	for (var = 0; var < varMax; var++) {
		for (iter = 0; iter < iterMax; iter++) {
			initIter();
			printf("\nProgress Variation: %d, Iteration: %d\n", var, iter);
			for (i = 0; i < t; i++) {
				for (j = 0; j < node_num; j++) {
					sDel[j] = sDel[j] + 1;
				}
				sfTimer = sfTimer + 1;
				queueUpdater(i);
				if (sfTimer == (int)(actPeriod * sfDur)) {
					actMode = 0;
				}
				if (sfTimer == sfDur) {
					actMode = 1;
					sfTimer = 0;
				}
				if (actMode == 1) {
					for (j = 0; j < node_num; j++) {
						leafNode(j);
					}
					sense_collission();
					shifter();
				}
			}
		}
		logResult();
	}
	printf("\n*************--Process Completed--***************\n");
	return (1);
}
