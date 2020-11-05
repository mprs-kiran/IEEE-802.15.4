/*
 * CSMAWithPCASlotted.c
 *
 *  Created on: Jan 10, 2017
 *      Author: Kiran
 */

#include<stdio.h>
#include<stdlib.h>
#include<time.h>
#include<math.h>

#define t 40000000 // Simulation duration in backoff slots
#define node_num 40 //No. of nodes
#define pkt 6 //Packet lenght in backoff slots
#define ack 1 // ACK length in backoff slots
#define Wmax 3 // Maximum backoff window size for PCA - 2^(macMinBE-1)-1
#define debugEn 0 // For debugging of the state model over time. Will generate log file of huge size, hence do not enable until necessary

int h = 1, macMinBE = 3, macMaxBE = 8, maxBackoffStages = 5, n_b = 1, critDelay = 16; //// macMinBE - Min. Backoff Exponent, macMaxBE - Max. Backoff Exponent, maxBackoffStages - Max. Backoff Stages, n_b - Maximum no. of retransmissions allowed, critDelay - Critical delay for PCA packet
int qStateInd[node_num], csmaStateInd[node_num];
int wakeCount[node_num];
int qSlot[node_num];
int pcaInd[node_num];
int w[node_num], m[node_num], n[node_num];
int inB000[node_num], channel[node_num][pkt + ack + 2];
int in_alpha[node_num], in_beta[node_num], fail_alpha[node_num],
		fail_beta[node_num], succ_beta[node_num];
int out_fail_csma_backoff[node_num], out_fail_csma_retry[node_num],
		out_succ[node_num], in[node_num], inPr[node_num];
int delayCsma[node_num], delay[node_num], delayPr[node_num], delayPCA[node_num];
int delayMeasure[node_num], inSleep[node_num];
int bPCA[node_num][Wmax], inAlphaPCA[node_num][Wmax],
		succAlphaPCA[node_num][Wmax], failAlphaPCA[node_num][Wmax],
		succTxdPCA[node_num], collTxdPCA[node_num], inBetaPCA[node_num],
		failBetaPCA[node_num], succBetaPCA[node_num], failDelExp[node_num];
float eta = 0.007; // Packet availability probability
float hPrior = 0.1; // Probability of available packet being priority packet (PCA)
int varMax = 1, iterMax = 1;
FILE *fp_r, *fp_al, *fp_be, *fp_booo, *fp_T, *fp_debug, *fp_delay,
		*fp_pca, *fp_power;
int Dsleep[node_num], Didle[node_num], Dtx[node_num], Drx[node_num];
float Psleep  = 0.00026, Pidle=0.16, Prx=0.17, Ptx=0.16; // Power consumption in different states of the node

void initPCA() {
	int i = 0, j = 0;
	for (i = 0; i < node_num; i++) {
		for (j = 0; j < Wmax; j++) {
			bPCA[i][j] = 0;
			inAlphaPCA[i][j] = 0;
			failAlphaPCA[i][j] = 0;
			succAlphaPCA[i][j] = 0;

		}
		succTxdPCA[i] = 0;
		collTxdPCA[i] = 0;
		failDelExp[i]=0;
		inBetaPCA[i] = 0;
		failBetaPCA[i] = 0;
		succBetaPCA[i] = 0;
		delayMeasure[i] = 0;
		delayPCA[i] = 0;
	}
}

void initIter() {
	int i = 0, j = 0;
	srand(0);
	for (i = 0; i < node_num; i++) {
		qStateInd[i] = 1;
		qSlot[i] = 0;
		csmaStateInd[i] = 0;
		wakeCount[i] = 0;
		inSleep[i] = 0;
		m[i] = 0;
		n[i] = 0;
		w[i] = 0;
		in[i] = 0;
		inPr[i] = 0;
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
		delay[i] = 0;
		delayPr[i] = 0;
		for (j = 0; j <= pkt + ack + 1; j++) {
			channel[i][j] = 0;
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
	float randNum = (double) rand() / (double) ((unsigned)RAND_MAX);
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
int MAX(int a, int b) {
	int c;
	c = (((a) > (b)) ? (a) : (b));
	return (c);
}
int randPCABackOffSlot(int m) {
	return rand() / (RAND_MAX/(Wmax-1));
}
int randBackOffSlot(int m) {
	return rand() /(RAND_MAX/(power(2, MIN(macMaxBE, macMinBE + m))-1));
}
void classifyPkt(int node) {
	if (randNum_0_1() <= hPrior) {
		csmaStateInd[node] = 1;
		pcaInd[node] = 1;
		m[node] = 0;
		w[node] = randPCABackOffSlot(MAX(1, macMaxBE - 1));
		n[node] = 0;
		qStateInd[node] = 0;
		inPr[node] = inPr[node] + 1;
		delayPr[node] = 0;
		delayMeasure[node] = 0;
	} else {
		csmaStateInd[node] = 1;
		pcaInd[node] = 0;
		m[node] = 0;
		w[node] = randBackOffSlot(m[node]);
		n[node] = 0;
		qStateInd[node] = 0;
		in[node] = in[node] + 1;
		delay[node] = -1;
	}
}
void leafNode(int node) {
	if (qStateInd[node] == 1) {
		Didle[node]++;

		inSleep[node]++;
		if (qSlot[node] < h) {
			qSlot[node] = qSlot[node] + 1;
			if (qSlot[node] == h) {
				wakeCount[node] = wakeCount[node] + 1;
				if (randNum_0_1() <= eta) {
					classifyPkt(node);
				} else {
					qSlot[node] = 0;
				}
			}
		}

	} else if (csmaStateInd[node] == 1) {
		if (pcaInd[node] == 0) {
			delay[node] = delay[node] + 1;
			// CCA Contention slots
			if ((w[node] == 0) && (m[node] == 0) && (n[node] == 0)) {
				inB000[node] = inB000[node] + 1;
			}
			if ((w[node] == 0) && (m[node] != -1) && (m[node] != -2)) {
				Drx[node]++;
				in_alpha[node] = in_alpha[node] + 1;
				if (csma_sense() > 0) {
					w[node] = -1;
				} else {
					fail_alpha[node] = fail_alpha[node] + 1;
					if (m[node] < maxBackoffStages) {
						m[node] = m[node] + 1;
						w[node] = randBackOffSlot(m[node]);
					} else {
						wakeCount[node] = wakeCount[node] + 1;
						if (randNum_0_1() <= eta) {
							classifyPkt(node);
						} else {
							qSlot[node] = 0;
							qStateInd[node] = 1;
							csmaStateInd[node] = 0;
						}
						out_fail_csma_backoff[node] =
								out_fail_csma_backoff[node] + 1;
					}
				}
			}

			else if ((w[node] == -1) && (m[node] != -1) && (m[node] != -2)) {
				Drx[node]++;

				in_beta[node] = in_beta[node] + 1;
				if (csma_sense()) {
					succ_beta[node] = succ_beta[node] + 1;
					channel[node][1] = 1;
				} else {
					fail_beta[node] = fail_beta[node] + 1;
					if (m[node] < maxBackoffStages) {
						m[node] = m[node] + 1;
						w[node] = randBackOffSlot(m[node]);
					} else {
						wakeCount[node] = wakeCount[node] + 1;
						if (randNum_0_1() <= eta) {
							classifyPkt(node);
						} else {
							qSlot[node] = 0;
							qStateInd[node] = 1;
							csmaStateInd[node] = 0;
						}
						out_fail_csma_backoff[node] =
								out_fail_csma_backoff[node] + 1;
					}
				}
			} else if ((w[node] > 0) && (m[node] != -1) && (m[node] != -2)) {
				Didle[node]++;
				w[node] = w[node] - 1;
			} else if (m[node] == -2) {
				Dtx[node]++;
				if (w[node] > 1)
					w[node] = w[node] - 1;
				else {
					Dtx[node] = Dtx[node] - 2;
					Didle[node]++;
					Drx[node]++;
					if (n[node] < n_b) {
						m[node] = 0;
						w[node] = randBackOffSlot(m[node]);
						n[node] = n[node] + 1;
					} else {
						wakeCount[node] = wakeCount[node] + 1;
						if (randNum_0_1() <= eta) {
							classifyPkt(node);
						} else {
							qSlot[node] = 0;
							qStateInd[node] = 1;
							csmaStateInd[node] = 0;
						}
						out_fail_csma_retry[node] = out_fail_csma_retry[node]
								+ 1;
					}

				}
			} else if (m[node] == -1) {
				Dtx[node]++;
				if (w[node] > 1)
					w[node] = w[node] - 1;
				else {
					Dtx[node] = Dtx[node] - 2;
					Didle[node]++;
					Drx[node]++;
					wakeCount[node] = wakeCount[node] + 1;
					delayCsma[node] = delayCsma[node] + delay[node];

					if (randNum_0_1() <= eta) {
						classifyPkt(node);
					} else {
						qSlot[node] = 0;
						qStateInd[node] = 1;
						csmaStateInd[node] = 0;
					}
					out_succ[node] = out_succ[node] + 1;
				}
			}

		} // End of CSMA/CA flow
		else { // Start of PCA flow
			delayMeasure[node] = delayMeasure[node]+1;
			delayPr[node] = delayPr[node] + 1;

			if (delayMeasure[node] >= critDelay && (m[node]!=-1 && m[node]!=-2)) {
				failDelExp[node]++;
				if (randNum_0_1() <= eta) {
					classifyPkt(node);
				} else {
					qSlot[node] = 0;
					qStateInd[node] = 1;
					csmaStateInd[node] = 0;
					pcaInd[node]=0;
				}
			} else {
				if (w[node] >= 0 && m[node] == 0) {
					Drx[node]++;
					bPCA[node][w[node]] = bPCA[node][w[node]] + 1;
					in_alpha[node]++;
					inAlphaPCA[node][w[node]] = inAlphaPCA[node][w[node]] + 1;
					if (csma_sense()) {
						succAlphaPCA[node][w[node]] =
								succAlphaPCA[node][w[node]] + 1;
						w[node] = w[node] - 1;

					} else {
						fail_alpha[node]++;
						failAlphaPCA[node][w[node]] =
								failAlphaPCA[node][w[node]] + 1;
					}
				} else if (w[node] == -1 && m[node] == 0) {
					Drx[node]++;
					inBetaPCA[node] = inBetaPCA[node] + 1;
					in_beta[node]++;
					if (csma_sense()) {
						succ_beta[node]++;
						succBetaPCA[node] = succBetaPCA[node] + 1;
						channel[node][1] = 1;

					} else {
						fail_beta[node]++;
						failBetaPCA[node] = failBetaPCA[node] + 1;
						w[node] = 0;
					}
				} else if (m[node] == -1) {
					Dtx[node]++;
					if (w[node] > 1) {
						w[node] = w[node] - 1;
					} else {
						Dtx[node] = Dtx[node] - 2;
						Didle[node]++;
						Drx[node]++;
						delayPCA[node] = delayPCA[node] + delayPr[node];
						succTxdPCA[node] = succTxdPCA[node] + 1;
						qStateInd[node] = 1;
						csmaStateInd[node] = 0;
						if (randNum_0_1() <= eta) {
							classifyPkt(node);
						} else {
							qSlot[node] = 0;
							qStateInd[node] = 1;
							csmaStateInd[node] = 0;
							pcaInd[node]=0;

						}
					}
				} else if (m[node] == -2) {
					Dtx[node]++;
					if (w[node] > 1) {
						w[node] = w[node] - 1;
					} else {
						Dtx[node] = Dtx[node] - 2;
						Didle[node]++;
						Drx[node]++;
						collTxdPCA[node] = collTxdPCA[node] + 1;
						qStateInd[node] = 1;
						csmaStateInd[node] = 0;
						if (randNum_0_1() <= eta) {
							classifyPkt(node);
						} else {
							qSlot[node] = 0;
							qStateInd[node] = 1;
							csmaStateInd[node] = 0;
							pcaInd[node]=0;

						}
					}
				}
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
				for (l = pkt + 1; l <= pkt + ack + 1; l++)
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
	for (j = 0; j < node_num; j++) {
		fprintf(fp_pca, "Node: %d Stats - PCA Reliability: %f\n", j+1, (float) (succTxdPCA[j]) / (inPr[j]));
		fprintf(fp_al, "Node: %d Stats - CCA1 Failure/Busy Probability: %f\n", j+1, (float)fail_alpha[j]/in_alpha[j]);
		fprintf(fp_be, "Node: %d Stats - CCA2 Failure/Busy Probability: %f\n", j+1, (float) fail_beta[j] / in_beta[j]);
		fprintf(fp_booo, "Node: %d Stats - Probability of node residing in state (0,0,0): %f\n", j+1, (float) inB000[j] / t);
		fprintf(fp_T, "Node: %d Stats - Probability of node attempting CCA1 (i,0,j): %f\n", j+1, (float) in_alpha[j] / t);
		fprintf(fp_delay, "Node: %d Stats - Delay for successful packet transmission (CSMA): %f\t Delay for successful packet transmission (PCA): %f;\n", j+1, (float) delayCsma[j] / out_succ[j], (float) delayPCA[j] / succTxdPCA[j]);
		fprintf(fp_r,"Node: %d Stats - CSMA Reliability: %f\n", j+1, (float)(out_succ[j])/(in[j]));
		fprintf(fp_power,"Node: %d Stats - Average Power Consumption (W): %f\n", j+1, (float)(Didle[j]*Psleep + Dtx[j]*Ptx + Drx[j]*Prx)/t);
	}

}
void debugGen(int time) {
	int i = 0;
	for (i = 0; i < node_num; i++) {
		fprintf(fp_debug,
				"{t:%d|I:%d|I_N:%d|C:%d|P:%d|m:%d|w:%d|n:%d|ch:%d|be:%d|fai:%d}\t",
				time, qStateInd[i], qSlot[i], csmaStateInd[i], pcaInd[i], m[i],
				w[i], n[i], channel[i][0], delay[i], fail_beta[i]);
	}
	fprintf(fp_debug, "\n");
}

int main() {
	int i, j, iter, var;

// File pointer initializations
	fp_r = fopen("/results/CSMA Reliability.txt", "w");
	fp_al = fopen("/results/CCA1 Fail Probabilities.txt", "w");
	fp_be = fopen("/results/CCA2 Fail Probabilities.txt", "w");
	fp_booo = fopen("/results/b000 Probabilities.txt", "w");
	fp_T = fopen("/results/Tau Probabilities.txt", "w");
	fp_debug = fopen("/results/Debug.txt", "w");
	fp_delay = fopen("/results/Packet Delay.txt", "w");
	fp_pca = fopen("/results/PCA Stats.txt", "w");
	fp_power = fopen("/results/Power.txt", "w");

	setvbuf(stdout, NULL, _IONBF, 0);

	printf("Process Started for IEEE 802.15.4-2015 Standard\n");
	for (var = 0; var < varMax; var++) {
		for (iter = 0; iter < iterMax; iter++) {
			initIter();
			initPCA();
			printf("\nProgress %d-%d\n", var, iter);
			for (i = 0; i < t; i++) {
				if (debugEn)
					debugGen(i);

				for (j = 0; j < node_num; j++) {
					leafNode(j);
				}
				sense_collission();
				shifter();
			}
		}
		logResult();
		printf("Time: %d", i);
	}
	printf("\n*************--Process Completed--***************\n");
	return (1);
}
