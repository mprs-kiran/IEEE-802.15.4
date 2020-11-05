/*802_11_EDCA
 Created on: 05-Apr-2018
 Author: kiran*/

#include <sys/types.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <vector>
#define ON_DEMAND_ALLOCATION
#undef ON_DEMAND_ALLOCATION

//#define SCHEDULE_USING_QUEUE_SIZE
//#define SCHEDULE_USING_SUCC_TXD
#define SCHEDULE_USING_TRAFFIC_ESTM

using namespace std;
std::random_device rd;     // only used once to initialize (seed) engine
std::default_random_engine rng(rd()); // random-number engine used (Mersenne-Twister in this case)

class EdcaSimulationSetup {
public:
	EdcaSimulationSetup(long nodes, long cwMin, long cwMax, long slrc,
			double simDuration, long pktSize, long rts, long cts, long ack, long H,
			double dr, long BI, double beaconFrac, long noOfSectors,
			std::vector<double> traffic);
	void setSifs(double dur);
	void setSlot(double dur);
	void setDifs(double dur);
	long getSecondsInSlots(double time);
	long getSlotsFromMilliSeconds(double time);
	void run();
	void staOperation(long sector, long node, long t);
	long genRandomBackoff(long NB);
	void manageNetworkAccess(long t);
	long getSlotsFromBytes(long bytes);
	long getSlotsFromMicroSeconds(double time);
	long getSlotsFromBits(long bits);
	void printStats(long);
	void printDetailedFinalStats(long);
	void printExcelCompatFinalStats(long);
	void initChannel();
	void queueUpdater();
	void uniformSectorAllocation();
	void onDemandSectorAllocation();
	void populateCbapTimeUsingQueueSize();
	void populateCbapTimeWithSuccessfulTxdEstimation();
	void populateCbapTimeWithCongestionEstimation();
	void checkIfStaReadyForTxn(long sector, long node);
private:
	float aggregateTPT = 0;
	float aggregateDelay = 0;
	float aggregateCollDelay = 0;
	long m_nodes = 10;
	vector<double> m_traffic = { 1.0, 1.0, 1.0, 1.0 };
	long m_cwMax = 1024;
	long m_minBE = 5;
	long m_slrc = 7;
	vector<long> m_backoffProb = { 0, 0, 0, 0, 0, 0, 0, 0 };
	long m_pktSize = 1000, m_cn = 0, succTime = 0, collTime = 0;
	long m_beaconDur = 0;
	long m_cbap = 0;
	long m_cfp = 0;
	double m_cbapFrac = 1;
	long m_sectors = 1;
	long tx = 0;
	double m_dr = 2000000000.0;
	double m_simDuration = 1000;
	double slotTime_us = 5; // Slot time in us
	double sifs = 2.5;
	double difs = 13.5;
	double rifs = 9;
	bool* channelState = NULL;
	bool m_debug = false;
	bool m_printNodeStats = true;
	long *secWisepktsGen;
	long *secWiseChanEff;
	long *secWiseChanAlloc;
	long *secWisepktsSer;
	long *secWisepktsOve;
	long *secWisepktsSucc;
	float *secWiseSuccDel;
	float *secWiseCollDel;

	long *secWisePktsTxdPrevBI;
	long *secWiseCongestionCounter;

	long slotsPerSec = 200000;
	long rts, cts, m_HeadSize, ACK;
	struct SuperframeState {
		long startOfBeacon = 0;
		long startOfCbap = 0;
		long endOfCbap = 0;
		long endOfBeacon = 0;
		long currentSector = 0;
		long remainingTime = 0;
		long timePerSector = 0;
		long *cbapRemain;
	} sfState;

	struct DcfState {
		long tEvent = 1;
		long qLim = 5000;
		long qArrivals = 0;
		long qOverflow = 0;
		long nodeNum = 0;
		long NB = 0;
		long backoff = 0;
		long prevBackoff = 0;
		long freezingCount = 0;
		bool inSuccTx = 0;
		bool inCollTx = 0;
		long txRemaining = 0;
		bool readyForTxn = false;
		long pktServed = 0;
		long pktsTxdSucc = 0;
		long pktsTxdColl = 0;
		long pktsDropped = 0;
		long stTime = 0;
		long endTime = 0;
		bool inSleep = true;
		long accTimeForPktTx = 0;
		bool inCBAP = true;
		long b000 = 0;
		long coll = 0;
		long totIn = 0;
		long totFail = 0;
		long csIn = 0;
		long csFail = 0;
		long succDelay = 0;
		long collDelay = 0;
		long delayCount = 0;
		long inTx = 0;
		long freezeFail = 0;
		long tau = 0;
		long pktCheck = 0;
		long pktPresence = 0;
		long avgQSize = 0;
		long qSize = 0;
	} dcfState;

	std::vector<DcfState> nodeList;
	std::vector<std::vector<DcfState>> sectorWiseNodeList;

};

EdcaSimulationSetup::EdcaSimulationSetup(long nodes, long minBE, long cwMax,
		long slrc, double simDuration, long pktSize, long rts, long cts, long ack,
		long H, double dr, long BI, double beaconFrac, long noOfSectors,
		std::vector<double> traffic) :
						m_nodes(nodes), m_minBE(minBE), m_cwMax(cwMax), m_slrc(slrc), m_simDuration(
								simDuration), m_pktSize(pktSize), m_dr(dr), m_HeadSize(H), ACK(ack), rts(
										rts), cts(cts), m_traffic(traffic), m_sectors(noOfSectors), m_cbapFrac(
												beaconFrac) {
	sfState.cbapRemain = new long[m_sectors];
	secWisepktsGen = new long[m_sectors];
	secWiseChanEff = new long[m_sectors];
	secWiseChanAlloc = new long[m_sectors];
	secWisepktsSer = new long[m_sectors];
	secWisepktsOve = new long[m_sectors];
	secWisepktsSucc = new long[m_sectors];
	secWiseSuccDel = new float[m_sectors];
	secWiseCollDel = new float[m_sectors];
	secWisePktsTxdPrevBI = new long[m_sectors];
	secWiseCongestionCounter = new long[m_sectors];
	for (long i = 0; i < m_sectors; i++) {
		sfState.cbapRemain[i] = m_cbap / m_sectors;
	}
	for (long j = 0; j < m_sectors; j++) {
		secWisepktsGen[j] = 0;
		secWisepktsSer[j] = 0;
		secWisepktsOve[j] = 0;
		secWisepktsSucc[j] = 0;
		secWiseSuccDel[j] = 0;
		secWiseChanAlloc[j] = 0;
		secWiseChanEff[j] = 0;
		secWisePktsTxdPrevBI[j] = 0;
		secWiseCongestionCounter[j] = 0;
	}
	m_beaconDur = getSlotsFromMilliSeconds(BI);
	std::cout << "Beacon duration " << m_beaconDur << std::endl;
	m_cbap = m_beaconDur * m_cbapFrac;
	std::cout << "CBAP duration " << m_cbap << std::endl;
}
void EdcaSimulationSetup::manageNetworkAccess(long t) {
	if(!channelState[0]) {
		secWiseCongestionCounter[sfState.currentSector]++;
		secWiseChanEff[sfState.currentSector]++;
	}
	secWiseChanAlloc[sfState.currentSector]++;
	for (long i = 0; i < m_cn - 1; i++) {
		channelState[i] = channelState[i + 1]; //move all element to the left except first one
	}
	channelState[m_cn - 1] = true;
	long stReady = 0;

	for (long node = 0; node < m_nodes; node++) {
		if (sectorWiseNodeList[sfState.currentSector][node].readyForTxn) {
			stReady++;
			sectorWiseNodeList[sfState.currentSector][node].inTx++;
		}
	}
	if(stReady>0) tx++;
	if (stReady == 1) {
		for (long node = 0; node < m_nodes; node++) {
			if (sectorWiseNodeList[sfState.currentSector][node].readyForTxn) {
				sectorWiseNodeList[sfState.currentSector][node].inSuccTx = true;
				sectorWiseNodeList[sfState.currentSector][node].txRemaining =
						succTime;
				sectorWiseNodeList[sfState.currentSector][node].readyForTxn =
						false;

				break;
			}
		}
		for (long i = 0; i <= succTime; i++) {
			channelState[i] = false;
		}
	} else if (stReady > 1) {
		for (long node = 0; node < m_nodes; node++) {
			if (sectorWiseNodeList[sfState.currentSector][node].readyForTxn) {
				sectorWiseNodeList[sfState.currentSector][node].coll++;
				sectorWiseNodeList[sfState.currentSector][node].inCollTx = true;
				sectorWiseNodeList[sfState.currentSector][node].readyForTxn =
						false;
				sectorWiseNodeList[sfState.currentSector][node].txRemaining =
						collTime;
			}
		}
		for (long i = 0; i <= collTime; i++) {
			channelState[i] = false;
		}
	}
#ifdef ON_DEMAND_ALLOCATION
	onDemandSectorAllocation();
#else
	uniformSectorAllocation();
#endif
}

void EdcaSimulationSetup::uniformSectorAllocation() {
	sfState.remainingTime--;
	if (sfState.remainingTime == -1) {
		sfState.currentSector = sfState.currentSector + 1;
		sfState.currentSector = (sfState.currentSector) % m_sectors;
		sfState.remainingTime = sfState.timePerSector;
	}
}
void EdcaSimulationSetup::onDemandSectorAllocation() {
	sfState.cbapRemain[sfState.currentSector]--;
	if (sfState.cbapRemain[sfState.currentSector] == -1) {
		sfState.currentSector = sfState.currentSector + 1;
		sfState.currentSector = (sfState.currentSector) % m_sectors;
		if (sfState.currentSector == 0) {
#ifdef SCHEDULE_USING_QUEUE_SIZE
			populateCbapTimeUsingQueueSize();
#endif
#ifdef SCHEDULE_USING_SUCC_TXD
			populateCbapTimeWithSuccessfulTxdEstimation();
#endif
#ifdef SCHEDULE_USING_TRAFFIC_ESTM
			populateCbapTimeWithCongestionEstimation();
#endif
		}
	}
}

void EdcaSimulationSetup::populateCbapTimeUsingQueueSize() {
	long queSizePerSec[m_sectors];
	long totalPkts = 0;
	for (long sec = 0; sec < m_sectors; sec++) {
		queSizePerSec[sec] = 0;
		for (long node = 0; node < m_nodes; node++) {
			queSizePerSec[sec] += sectorWiseNodeList[sec][node].qSize;
			totalPkts += sectorWiseNodeList[sec][node].qSize;
		}
	}
	for (long sec = 0; sec < m_sectors; sec++) {
		sfState.cbapRemain[sec] = (float) queSizePerSec[sec] * m_cbap
				/ totalPkts;
		//		std::cout<<(float)queSizePerSec[sec]/totalPkts<<"-"<<sfState.cbapRemain[sec]<<" - "<<queSizePerSec[sec]<<"|";
	}
	//	std::cout<<std::endl;

}
void EdcaSimulationSetup::populateCbapTimeWithSuccessfulTxdEstimation() {
	long secWisePktsTxdUntil[m_sectors];
	long totalPktsSucc = 0;
	for (long sec = 0; sec < m_sectors; sec++) {
		secWisePktsTxdUntil[sec] = 0;
		for (long node = 0; node < m_nodes; node++) {
			secWisePktsTxdUntil[sec] = sectorWiseNodeList[sec][node].pktsTxdSucc+1;
		}
		totalPktsSucc += secWisePktsTxdUntil[sec]-secWisePktsTxdPrevBI[sec];
		//		std::cout<<sec<<"|"<<secWisePktsTxdUntil[sec]<<"|"<<secWisePktsTxdPrevBI[sec]<<std::endl;
	}

	for (long sec = 0; sec < m_sectors; sec++) {
		sfState.cbapRemain[sec] = (float) (secWisePktsTxdUntil[sec]-secWisePktsTxdPrevBI[sec]) * m_cbap
				/ totalPktsSucc;
		//		std::cout<<(float)(secWisePktsTxdUntil[sec]-secWisePktsTxdPrevBI[sec])/totalPktsSucc<<"-"<<sfState.cbapRemain[sec]<<"-"<<secWisePktsTxdUntil[sec]-secWisePktsTxdPrevBI[sec]<<"|";
		secWisePktsTxdPrevBI[sec] = secWisePktsTxdUntil[sec];
	}
	//	std::cout<<std::endl;

}
void EdcaSimulationSetup::populateCbapTimeWithCongestionEstimation() {
	long totalCong = 0;
	for (long sec = 0; sec < m_sectors; sec++) {

		totalCong += 1+secWiseCongestionCounter[sec];
		//		std::cout<<sec<<"|"<<secWiseCongestionCounter[sec]<<std::endl;
	}

	for (long sec = 0; sec < m_sectors; sec++) {
		sfState.cbapRemain[sec] = (float) (1+secWiseCongestionCounter[sec]) * m_cbap
				/ totalCong;
		//		std::cout<<(float)(1+secWiseCongestionCounter[sec])/totalCong<<"-"<<sfState.cbapRemain[sec]<<"|";
		secWiseCongestionCounter[sec] = 0;
	}
	//	std::cout<<std::endl;

}
void EdcaSimulationSetup::initChannel() {
	for (long i = 0; i < m_cn; i++) {
		channelState[i] = true;
	}
}
long EdcaSimulationSetup::getSecondsInSlots(double time) {
	long slotsCalc = time * 1000000 / slotTime_us;
	return slotsCalc;
}
long EdcaSimulationSetup::getSlotsFromBytes(long bytes) {
	//	long slotsCalc =  (unsigned long)(((bytes *8*1000000) / slotTime / (m_dr)));
	//	std::cout<<"Packet: "<<(double)((bytes *8) / slotTime / (m_dr))*1000000<<std::endl;
	return ((((double) bytes * 8 * 1000000) / slotTime_us / (m_dr)));
}
long EdcaSimulationSetup::getSlotsFromBits(long bits) {
	long slotsCalc = bits*1000000/ slotTime_us / m_dr;
	return slotsCalc;
}
long EdcaSimulationSetup::getSlotsFromMicroSeconds(double time) {
	long slotsCalc = time / slotTime_us;
	if (time > 0 && slotsCalc == 0)
		slotsCalc++;
	return slotsCalc;
}
long EdcaSimulationSetup::getSlotsFromMilliSeconds(double time) {
	long slotsCalc = time * 1000 / slotTime_us;
	if (time > 0 && slotsCalc == 0)
		slotsCalc++;
	return slotsCalc;
}
void EdcaSimulationSetup::printDetailedFinalStats(long iter) {
	std::cout << std::endl << std::endl;

	std::cout
	<< "**************************  Final stats after all iterations (Sector and AC Wise) *********************************** "
	<< std::endl;

	for (long i = 0; i < m_sectors; i++) {
		std::cout << std::endl << "Sector: " << i << std::endl;
		std::cout << " POR: "
				<< (float) secWisepktsOve[i] * 100 / secWisepktsGen[i] << "%"
				<< " PT: " << secWisepktsSucc[i] << " PG: " << secWisepktsGen[i]
																			  << " TPT: "
																			  << (secWisepktsSucc[i] / iter / m_dr / m_simDuration)
																			  * m_pktSize * 8 << " Delay: "
																			  << secWiseSuccDel[i] * slotTime_us * 1e-3 / iter << "ms"
																			  << std::endl;
	}

	std::cout << std::endl << std::endl
			<< "**************************  Final stats after all iterations *********************************** "
			<< std::endl;
	std::cout << "Aggregate TPT: " << aggregateTPT / m_sectors/iter << " Average Succ Delay: "
			<< aggregateDelay * slotTime_us * 1e-3 / iter << "ms"
			<< " Average Coll. Delay: " << aggregateCollDelay * slotTime_us * 1e-3 / iter << "ms"<< std::endl;

}
void EdcaSimulationSetup::printExcelCompatFinalStats(long iter) {
	std::cout << std::endl << std::endl;

	std::cout
	<< "**************************  Final stats after all iterations (Sector and AC Wise) *********************************** "
	<< std::endl;


//	std::cout << ((float)tx/getSecondsInSlots(m_simDuration))<<"\t";

	for (long i = 0; i < m_sectors; i++) {
		std::cout << (secWisepktsSucc[i] / iter / m_dr / m_simDuration)
										* m_pktSize * 8<<"\t";
	}
	for (long i = 0; i < m_sectors; i++) {
		std::cout << secWiseSuccDel[i]/* * slotTime_us * 1e-3*/ / iter<<"\t";
	}
	for (long i = 0; i < m_sectors; i++) {
		std::cout << secWiseCollDel[i]/* * slotTime_us * 1e-3*/ / iter<<"\t";
	}
	for (long i = 0; i < m_sectors; i++) {
		std::cout << (float)secWiseChanEff[i] / secWiseChanAlloc[i]<<"\t";
	}
	for (long i = 0; i < m_sectors; i++) {
		std::cout << (float) secWisepktsOve[i] / secWisepktsGen[i]<<"\t";
	}


	std::cout << std::endl << std::endl
			<< "**************************  Final stats after all iterations *********************************** "
			<< std::endl;
	std::cout << "Aggregate TPT: " << aggregateTPT/iter << " Average Succ Delay: "
			<< aggregateDelay/* * slotTime_us * 1e-3*/ / iter/m_sectors << "ms" <<
			" Average Coll Delay: " << aggregateCollDelay/* * slotTime_us * 1e-3*/ / iter/m_sectors << "ms" <<std::endl;

}

void EdcaSimulationSetup::printStats(long currIter) {
	long pktsTxdSuccessfully = 0;
	long pktsGen[m_sectors];
	long pktsSer[m_sectors];
	long pktsOver[m_sectors];
	long pktsSucc[m_sectors];
	float delaySucc[m_sectors];
	long pktsColl[m_sectors];
	float delayColl[m_sectors];
	long delaySuccNum[m_sectors];
	long delaySuccDen[m_sectors];
	long delayCollNum[m_sectors];
	long delayCollDen[m_sectors];
	long nodCount[m_sectors];
	long pktCheck[m_sectors];
	long pktPresence[m_sectors];
	long nodesWithDelay = 0;
	double avgSuccDelay = 0;
	long nodCollCount[m_sectors];
	long nodesWithCollDelay=0;
	double avgCollDelay = 0;


	for (int sec=0; sec < m_sectors; sec++) {
		pktsGen[sec] = 0;
		pktsSer[sec] = 0;
		pktsOver[sec] = 0;
		pktsSucc[sec] = 0;
		delaySucc[sec] = 0;
		pktsColl[sec] = 0;
		delayColl[sec] = 0;
		delaySuccNum[sec] = 0;
		delaySuccDen[sec] = 0;
		delayCollNum[sec] = 0;
		delayCollDen[sec] = 0;
		nodCount[sec] = 0;
		pktCheck[sec] = 0;
		pktPresence[sec] = 0;
	}

	for (long sector = 0; sector < m_sectors; sector++) {
		for (long node = 0; node < m_nodes; node++) {
			if (m_printNodeStats) {
				std::cout << "--------- Sector: " << sector << " Node: " << node
						<< " ---------" << std::endl;
				std::cout << "Packets Generated: "
						<< sectorWiseNodeList[sector][node].qArrivals
						<< " | Packets Served: "
						<< sectorWiseNodeList[sector][node].pktServed
						<< " | Packets Overflown: "
						<< sectorWiseNodeList[sector][node].qOverflow
						<< " | Avg. queue size: "
						<< (float)sectorWiseNodeList[sector][node].avgQSize/getSecondsInSlots(m_simDuration)
						<< std::endl;
				std::cout << "Packets Transmitted: "
						<< sectorWiseNodeList[sector][node].pktsTxdSucc
						<< std::endl;
				std::cout << "Packets Dropped: "
						<< sectorWiseNodeList[sector][node].pktsDropped
						<< std::endl;
				std::cout << "b000 prob: "
						<< (double) sectorWiseNodeList[sector][node].b000
						/ getSecondsInSlots(m_simDuration) << std::endl;
				std::cout << "Tau prob: "
						<< (double) sectorWiseNodeList[sector][node].tau
						/ getSecondsInSlots(m_simDuration) << std::endl;
				std::cout << "Coll prob: "
						<< (double) sectorWiseNodeList[sector][node].coll
						/ sectorWiseNodeList[sector][node].inTx
						<< std::endl;

				std::cout << "Pb: "
						<< (double) sectorWiseNodeList[sector][node].csFail
						/ sectorWiseNodeList[sector][node].csIn
						<< std::endl;
				std::cout << "Rel: "
						<< (double) sectorWiseNodeList[sector][node].pktsTxdSucc
						/ sectorWiseNodeList[sector][node].pktServed
						<< std::endl;

				if (sectorWiseNodeList[sector][node].pktsTxdSucc > 0)
					std::cout << "Avg. Medium Access Delay: "
					<< (double) sectorWiseNodeList[sector][node].accTimeForPktTx
					/ sectorWiseNodeList[sector][node].pktServed
					<< endl << "Avg. Success Delay: "
					<< (double) sectorWiseNodeList[sector][node].succDelay
					/ sectorWiseNodeList[sector][node].pktsTxdSucc
					<< std::endl;
				if (sectorWiseNodeList[sector][node].pktsTxdColl > 0)
					std::cout << "Avg. Coll. Delay: "
					<< (double) sectorWiseNodeList[sector][node].collDelay
					/ sectorWiseNodeList[sector][node].pktsTxdColl
					<< std::endl;

			}
			pktsGen[sector] += sectorWiseNodeList[sector][node].qArrivals;
			pktsSer[sector] += sectorWiseNodeList[sector][node].pktServed;
			pktsOver[sector] += sectorWiseNodeList[sector][node].qOverflow;
			pktsSucc[sector] += sectorWiseNodeList[sector][node].pktsTxdSucc;
			delaySuccNum[sector] += sectorWiseNodeList[sector][node].succDelay;
			delaySuccDen[sector] += sectorWiseNodeList[sector][node].pktsTxdSucc;
			delayCollNum[sector] += sectorWiseNodeList[sector][node].collDelay;
			delayCollDen[sector] += sectorWiseNodeList[sector][node].pktsTxdColl;
			pktsTxdSuccessfully += sectorWiseNodeList[sector][node].pktsTxdSucc;
			pktCheck[sector] += sectorWiseNodeList[sector][node].pktCheck;
			pktPresence[sector] += sectorWiseNodeList[sector][node].pktPresence;

			if (sectorWiseNodeList[sector][node].pktsTxdSucc > 0) {
				nodCount[sector]++;
				nodesWithDelay++;
				avgSuccDelay +=
						(double) sectorWiseNodeList[sector][node].succDelay
						/ sectorWiseNodeList[sector][node].pktsTxdSucc;
			}
			if (sectorWiseNodeList[sector][node].pktsTxdColl > 0) {
				nodCollCount[sector]++;
				nodesWithCollDelay++;
				avgCollDelay +=
						(double) sectorWiseNodeList[sector][node].collDelay
						/ sectorWiseNodeList[sector][node].pktsTxdColl;
			}

		}

	}
	for (long sector = 0; sector < m_sectors; sector++) {
		secWisepktsGen[sector] += pktsGen[sector];
		secWisepktsSer[sector] += pktsSer[sector];
		secWisepktsOve[sector] += pktsOver[sector];
		secWisepktsSucc[sector] += pktsSucc[sector];
		delaySucc[sector] = (float) delaySuccNum[sector]/delaySuccDen[sector];
		delayColl[sector] = (float) delayCollNum[sector]/delayCollDen[sector];
		secWiseSuccDel[sector] += delaySucc[sector];
		secWiseCollDel[sector] += delayColl[sector];
	}

	std::cout << "Packets Transmitted: " << (pktsTxdSuccessfully) << std::endl;
	std::cout << "Total Network Throughput: "
			<< (pktsTxdSuccessfully / m_dr / m_simDuration) * m_pktSize * 8
			<< std::endl;
#ifdef ON_DEMAND_ALLOCATION
	std::cout<<"Performance using on demand allocation"<<std::endl;
#else
	std::cout<<"Performance using uniform allocation"<<std::endl;
#endif
	std::cout << std::endl << std::endl
			<< "------------------- Sector Wise Stats (" << currIter
			<< ") -------------------" << std::endl;
	for (long i = 0; i < m_sectors; i++) {
		std::cout << std::endl << " Sector: " << i << std::endl;
		std::cout << "Total Sector Throughput: "
				<< ((pktsSucc[i]) / m_dr / m_simDuration) * m_pktSize * 8
				<< "   Eta: "<< (float)pktPresence[i]/pktCheck[i]<<std::endl;
	}

	aggregateTPT += (double) (pktsTxdSuccessfully / m_dr / m_simDuration)
							* m_pktSize * 8;
	aggregateDelay += (double) avgSuccDelay / nodesWithDelay;

	aggregateCollDelay += (double) avgCollDelay / nodesWithCollDelay;

}


double rand_0_1() {
	double r = rand() / double(RAND_MAX);
	//	std::cout<<r<<std::endl;
	return r;
}



void EdcaSimulationSetup::run() {
	succTime = getSlotsFromBits(rts + cts + ACK) + getSlotsFromBytes(m_pktSize) + getSlotsFromBits(m_HeadSize) +
										+ getSlotsFromMicroSeconds(difs+3 * sifs);
	collTime = getSlotsFromBits(rts) + getSlotsFromMicroSeconds(difs);

	m_cn = 2 * succTime;
	std::cout << "Simulation values to use - Collision time: " << collTime + 1
			<< ", Succ time: " << succTime + 1 << ", Pkt time: "
			<< getSlotsFromBytes(m_pktSize) << std::endl;
	channelState = new bool[m_cn];
	initChannel();

	sfState.currentSector = 0;
	sfState.endOfBeacon = m_beaconDur;
	sfState.endOfCbap = m_cbap;
	sfState.startOfCbap = 0;
	sfState.startOfBeacon = 0;
	sfState.timePerSector = m_cbap / m_sectors;
	sfState.remainingTime = sfState.timePerSector;
	std::cout << "Time Per Sector: " << sfState.timePerSector << std::endl;
	nodeList.clear();
	sectorWiseNodeList.clear();


	for (long sector = 0; sector < m_sectors; sector++) {
		for (long node = 0; node < m_nodes; node++) {
			dcfState.nodeNum = node;
			dcfState.backoff = genRandomBackoff(dcfState.NB);
			dcfState.tEvent = 1+(float) (-log(rand_0_1()) * slotsPerSec) / m_traffic[sector];
			nodeList.push_back(dcfState);
		}
		sectorWiseNodeList.push_back(nodeList);
	}
	std::cout << "CBAP time for each sector: " << sfState.timePerSector
			<< std::endl;
	std::cout << "Simulation time: " << getSecondsInSlots(m_simDuration)
							<< std::endl;
	for (long t = 0; t < getSecondsInSlots(m_simDuration); t++) {
		for (long node = 0; node < m_nodes; node++) {
			staOperation(sfState.currentSector, node, t);
		}
		if (m_debug)
			std::cout << std::endl;
		manageNetworkAccess(t);
		queueUpdater(); // This should be executed for every sector irrespective of current sector, hence is run for every time instance
	}
}

long EdcaSimulationSetup::genRandomBackoff(long NB) {
	long minB = 0;
	long maxB = std::min((long) m_cwMax, (long) pow(2, m_minBE + NB)) - 1;
	std::uniform_int_distribution<long> uni(minB, maxB); // guaranteed unbiased
	long rn = uni(rng);
//	std::cout<<rn<<std::endl;
	return rn;
}


void EdcaSimulationSetup::queueUpdater() {
	for (long sector = 0; sector < m_sectors; sector++) {
		for (long node = 0; node < m_nodes; node++) {

			/*
			sectorWiseNodeList[sector][node].pktCheck++;
			if(sectorWiseNodeList[sector][node].qSize >0)
				sectorWiseNodeList[sector][node].pktPresence++;
			 */

			sectorWiseNodeList[sector][node].avgQSize = sectorWiseNodeList[sector][node].avgQSize  + sectorWiseNodeList[sector][node].qSize;
			sectorWiseNodeList[sector][node].tEvent--;
			if (sectorWiseNodeList[sector][node].tEvent == 0) {
				sectorWiseNodeList[sector][node].qArrivals++;
				if (sectorWiseNodeList[sector][node].qSize
						< sectorWiseNodeList[sector][node].qLim) {
					sectorWiseNodeList[sector][node].qSize++;
				} else {
					sectorWiseNodeList[sector][node].qOverflow++;
				}
				sectorWiseNodeList[sector][node].tEvent = 1+(float) (-log(
						rand_0_1()) * slotsPerSec) / m_traffic[sector];
//				sectorWiseNodeList[sector][node].tEvent =
//						(sectorWiseNodeList[sector][node].tEvent < 1) ?
//								1 : sectorWiseNodeList[sector][node].tEvent;
			}

		}
	}
}
void EdcaSimulationSetup::checkIfStaReadyForTxn(long sector, long node) {
	/*if(sectorWiseNodeList[sector][node].backoff == 0) {
		if (sectorWiseNodeList[sector][node].NB == 0)
			sectorWiseNodeList[sector][node].b000++;
		sectorWiseNodeList[sector][node].tau++;
		//		if (sfState.remainingTime > succTime) {
		sectorWiseNodeList[sector][node].csIn++;
			if (channelState[0]) {
				sectorWiseNodeList[sector][node].readyForTxn = true;
			} else {
				sectorWiseNodeList[sector][node].backoff = 0;
				sectorWiseNodeList[sfState.currentSector][node].csFail++;
			}
		} else {
			std::cout<<"Sector Switch"<<std::endl;
		}
	}*/
}

void EdcaSimulationSetup::staOperation(long sector, long node, long t) {

	if (m_debug) {
		long cs = (channelState[0]) ? 0 : 1;
		std::cout << "\t|||\t" << " Curr Sec:" << sfState.currentSector << " T:"
				<< sectorWiseNodeList[sector][node].tEvent << "-" << "-"
				<< " Node:" << node << "- Sleep: "
				<< sectorWiseNodeList[sector][node].inSleep << " -" << " Ch. P:"
				<< sectorWiseNodeList[sector][node].readyForTxn << "-"
				<< " Col:" << sectorWiseNodeList[sector][node].inCollTx << "-"
				<< " Succ:" << sectorWiseNodeList[sector][node].inSuccTx << "-"
				<< " NB:" << sectorWiseNodeList[sector][node].NB << "-" << " B:"
				<< sectorWiseNodeList[sector][node].backoff << "-" << " TXR:"
				<< sectorWiseNodeList[sector][node].txRemaining << "-" << " FC:"
				<< sectorWiseNodeList[sector][node].freezingCount << "-"
				<< "SD:" << sectorWiseNodeList[sector][node].delayCount << "-"
				<< "ASD:" << sectorWiseNodeList[sector][node].succDelay << "-"
				<< "SC:" << sectorWiseNodeList[sector][node].pktsTxdSucc << "-"
				<< "CC:" << sectorWiseNodeList[sector][node].coll << "-"
				<< "QS:" << sectorWiseNodeList[sector][node].qSize << "-"
				<< " CS:" << cs;
		if (sectorWiseNodeList[sector][node].qSize > 10) {
			exit(0);
		}
	}
	if (sectorWiseNodeList[sector][node].inSleep) {
		sectorWiseNodeList[sector][node].stTime = t + 1;
		sectorWiseNodeList[sector][node].pktCheck++;
//		if(rand_0_1()<=0.00532) {
		if (sectorWiseNodeList[sector][node].qSize > 0) {
			sectorWiseNodeList[sector][node].inSleep = false;
			checkIfStaReadyForTxn(sector, node);
			sectorWiseNodeList[sector][node].pktServed++;
			sectorWiseNodeList[sector][node].pktPresence++;
		}
	} else {
		sectorWiseNodeList[sector][node].delayCount++;
		if (!sectorWiseNodeList[sector][node].inSuccTx
				&& !sectorWiseNodeList[sector][node].inCollTx) {
			sectorWiseNodeList[sector][node].csIn++;
			if (sectorWiseNodeList[sector][node].backoff >= 1) {
//				if (sectorWiseNodeList[sector][node].NB == 0 && sectorWiseNodeList[sector][node].backoff==3)
//									sectorWiseNodeList[sector][node].b000++;
				if (channelState[0]) {
					sectorWiseNodeList[sector][node].backoff--;
				} else {
					sectorWiseNodeList[sfState.currentSector][node].csFail++;
				}
			} else if (sectorWiseNodeList[sector][node].backoff == 0) {
				if (sectorWiseNodeList[sector][node].NB == 0)
							sectorWiseNodeList[sector][node].b000++;
				sectorWiseNodeList[sector][node].tau++;
				//				if (sfState.remainingTime > succTime) {
				if (channelState[0]) {
					sectorWiseNodeList[sector][node].readyForTxn = true;
				} else {
					sectorWiseNodeList[sector][node].backoff = 0;
					sectorWiseNodeList[sfState.currentSector][node].csFail++;
				}
				//}
			}
		} else if (sectorWiseNodeList[sector][node].inSuccTx) {
			if (sectorWiseNodeList[sector][node].txRemaining >= 1) {
				sectorWiseNodeList[sector][node].txRemaining--;
			} else {
				sectorWiseNodeList[sector][node].pktsTxdSucc++;
				sectorWiseNodeList[sector][node].inSuccTx = false;
				sectorWiseNodeList[sector][node].qSize--;
				sectorWiseNodeList[sector][node].NB = 0;
				sectorWiseNodeList[sector][node].backoff = genRandomBackoff(
						sectorWiseNodeList[sector][node].NB);
//				std::cout<<sectorWiseNodeList[sector][node].backoff<<std::endl;
				sectorWiseNodeList[sector][node].prevBackoff =sectorWiseNodeList[sector][node].backoff;
								sectorWiseNodeList[sector][node].accTimeForPktTx += (t
										- sectorWiseNodeList[sector][node].stTime) - succTime;
				sectorWiseNodeList[sector][node].succDelay +=(t
						- sectorWiseNodeList[sector][node].stTime + 1);
				sectorWiseNodeList[sector][node].stTime = t + 1;
				sectorWiseNodeList[sector][node].delayCount = 0;
				sectorWiseNodeList[sector][node].pktCheck++;
//				if(rand_0_1()>0.00532) {
				if(sectorWiseNodeList[sector][node].qSize == 0) {
					sectorWiseNodeList[sector][node].inSleep = true;
				} else {
					sectorWiseNodeList[sector][node].pktServed++;
					sectorWiseNodeList[sector][node].pktPresence++;
					checkIfStaReadyForTxn(sector, node);
				}
			}
		} else if (sectorWiseNodeList[sector][node].inCollTx) {
			if (sectorWiseNodeList[sector][node].txRemaining >= 1) {
				sectorWiseNodeList[sector][node].txRemaining--;
			} else {

				sectorWiseNodeList[sector][node].inCollTx = false;
				sectorWiseNodeList[sector][node].NB++;
				if (sectorWiseNodeList[sector][node].NB > m_slrc) {
					sectorWiseNodeList[sector][node].backoff = genRandomBackoff(
							m_slrc);
					checkIfStaReadyForTxn(sector, node);
				} else {
					sectorWiseNodeList[sector][node].backoff = genRandomBackoff(
							sectorWiseNodeList[sector][node].NB);
					checkIfStaReadyForTxn(sector, node);
				}
			}

		}
	}
}

int main(int argc, char *argv[]) {
	srand(time(0));
	long iter = 1;
	long nodesPerSec = 20; // Currently supporting only uniform node distribution

	long minBE = 5;
	long mkl = 25;
	long slrc = 3;
	long cwMax = pow(2, minBE + mkl);
	//	vector<double> traffic = {80,40,20,10}; // Sector wise traffic per node, traffic constant for all nodes within a sector
	//	vector<double> traffic = {160,80,40,20};
	//	vector<double> traffic = {200,100,50,25};
	//	vector<double> traffic = {300,150,100,50};
	//	vector<double> traffic = {400,200,100,50};
	//	vector<double> traffic = {500,250,125,80};
	//	vector<double> traffic = {320,320,320,320};
	//	vector<double> traffic = {160,160,160,160};
	vector<double> traffic = {100,400,300,100};
	long PH = 128;
	long MH = 272;
	long H = PH + MH;
	long rts = 160 + PH;
	long cts = 112 + PH;
	long ACK = 112 + PH;
	long BI = 1000; // BI in ms
	double beaconFrac = 1; // Fraction of interval in Beacon for CBAP
	long noOfSectors = 1;
	long pktSize = 7500;
	double simDuration = 1000; // In seconds
	double dr = 2000000000.0;

	long seed = time(0);
	rng.seed(seed);
	EdcaSimulationSetup edcaSetup = EdcaSimulationSetup(nodesPerSec, minBE,
			cwMax, slrc, simDuration, pktSize, rts, cts, ACK, H, dr, BI,
			beaconFrac, noOfSectors, traffic);

	for (long i = 0; i < iter; i++) {
		long seed = time(0);
		//		seed = 1564652224;
		rng.seed(seed);
		std::cout << "**************************  Stats of iteration: " << i
				<< " Seed (" << seed << ") *********************************** "
				<< std::endl;
		edcaSetup.run();
		edcaSetup.printStats(i);
	}
	edcaSetup.printExcelCompatFinalStats(iter);
	return 1;
}


