#include <omnetpp.h>
#include <deque>
#include <vector>
using namespace omnetpp;

class CarSource : public cSimpleModule {
  private:
    cMessage *sendEvt = nullptr;
    double     mean;

  protected:
    virtual void initialize() override {
        mean    = par("interArrivalMean").doubleValue();
        sendEvt = new cMessage("sendEvt");
        scheduleAt(simTime() + exponential(mean), sendEvt);
        EV << getFullPath() << ": interArrivalMean = "
           << par("interArrivalMean") << "s\n";

    }

    virtual void handleMessage(cMessage *msg) override {

        auto *car = new cMessage("car");
        send(car, "out");


        scheduleAt(simTime() + exponential(mean), sendEvt);
    }

    virtual void finish() override {
        cancelAndDelete(sendEvt);
    }
};

Define_Module(CarSource);
#ifndef TRAFFICLIGHTCONTROLLER_H_
#define TRAFFICLIGHTCONTROLLER_H_


enum CtrlKind { GREEN_START = 1, GREEN_END = 2 };

class TrafficLightController : public cSimpleModule {
  private:
    enum Phase { NS_LEFT, NS_STRAIGHT, EW_LEFT, EW_STRAIGHT };
    Phase        curPhase;
    simtime_t    durations[4];
    cMessage    *switchEvt = nullptr;
    std::deque<cMessage*> queues[12];

    void advancePhase();
    void releaseGreen();

  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
};

Define_Module(TrafficLightController);

#endif
void TrafficLightController::initialize() {
    durations[NS_LEFT]     = par("durNSleft").doubleValue();
    durations[NS_STRAIGHT] = par("durNSstraight").doubleValue();
    durations[EW_LEFT]     = par("durEWleft").doubleValue();
    durations[EW_STRAIGHT] = par("durEWstraight").doubleValue();

    curPhase = NS_LEFT;
    switchEvt = new cMessage("phaseSwitch");
    releaseGreen();
    scheduleAt(simTime() + durations[curPhase], switchEvt);
    EV << "Left‑turn green = " << durations[NS_LEFT] << " s, straight green = "
       << durations[NS_STRAIGHT] << " s\n";
}

#include <algorithm>

void TrafficLightController::handleMessage(cMessage *msg) {
    if (msg->isSelfMessage() && msg->getKind()==GREEN_END) {
        int idx = (int)(intptr_t)msg->getContextPointer();
        send(msg, gate("ctrlOut", idx));
        return;
    }
    if (msg == switchEvt) {
        advancePhase();
        return;
    }
    int idx = msg->getArrivalGate()->getIndex();
    msg->setTimestamp(simTime());
    static const std::vector<std::vector<int>> ALLOWED = {
        {0,6},        // NS_LEFT
        {1,2,7,8},    // NS_STRAIGHT
        {3,9},        // EW_LEFT
        {4,5,10,11}   // EW_STRAIGHT
    };
    const auto &allowedThisPhase = ALLOWED[(int)curPhase];
    if (std::find(allowedThisPhase.begin(),allowedThisPhase.end(),idx)!= allowedThisPhase.end())
    {
        send(msg, gate("out", idx));
    }
    else {
        queues[idx].push_back(msg);
    }
}


void TrafficLightController::advancePhase() {

    curPhase = Phase((curPhase + 1) % 4);
    releaseGreen();
    scheduleAt(simTime() + durations[curPhase], switchEvt);
}

void TrafficLightController::releaseGreen() {
    static const std::vector<std::vector<int>> ALLOWED = {
        {0,6},     // NS_LEFT
        {1,2,7,8}, // NS_STRAIGHT
        {3,9},     // EW_LEFT
        {4,5,10,11} // EW_STRAIGHT
    };
    const auto &allowed = ALLOWED[(int)curPhase];

    for (int idx : allowed) {
        cMessage *on = new cMessage("ctrl", GREEN_START);
        on->setKind(GREEN_START);
        send(on, gate("ctrlOut", idx));

        cMessage *off = new cMessage("ctrl", GREEN_END);
        off->setKind(GREEN_END);
        off->setContextPointer((void*)(intptr_t)idx);
        scheduleAt(simTime() + durations[curPhase], off);
    }

    for (int idx : allowed) {
        auto &q = queues[idx];
        while (!q.empty()) {
            cMessage *car = q.front();
            q.pop_front();
            send(car, gate("out", idx));
        }
    }
}
class CarSink : public cSimpleModule {
  private:
    cOutVector delayVec;
    simtime_t  totalDelay = SIMTIME_ZERO;
    long       numCars    = 0;

    static simtime_t globalTotalDelay;
    static long      globalNumCars;

  protected:
    virtual void initialize() override {
        delayVec.setName("delay");
    }

    virtual void handleMessage(cMessage *msg) override {
        simtime_t d = simTime() - msg->getTimestamp();

        delayVec.record(d);
        totalDelay += d;
        numCars++;

        globalTotalDelay += d;
        globalNumCars++;

        delete msg;
    }

    virtual void finish() override {
        simtime_t myAvg = numCars>0 ? totalDelay/numCars : SIMTIME_ZERO;
        EV << "Average delay at " << getFullPath()
           << " = " << myAvg << "s over " << numCars << " cars\n";
        recordScalar("averageDelay", myAvg.dbl());

        if (getIndex() == 0) {
            simtime_t overallAvg = globalNumCars>0
                                   ? globalTotalDelay/globalNumCars
                                   : SIMTIME_ZERO;
            EV << ">>> Overall avg delay = " << overallAvg
               << "s over " << globalNumCars << " cars\n";
            recordScalar("overallAverageDelay", overallAvg.dbl());
        }
    }
};

simtime_t CarSink::globalTotalDelay = SIMTIME_ZERO;
long      CarSink::globalNumCars    = 0;

Define_Module(CarSink);

#include <queue>

class Intersection : public cSimpleModule {
  private:
    cMessage        *endService = nullptr;
    std::queue<cMessage*> q;
    simtime_t        serviceTime;
    bool             isGreen = false;

  protected:
    virtual void initialize() override {
        serviceTime = par("serviceTime").doubleValue();
        endService  = new cMessage("endService");
    }

    virtual void handleMessage(cMessage *msg) override {
        if (msg->isSelfMessage()) {
            cMessage *car = (cMessage*)msg->getContextPointer();
            send(car, "out");
            if (isGreen && !q.empty()) {
                cMessage *next = q.front(); q.pop();
                endService->setContextPointer(next);
                scheduleAt(simTime() + serviceTime, endService);
            }
        }
        else if (msg->getKind()==GREEN_START || msg->getKind()==GREEN_END) {
            isGreen = (msg->getKind()==GREEN_START);
            delete msg;
            if (isGreen && !endService->isScheduled() && !q.empty()) {
                cMessage *next = q.front(); q.pop();
                endService->setContextPointer(next);
                scheduleAt(simTime() + serviceTime, endService);
            }
        }
        else {
            q.push(msg);
            if (isGreen && !endService->isScheduled()) {
                cMessage *next = q.front(); q.pop();
                endService->setContextPointer(next);
                scheduleAt(simTime() + serviceTime, endService);
            }
        }
    }

    virtual void finish() override {
        cancelAndDelete(endService);
        while (!q.empty()) { delete q.front(); q.pop(); }
    }
};

Define_Module(Intersection);
