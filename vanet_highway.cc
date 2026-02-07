// NS-3.40 VANET highway DDoS simulation (enhanced)
// Two-way highway, 3 RSUs along Y, V2V + V2I, focused DDoS on RSU2.
// - U-turn + slowdown before turn + acceleration after turn
// - Attack packets tagged (for NetAnim metadata)
// - RSU2 turns ORANGE on crash (overload) then back GREEN on recovery

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/aodv-helper.h"
#include "ns3/netanim-module.h"

#include <fstream>
#include <iostream>
#include <random>
#include <set>
#include <string>
#include <vector>
#include <cmath>
#include <functional>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("VanetDdosHighwayNodesFixed");

// ---------------------------------------------------------
// RSU sink
// ---------------------------------------------------------
class RsuSink : public Application {
public:
    void Configure(Ipv4Address bindAddr, uint16_t port, const std::string &csv) {
        m_bindAddr = bindAddr; m_port = port; m_csv = csv;
    }
    void SetOverloadControl(bool autoCrash, double overloadPps, uint32_t consec, double recoverySeconds) {
        m_autoCrash = autoCrash; m_overloadPps = overloadPps; m_overloadConsec = consec; m_recoverySeconds = recoverySeconds;
    }
    void SetCrashCallback(std::function<void()> onCrash) { m_onCrash = std::move(onCrash); }
    void SetRecoverCallback(std::function<void()> onRecover) { m_onRecover = std::move(onRecover); }

    void StartApplication() override {
        if (!m_sock) {
            m_sock = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
            m_sock->Bind(InetSocketAddress(m_bindAddr, m_port));
            m_sock->SetRecvCallback(MakeCallback(&RsuSink::HandleRead, this));
        }
        m_enabled = true; m_isCrashed = false; m_windowCount = 0; m_consecOver = 0; m_eventCounter = 0;
        std::ofstream f(m_csv, std::ios::out);
        f << "event,time,src,dst,packetName,srcPort,dstPort,pktLen\n";
        f.close();
        ScheduleTick();
    }

    void StopApplication() override {
        m_enabled = false;
        if (m_sock) { m_sock->Close(); m_sock = nullptr; }
    }

    void ForceCrash() {
        if (!m_isCrashed) {
            m_isCrashed = true;
            if (m_sock) { m_sock->Close(); m_sock = nullptr; }
            if (m_onCrash) m_onCrash();
            if (m_recoverySeconds > 0) {
                Simulator::Schedule(Seconds(m_recoverySeconds), &RsuSink::ForceRecover, this);
            }
        }
    }

    void ForceRecover() {
        if (m_isCrashed) {
            if (!m_sock) {
                m_sock = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
                m_sock->Bind(InetSocketAddress(m_bindAddr, m_port));
                m_sock->SetRecvCallback(MakeCallback(&RsuSink::HandleRead, this));
            }
            m_isCrashed = false; m_consecOver = 0; m_windowCount = 0;
            if (m_onRecover) m_onRecover();
            if (m_enabled) ScheduleTick();
        }
    }

    void SetAttackLocalPort(uint16_t p) { m_attackLocalPort = p; }

private:
    void HandleRead(Ptr<Socket> s) {
        if (!m_enabled || m_isCrashed) {
            Ptr<Packet> p; Address from;
            while ((p = s->RecvFrom(from))) {}
            return;
        }
        Address from; Ptr<Packet> p;
        while ((p = s->RecvFrom(from))) {
            double t = Simulator::Now().GetSeconds();
            InetSocketAddress a = InetSocketAddress::ConvertFrom(from);
            uint16_t srcPort = a.GetPort();
            std::string pktName = (srcPort == m_attackLocalPort) ? "attack" : "normal";
            uint32_t plen = p->GetSize();
            ++m_eventCounter;
            std::ofstream f(m_csv, std::ios::app);
            f << m_eventCounter << "," << t << "," << a.GetIpv4() << "," << m_bindAddr
              << "," << pktName << "," << srcPort << "," << m_port << "," << plen << "\n";
            ++m_windowCount;
        }
    }

    void ScheduleTick() {
        if (m_enabled) Simulator::Schedule(Seconds(1.0), &RsuSink::Tick, this);
    }

    void Tick() {
        if (!m_enabled) return;
        double pps = m_windowCount; m_windowCount = 0;

        std::cout << "RSU " << m_bindAddr << " pps=" << pps
                  << " at t=" << Simulator::Now().GetSeconds() << "s\n";

        if (!m_isCrashed && m_autoCrash) {
            if (pps > m_overloadPps) ++m_consecOver; else m_consecOver = 0;
            if (m_consecOver >= m_overloadConsec) {
                NS_LOG_WARN("RSU overload -> crash");
                ForceCrash();
            }
        }
        if (m_enabled && !m_isCrashed) ScheduleTick();
    }

private:
    Ptr<Socket> m_sock{nullptr};
    Ipv4Address m_bindAddr; uint16_t m_port{0}; std::string m_csv{"rsu_received.csv"};
    bool m_enabled{false}, m_isCrashed{false};
    bool m_autoCrash{false}; double m_overloadPps{60.0}; uint32_t m_overloadConsec{2}; double m_recoverySeconds{10.0};
    uint32_t m_consecOver{0}, m_windowCount{0};
    uint64_t m_eventCounter{0};
    uint16_t m_attackLocalPort{9003};
    std::function<void()> m_onCrash, m_onRecover;
};

// ---------------------------------------------------------
// NORMAL VEHICLE (V2V + V2I)
// ---------------------------------------------------------
class NormalVehApp : public Application {
public:
    void Configure(Ptr<Node> self,
                   Ptr<UniformRandomVariable> startRv,
                   Ptr<UniformRandomVariable> intervalRv,
                   Ipv4Address rsu1, Ipv4Address rsu2, Ipv4Address rsu3,
                   uint16_t localPort, uint16_t dstPort,
                   uint32_t msgLen, double endTime) {
        m_self = self; m_rsu1 = rsu1; m_rsu2 = rsu2; m_rsu3 = rsu3;
        m_localPort = localPort; m_dstPort = dstPort; m_msgLen = msgLen; m_end = endTime;
        m_startRv = startRv; m_intervalRv = intervalRv; m_firstSent = false; m_enabled = true;
        m_intervalScale = 1.0;
        m_beaconPort = 9004;
        m_beaconLocalPort = 9005;
        m_beaconMsgLen = 80;
    }

    void StartApplication() override {
        if (!m_sockRsu) {
            m_sockRsu = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
            m_sockRsu->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_localPort));
        }
        if (!m_sockV2V) {
            m_sockV2V = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
            m_sockV2V->SetAllowBroadcast(true);
            m_sockV2V->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_beaconLocalPort));
        }
        m_enabled = true;
        Simulator::Schedule(Seconds(m_startRv->GetValue()), &NormalVehApp::SendOnce, this);
    }

    void StopApplication() override {
        m_enabled = false;
        if (m_sockRsu) { m_sockRsu->Close(); m_sockRsu = nullptr; }
        if (m_sockV2V) { m_sockV2V->Close(); m_sockV2V = nullptr; }
    }

    void Enable(bool en) {
        m_enabled = en;
        if (en) Simulator::Schedule(Seconds(0.01), &NormalVehApp::SendOnce, this);
    }

    void SetIntervalScale(double s) { m_intervalScale = s; }

private:
    void SendOnce() {
        if (!m_enabled || Simulator::Now().GetSeconds() >= m_end) return;

        // ----- V2I: send only to nearest RSU (handover style) -----
        Ipv4Address targetRsu = m_rsu2; // default middle

        Ptr<MobilityModel> mob = m_self->GetObject<MobilityModel>();
        if (mob) {
            Vector pos = mob->GetPosition();
            double y = pos.y;

            double mid12 = (100.0 + 200.0) / 2.0; // 150
            double mid23 = (200.0 + 300.0) / 2.0; // 250

            if (y < mid12) {
                targetRsu = m_rsu1;
            } else if (y < mid23) {
                targetRsu = m_rsu2;
            } else {
                targetRsu = m_rsu3;
            }
        }

        // V2I data
        m_sockRsu->SendTo(Create<Packet>(m_msgLen), 0,
                          InetSocketAddress(targetRsu, m_dstPort));

        // ----- V2V: broadcast beacon -----
        m_sockV2V->SendTo(Create<Packet>(m_beaconMsgLen), 0,
                          InetSocketAddress(Ipv4Address("255.255.255.255"), m_beaconPort));

        if (!m_firstSent) {
            m_firstSent = true;
            std::cout << "Normal node " << GetNode()->GetId()
                      << " first send at t=" << Simulator::Now().GetSeconds() << "\n";
        }

        double interval = m_intervalRv->GetValue() * m_intervalScale;
        if (interval < 0.05) interval = 0.05;
        Simulator::Schedule(Seconds(interval), &NormalVehApp::SendOnce, this);
    }

    Ptr<Node> m_self;
    Ptr<Socket> m_sockRsu{nullptr};
    Ptr<Socket> m_sockV2V{nullptr};

    Ipv4Address m_rsu1, m_rsu2, m_rsu3;
    uint16_t m_localPort{9001}, m_dstPort{9002};
    uint32_t m_msgLen{100}; double m_end{1e9};
    bool m_enabled{true}, m_firstSent{false};

    Ptr<UniformRandomVariable> m_startRv, m_intervalRv;
    double m_intervalScale{1.0};

    uint16_t m_beaconPort{9004};
    uint16_t m_beaconLocalPort{9005};
    uint32_t m_beaconMsgLen{80};
};

// ---------------------------------------------------------
// ATTACK TAG (for NetAnim packet metadata)
// ---------------------------------------------------------
class AttackTag : public Tag
{
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("AttackTag")
            .SetParent<Tag>()
            .AddConstructor<AttackTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override { return GetTypeId(); }
    uint32_t GetSerializedSize() const override { return 1; }

    void Serialize(TagBuffer i) const override {
        i.WriteU8(1); // mark attack
    }

    void Deserialize(TagBuffer i) override {
        uint8_t v = i.ReadU8(); (void)v;
    }

    void Print(std::ostream &os) const override {
        os << "ATTACK";
    }
};

// ---------------------------------------------------------
// ATTACKER
// ---------------------------------------------------------
class AttackerApp : public Application {
public:
    AttackerApp() : m_dst(Ipv4Address(),0), m_reported(false) {}

    void Configure(Ipv4Address rsu2, uint16_t port, uint16_t localPort,
                   double start, double end, double pps,
                   double tickInterval = 0.1, uint32_t msgLen = 100) {
        m_dst = InetSocketAddress(rsu2, port); m_s=start; m_e=end; m_pps=pps; m_tick=tickInterval;
        m_localPort=localPort; m_msgLen=msgLen;
        m_burstCount = std::max<uint32_t>(1,(uint32_t)std::lrint(m_pps*m_tick));
    }

    void StartApplication() override {
        if (!m_sock) {
            m_sock = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
            m_sock->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_localPort));
        }
        double delay = m_s - Simulator::Now().GetSeconds(); if (delay < 0.0) delay = 0.0;
        Simulator::Schedule(Seconds(delay), &AttackerApp::Loop, this);
    }

    void StopApplication() override {
        if (m_sock) { m_sock->Close(); m_sock = nullptr; }
    }

private:
    void Loop() {
        if (!m_sock || Simulator::Now().GetSeconds() > m_e) return;
        if (!m_reported) {
            m_reported = true;
            std::cout << "Attacker node " << GetNode()->GetId()
                      << " starts attack at t=" << Simulator::Now().GetSeconds() << "\n";
        }
        for (uint32_t i=0;i<m_burstCount;i++) {
            Ptr<Packet> p = Create<Packet>(m_msgLen);
            AttackTag tag;
            p->AddPacketTag(tag);      // mark as attack (visible in NetAnim metadata)
            m_sock->SendTo(p,0,m_dst);
        }
        Simulator::Schedule(Seconds(m_tick), &AttackerApp::Loop, this);
    }

    Ptr<Socket> m_sock{nullptr}; InetSocketAddress m_dst;
    double m_s{0}, m_e{0}, m_pps{0}, m_tick{0.1}; uint32_t m_burstCount{1};
    bool m_reported; uint16_t m_localPort{9003}; uint32_t m_msgLen{100};
};

// ---------------------------------------------------------
// helper: highway U-TURN motion with slowdown & accel
// ---------------------------------------------------------
static void
HighwayLoopStep(NodeContainer veh, double minY, double maxY, double interval)
{
    static bool initialized = false;
    static std::vector<double> baseSpeed;   // desired cruising |vy|

    uint32_t n = veh.GetN();
    if (!initialized) {
        baseSpeed.resize(n, 0.0);
        for (uint32_t i = 0; i < n; ++i) {
            Ptr<ConstantVelocityMobilityModel> mm =
                veh.Get(i)->GetObject<ConstantVelocityMobilityModel>();
            if (!mm) continue;
            Vector v = mm->GetVelocity();
            baseSpeed[i] = std::fabs(v.y);
        }
        initialized = true;
    }

    const double slowZone = 30.0;   
    const double turnZone = 5.0;    
    const double minSpeed = 3.0;    
    const double accelStep = 2.0;   
    const double decelStep = 3.0;   

    for (uint32_t i = 0; i < n; ++i) {
        Ptr<ConstantVelocityMobilityModel> mm =
            veh.Get(i)->GetObject<ConstantVelocityMobilityModel>();
        if (!mm) continue;

        Vector p = mm->GetPosition();
        Vector v = mm->GetVelocity();
        double vy = v.y;
        double speed = std::fabs(vy);
        double desired = (i < baseSpeed.size() && baseSpeed[i] > 0.0) ? baseSpeed[i] : speed;

        // Northbound (vy > 0) near maxY
        if (vy > 0.0) {
            double distToEnd = maxY - p.y;
            if (distToEnd < slowZone && distToEnd > turnZone) {
                // slow down
                if (speed > minSpeed) {
                    double newSpeed = speed - decelStep;
                    if (newSpeed < minSpeed) newSpeed = minSpeed;
                    vy = (vy > 0 ? +newSpeed : -newSpeed);
                }
            } else if (distToEnd <= turnZone) {
                // perform U-turn: flip direction and start slowly
                vy = -std::max(minSpeed, 5.0);
            } else if (distToEnd > slowZone && vy < 0.0) {
                // far after turn? (safety)
                vy = +desired;
            }
        }
        // Southbound (vy < 0) near minY
        else if (vy < 0.0) {
            double distToEnd = p.y - minY;
            if (distToEnd < slowZone && distToEnd > turnZone) {
                if (speed > minSpeed) {
                    double newSpeed = speed - decelStep;
                    if (newSpeed < minSpeed) newSpeed = minSpeed;
                    vy = (vy > 0 ? +newSpeed : -newSpeed);
                }
            } else if (distToEnd <= turnZone) {
                // U-turn at lower end
                vy = +std::max(minSpeed, 5.0);
            } else if (distToEnd > slowZone && vy > 0.0) {
                vy = -desired;
            }
        }

        // After U-turn: accelerate gradually back to cruising speed
        if (vy > 0.0) {
            double speedUp = std::fabs(vy);
            if (speedUp < desired) {
                double newSpeed = speedUp + accelStep;
                if (newSpeed > desired) newSpeed = desired;
                vy = +newSpeed;
            }
        } else if (vy < 0.0) {
            double speedUp = std::fabs(vy);
            if (speedUp < desired) {
                double newSpeed = speedUp + accelStep;
                if (newSpeed > desired) newSpeed = desired;
                vy = -newSpeed;
            }
        }

        // keep X as is, just update Y velocity
        v.y = vy;
        mm->SetVelocity(v);
    }

    Simulator::Schedule(Seconds(interval), &HighwayLoopStep, veh, minY, maxY, interval);
}

// ---------------------------------------------------------
// MAIN
// ---------------------------------------------------------
int main(int argc, char* argv[]) {
    uint32_t N=40;
    double runTime=60.0;
    uint16_t normalLocalPort=9001, normalDstPort=9002; uint32_t normalMsgLen=100;
    uint16_t attackLocalPort=9003, attackDstPort=9002;
    double attackStart=20.0, attackEnd=40.0, attackTickInterval=0.1;
    double perAttackerPps=150.0;
    double attackRateAgg=0.0;
    bool autoCrash=false; double overloadPps=60.0; uint32_t overloadConsec=2; double recoverySeconds=10.0;
    bool packetMetadata = false;
    uint64_t maxPktsTrace = 20000000;
    double flashFactor = 0.3;

    CommandLine cmd;
    cmd.AddValue("N","number of vehicles",N);
    cmd.AddValue("runTime","simulation seconds",runTime);
    cmd.AddValue("attackStart","attack start",attackStart);
    cmd.AddValue("attackEnd","attack end",attackEnd);
    cmd.AddValue("autoCrash","enable RSU auto-crash (1/0)",autoCrash);
    cmd.AddValue("overloadPps","overload threshold pps",overloadPps);
    cmd.AddValue("overloadConsec","overload consecutive seconds",overloadConsec);
    cmd.AddValue("recoverySeconds","auto recovery seconds after crash",recoverySeconds);
    cmd.AddValue("attackRate","aggregate attack rate (pps) to be divided across attackers (optional)",attackRateAgg);
    cmd.AddValue("packetMetadata","enable NetAnim packet metadata (0/1)",packetMetadata);
    cmd.AddValue("flashFactor","multiplier for normal interval during attack phase (<1 = faster)",flashFactor);
    cmd.Parse(argc, argv);

    NodeContainer veh; veh.Create(N);
    NodeContainer rsu; rsu.Create(3);

    // RSUs vertical on Y axis, centered in X
    MobilityHelper mobRsu;
    mobRsu.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobRsu.Install(rsu);

    const double RSU_X  = 200.0;
    const double RSU_DY = 100.0;
    const double CRASH_RADIUS_Y = 40.0;   

    rsu.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(RSU_X, 1*RSU_DY, 0.0));
    rsu.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(RSU_X, 2*RSU_DY, 0.0));
    rsu.Get(2)->GetObject<MobilityModel>()->SetPosition(Vector(RSU_X, 3*RSU_DY, 0.0));

    // Vehicles: two-way highway
    MobilityHelper mobVeh;
    mobVeh.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobVeh.Install(veh);
    
    std::mt19937 rng(12345);

    const double minY    = 60.0;
    const double maxY    = 340.0;
    const double roadLen = maxY - minY;

    const double laneOffsetInner = 20.0;
    const double laneOffsetOuter = 35.0;

    uint32_t northCount = (N + 1) / 2;
    uint32_t southCount = N - northCount;
    
    std::uniform_real_distribution<double> speedFast(27.0, 33.0);
    std::uniform_real_distribution<double> speedMedium(20.0, 26.0);
    std::uniform_real_distribution<double> speedSlow(14.0, 19.0);
    std::uniform_real_distribution<double> yJitter(-2.0, 2.0);

    double spacingNorth = roadLen / std::max(1u, northCount);
    double spacingSouth = roadLen / std::max(1u, southCount);

    uint32_t northIdx = 0;
    uint32_t southIdx = 0;

    for (uint32_t i = 0; i < N; ++i)
    {
        auto mm = veh.Get(i)->GetObject<ConstantVelocityMobilityModel>();
        if (!mm) continue;

        bool northbound = (i % 2 == 0);
        double vy       = 0.0;
        double yBase    = 0.0;

        bool innerLane = false;
        if (northbound) {
            innerLane = (northIdx % 2 == 0);
        } else {
            innerLane = (southIdx % 2 == 0);
        }

        double xSign  = northbound ? +1.0 : -1.0;
        double xLocal = innerLane ? laneOffsetInner : laneOffsetOuter;
        double x      = RSU_X + xSign * xLocal;

        if (northbound) {
            yBase = minY + (northIdx + 0.5) * spacingNorth;
            yBase += yJitter(rng);
            if (yBase < minY) yBase = minY + 1.0;
            if (yBase > maxY) yBase = maxY - 1.0;

            double speed = innerLane ? speedFast(rng) : speedMedium(rng);
            vy = speed;
            ++northIdx;
        } else {
            yBase = maxY - (southIdx + 0.5) * spacingSouth;
            yBase += yJitter(rng);
            if (yBase < minY) yBase = minY + 1.0;
            if (yBase > maxY) yBase = maxY - 1.0;

            double speed = innerLane ? speedMedium(rng) : speedSlow(rng);
            vy = -speed;
            ++southIdx;
        }

        mm->SetPosition(Vector(x, yBase, 0.0));
        mm->SetVelocity(Vector(0.0, vy, 0.0));
    }

    // 802.11p
    YansWifiChannelHelper chan = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy; phy.SetChannel(chan.Create());
    WifiHelper wifi; wifi.SetStandard(WIFI_STANDARD_80211p);
    WifiMacHelper mac; mac.SetType("ns3::AdhocWifiMac");
    NodeContainer all; all.Add(veh); all.Add(rsu);
    NetDeviceContainer devs = wifi.Install(phy, mac, all);

    // Internet + AODV
    AodvHelper aodv;
    InternetStackHelper stack; stack.SetRoutingHelper(aodv);
    stack.Install(all);

    Ipv4AddressHelper addr;
    addr.SetBase("10.1.0.0","255.255.0.0");
    Ipv4InterfaceContainer ifs = addr.Assign(devs);

    // RSU addresses and sinks
    const uint32_t idxRSU1 = N, idxRSU2 = N+1, idxRSU3 = N+2;
    Ipv4Address ipRSU1 = ifs.GetAddress(idxRSU1);
    Ipv4Address ipRSU2 = ifs.GetAddress(idxRSU2);
    Ipv4Address ipRSU3 = ifs.GetAddress(idxRSU3);
    const uint16_t rsuPort = normalDstPort;

    Ptr<RsuSink> sink1 = CreateObject<RsuSink>();
    rsu.Get(0)->AddApplication(sink1);
    sink1->Configure(ipRSU1, rsuPort, "rsu1_received.csv");
    sink1->SetOverloadControl(false, 0.0, 0, 0.0);
    sink1->SetAttackLocalPort(attackLocalPort);
    sink1->SetStartTime(Seconds(0.0));
    sink1->SetStopTime(Seconds(runTime+2));

    Ptr<RsuSink> sink2 = CreateObject<RsuSink>();
    rsu.Get(1)->AddApplication(sink2);
    sink2->Configure(ipRSU2, rsuPort, "rsu2_received.csv");
    sink2->SetOverloadControl(autoCrash, overloadPps, overloadConsec, recoverySeconds);
    sink2->SetAttackLocalPort(attackLocalPort);
    sink2->SetStartTime(Seconds(0.0));
    sink2->SetStopTime(Seconds(runTime+2));

    Ptr<RsuSink> sink3 = CreateObject<RsuSink>();
    rsu.Get(2)->AddApplication(sink3);
    sink3->Configure(ipRSU3, rsuPort, "rsu3_received.csv");
    sink3->SetOverloadControl(false, 0.0, 0, 0.0);
    sink3->SetAttackLocalPort(attackLocalPort);
    sink3->SetStartTime(Seconds(0.0));
    sink3->SetStopTime(Seconds(runTime+2));

    Ptr<UniformRandomVariable> startRv = CreateObject<UniformRandomVariable>();
    startRv->SetAttribute("Min", DoubleValue(0.0));
    startRv->SetAttribute("Max", DoubleValue(5.0));

    Ptr<UniformRandomVariable> intervalRv = CreateObject<UniformRandomVariable>();
    intervalRv->SetAttribute("Min", DoubleValue(0.5));
    intervalRv->SetAttribute("Max", DoubleValue(1.5));

    std::vector<Ptr<NormalVehApp>> vapps(N);
    for (uint32_t i=0;i<N;++i) {
        auto app = CreateObject<NormalVehApp>();
        app->Configure(veh.Get(i), startRv, intervalRv,
                       ipRSU1, ipRSU2, ipRSU3,
                       normalLocalPort, normalDstPort,
                       normalMsgLen, runTime+1);
        veh.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.1));
        app->SetStopTime(Seconds(runTime+2));
        vapps[i] = app;
    }

    std::vector<uint32_t> attackers;
    if (N > 0)  attackers.push_back(0u);
    if (N > 30) attackers.push_back(30u);
    if (N > 59) attackers.push_back(59u);
    if (attackers.size() < 3 && N >= 3) {
        std::set<uint32_t> s(attackers.begin(), attackers.end());
        s.insert(N/2);
        s.insert(N-1);
        attackers.assign(s.begin(), s.end());
        if (attackers.size() > 3) attackers.resize(3);
    }

    if (attackRateAgg > 0.0) {
        perAttackerPps = attackRateAgg / std::max<size_t>(1, attackers.size());
    }

    for (auto idx : attackers) {
        if (idx >= N) continue;
        auto a = CreateObject<AttackerApp>();
        a->Configure(ipRSU2, attackDstPort, attackLocalPort,
                     attackStart, attackEnd,
                     perAttackerPps, attackTickInterval, 100);
        veh.Get(idx)->AddApplication(a);
        a->SetStartTime(Seconds(0.1));
        a->SetStopTime(Seconds(runTime+2));
    }
    std::cout << "Attackers (fixed) = ";
    for (auto i : attackers) std::cout << i << " ";
    std::cout << " perAttackerPps=" << perAttackerPps << "\n";

    std::set<uint32_t> attackerSet(attackers.begin(), attackers.end());
    std::vector<uint32_t> crashers;
    for (uint32_t i=0;i<N;++i) {
        if (attackerSet.count(i)) continue;
        if (i >= N/3 && i < 2*(N/3)) crashers.push_back(i);
    }
    if (crashers.size() > 10) crashers.resize(10);

    std::vector<Vector> savedVel(N);
    for (auto i : crashers) {
        if (i < N) {
            auto mm = veh.Get(i)->GetObject<ConstantVelocityMobilityModel>();
            if (mm) savedVel[i] = mm->GetVelocity();
        }
    }

    const std::string xmlFile = "vanet_highway.xml";
    AnimationInterface anim(xmlFile);
    anim.SetMobilityPollInterval(Seconds(0.25));
    anim.SetMaxPktsPerTraceFile(maxPktsTrace);
    if (packetMetadata) anim.EnablePacketMetadata(true); else anim.EnablePacketMetadata(false);

    // Node visuals
    for (uint32_t i=0;i<all.GetN();++i) {
        anim.UpdateNodeSize(i, 4.0, 4.0);
        if (i < N) {
            auto mm = veh.Get(i)->GetObject<MobilityModel>();
            double y = mm ? mm->GetPosition().y : 0.0;
            bool isAtt = attackerSet.count(i);
            if (isAtt) {
                anim.UpdateNodeColor(i, 200,0,200);   // attackers
                anim.UpdateNodeSize(i, 6.0, 6.0);
            } else if (y < 1.5*RSU_DY) {
                anim.UpdateNodeColor(i, 160,160,160);
            } else if (y < 2.5*RSU_DY) {
                anim.UpdateNodeColor(i, 255,128,0);
            } else {
                anim.UpdateNodeColor(i, 0,120,255);
            }
        } else {
            // RSUs: all GREEN initially
            anim.UpdateNodeColor(i, 0,200,0);
            anim.UpdateNodeSize(i, 10.0, 10.0);
        }
    }

    Simulator::Schedule(Seconds(0.2), [](){ std::cout << "Phase A: normal traffic\n"; });
    Simulator::Schedule(Seconds(attackStart), [](){ std::cout << "Phase B: ATTACK begins\n"; });
    double downStart = attackEnd;
    double downEnd   = downStart + 10.0;
    Simulator::Schedule(Seconds(downStart), [](){ std::cout << "Phase C: RSU2 DOWN + accidents\n"; });
    Simulator::Schedule(Seconds(downEnd),   [](){ std::cout << "Phase D: recovery\n"; });

    Simulator::Schedule(Seconds(attackStart), [&]() {
        std::cout << "Flash crowd ON at t=" << Simulator::Now().GetSeconds()
                  << " (factor=" << flashFactor << ")\n";
        for (uint32_t j=0;j<N;++j) {
            if (vapps[j]) vapps[j]->SetIntervalScale(flashFactor);
        }
    });
    Simulator::Schedule(Seconds(attackEnd), [&]() {
        std::cout << "Flash crowd OFF at t=" << Simulator::Now().GetSeconds() << "\n";
        for (uint32_t j=0;j<N;++j) {
            if (vapps[j]) vapps[j]->SetIntervalScale(1.0);
        }
    });

    if (!autoCrash) {
        Simulator::Schedule(Seconds(downStart), [sink2]() { sink2->ForceCrash(); });
        Simulator::Schedule(Seconds(downEnd),   [sink2]() { sink2->ForceRecover(); });
    }

    // Crash & recovery callbacks (RSU2 ORANGE on crash, GREEN on recover)
    sink2->SetCrashCallback([&]() {
    std::cout << "⚠️ RSU2 crashed at t=" << Simulator::Now().GetSeconds() << "s\n";

    anim.UpdateNodeColor(idxRSU2, 255, 0, 0);   
    crashers.clear();

    double rsu2Y = 2 * RSU_DY;  

    for (uint32_t i = 0; i < N; ++i) {
        if (attackerSet.count(i)) continue;  

        Ptr<MobilityModel> mm2 = veh.Get(i)->GetObject<MobilityModel>();
        if (!mm2) continue;

        double y = mm2->GetPosition().y;
        double dy = std::fabs(y - rsu2Y);

        if (dy <= CRASH_RADIUS_Y) {
            crashers.push_back(i);   
        }
    }


    for (auto i : crashers) {
        if (i < N) {
            anim.UpdateNodeColor(i, 0, 0, 0);      
            anim.UpdateNodeSize(i, 6.0, 6.0);

            if (vapps[i]) vapps[i]->Enable(false); 

            auto mm = veh.Get(i)->GetObject<ConstantVelocityMobilityModel>();
            if (mm) {
                savedVel[i] = mm->GetVelocity();   
                mm->SetVelocity(Vector(0.0, 0.0, 0.0));  
            }
        }
    }
});


    sink2->SetRecoverCallback([&]() {
    std::cout << "ℹ️ RSU2 recovered at t=" << Simulator::Now().GetSeconds() << "s\n";
    
    anim.UpdateNodeColor(idxRSU2, 0, 200, 0);
    
    for (auto i : crashers) {
        if (i < N) {
            auto mm2 = veh.Get(i)->GetObject<MobilityModel>();
            double y = mm2 ? mm2->GetPosition().y : 0.0;

            if (y < 1.5*RSU_DY)          anim.UpdateNodeColor(i,160,160,160);
            else if (y < 2.5*RSU_DY)     anim.UpdateNodeColor(i,255,128,0);
            else                         anim.UpdateNodeColor(i,0,120,255);
            anim.UpdateNodeSize(i,4.5,4.5);

            if (vapps[i]) vapps[i]->Enable(true);   
            auto mm = veh.Get(i)->GetObject<ConstantVelocityMobilityModel>();
            if (mm) {
                mm->SetVelocity(savedVel[i]);       
            }
        }
    }
});

    // U-turn motion loop
    HighwayLoopStep(veh, minY, maxY, 0.5);

    phy.EnablePcapAll("vanet_highway");

    Simulator::Stop(Seconds(runTime + 2.0));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "Wrote NetAnim XML: " << xmlFile << "\n";
    return 0;
}

