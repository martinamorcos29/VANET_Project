// scratch/vanet_ddos_sumo.cc
// NS-3.40 VANET DDoS simulation with LIVE SUMO mobility via UDP bridge (TraCI->Python->NS-3)
// Vehicles move based on SUMO positions streamed over UDP (port 9999).
// Three RSUs along the road, focused DDoS on RSU2.

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/aodv-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/udp-socket-factory.h"

#include <fstream>
#include <iostream>
#include <random>
#include <set>
#include <string>
#include <vector>
#include <cmath>
#include <functional>
#include <unordered_map>
#include <sstream>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("VanetDdosSumo");

// ------------------- LIVE SUMO -> NS-3 mapping (UDP) -------------------
// Message format from Python:
//   t;vehId,x,y;vehId,x,y;...
static std::unordered_map<std::string, uint32_t> gVidToNode;
static uint32_t gNextNode = 0;
static NodeContainer gVeh;

static void
HandleSumsPositions(Ptr<Socket> sock)
{
    Address from;
    Ptr<Packet> pkt = sock->RecvFrom(from);
    if (!pkt) return;

    std::string data;
    data.resize(pkt->GetSize());
    pkt->CopyData(reinterpret_cast<uint8_t*>(&data[0]), data.size());

    std::stringstream ss(data);
    std::string token;

    // first token = time (ignored)
    std::getline(ss, token, ';');

    while (std::getline(ss, token, ';'))
    {
        // token = "id,x,y"
        std::stringstream ts(token);
        std::string vid, sx, sy;
        if (!std::getline(ts, vid, ',')) continue;
        if (!std::getline(ts, sx, ',')) continue;
        if (!std::getline(ts, sy, ',')) continue;

        double x = 0.0, y = 0.0;
        try {
            x = std::stod(sx);
            y = std::stod(sy);
        } catch (...) {
            continue;
        }

        auto it = gVidToNode.find(vid);
        if (it == gVidToNode.end())
        {
            if (gNextNode >= gVeh.GetN()) continue; // ignore extra vehicles
            gVidToNode[vid] = gNextNode;
            it = gVidToNode.find(vid);
            std::cout << "[MAP] " << vid << " -> node " << it->second << "\n";
            gNextNode++;
        }

        uint32_t nid = it->second;
        Ptr<MobilityModel> mm = gVeh.Get(nid)->GetObject<MobilityModel>();
        if (mm)
        {
            mm->SetPosition(Vector(x, y, 0.0));
        }
    }
}

// ---------- DEBUG: print some vehicle positions every 5s ----------
static void
PrintSomePositions(NodeContainer veh)
{
    std::vector<uint32_t> ids = {0, 10, 20, 30, 40, 50};

    std::cout << "== DEBUG positions at t=" << Simulator::Now().GetSeconds() << "s ==\n";
    for (uint32_t id : ids)
    {
        if (id >= veh.GetN()) continue;
        Ptr<MobilityModel> m = veh.Get(id)->GetObject<MobilityModel>();
        if (!m) continue;
        Vector p = m->GetPosition();
        std::cout << " node " << id << " : (" << p.x << ", " << p.y << ")\n";
    }
    std::cout << "========================================\n";

    Simulator::Schedule(Seconds(5.0), &PrintSomePositions, veh);
}

// ---------- RSU sink ----------
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
    void SetAttackLocalPort(uint16_t p) { m_attackLocalPort = p; }

    void StartApplication() override {
        if (!m_sock) {
            m_sock = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
            m_sock->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_port));
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
                m_sock->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_port));
                m_sock->SetRecvCallback(MakeCallback(&RsuSink::HandleRead, this));
            }
            m_isCrashed = false; m_consecOver = 0; m_windowCount = 0;
            if (m_onRecover) m_onRecover();
            if (m_enabled) ScheduleTick();
        }
    }

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

    bool m_autoCrash{false};
    double m_overloadPps{60.0};
    uint32_t m_overloadConsec{2};
    double m_recoverySeconds{10.0};

    uint32_t m_consecOver{0}, m_windowCount{0};
    uint64_t m_eventCounter{0};
    uint16_t m_attackLocalPort{9003};

    std::function<void()> m_onCrash, m_onRecover;
};

// ---------- Normal Vehicle (V2V + V2I) ----------
class NormalVehApp : public Application {
public:
    void Configure(Ptr<Node> self,
                   Ptr<UniformRandomVariable> startRv,
                   Ptr<UniformRandomVariable> intervalRv,
                   Ipv4Address rsu1, Ipv4Address rsu2, Ipv4Address rsu3,
                   uint16_t localPort, uint16_t dstPort,
                   uint32_t msgLen, double endTime,
                   double rsu1x, double rsu2x, double rsu3x) {
        m_self = self; m_rsu1 = rsu1; m_rsu2 = rsu2; m_rsu3 = rsu3;
        m_localPort = localPort; m_dstPort = dstPort; m_msgLen = msgLen; m_end = endTime;
        m_startRv = startRv; m_intervalRv = intervalRv;
        m_firstSent = false; m_enabled = true; m_intervalScale = 1.0;

        m_beaconPort = 9004;
        m_beaconLocalPort = 9005;
        m_beaconMsgLen = 80;

        m_mid12 = (rsu1x + rsu2x) / 2.0;
        m_mid23 = (rsu2x + rsu3x) / 2.0;
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

        Ipv4Address targetRsu = m_rsu2;

        Ptr<MobilityModel> mob = m_self->GetObject<MobilityModel>();
        if (mob) {
            double x = mob->GetPosition().x;
            if (x < m_mid12) targetRsu = m_rsu1;
            else if (x < m_mid23) targetRsu = m_rsu2;
            else targetRsu = m_rsu3;
        }

        // V2I unicast to nearest RSU
        m_sockRsu->SendTo(Create<Packet>(m_msgLen), 0, InetSocketAddress(targetRsu, m_dstPort));

        // V2V beacon broadcast
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

    double m_mid12{0.0}, m_mid23{0.0};
};

// ---------- Attacker ----------
class AttackerApp : public Application {
public:
    void Configure(Ipv4Address victimRsuIp, uint16_t victimPort, uint16_t localPort,
                   double start, double end, double pps,
                   double tickInterval = 0.1, uint32_t msgLen = 100)
    {
        m_victimIp = victimRsuIp;
        m_victimPort = victimPort;
        m_localPort = localPort;
        m_s = start; m_e = end; m_pps = pps; m_tick = tickInterval;
        m_msgLen = msgLen;
        m_burstCount = std::max<uint32_t>(1, (uint32_t)std::lrint(m_pps * m_tick));
    }

    void StartApplication() override {
        if (!m_sock) {
            m_sock = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
            m_sock->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_localPort));
        }
        m_dst = InetSocketAddress(m_victimIp, m_victimPort);
        m_dstReady = true;


        double delay = m_s - Simulator::Now().GetSeconds();
        if (delay < 0.0) delay = 0.0;
        Simulator::Schedule(Seconds(delay), &AttackerApp::Loop, this);
    }

    void StopApplication() override {
        if (m_sock) { m_sock->Close(); m_sock = nullptr; }
    }

private:
    void Loop() {
        if (!m_sock || !m_dstReady || Simulator::Now().GetSeconds() > m_e) return;
        if (!m_reported) {
            m_reported = true;
            std::cout << "Attacker node " << GetNode()->GetId()
                      << " starts attack at t=" << Simulator::Now().GetSeconds() << "\n";
        }
        for (uint32_t i = 0; i < m_burstCount; i++) {
            m_sock->SendTo(Create<Packet>(m_msgLen), 0, m_dst);
        }
        Simulator::Schedule(Seconds(m_tick), &AttackerApp::Loop, this);
    }

    Ptr<Socket> m_sock{nullptr};
    Address m_dst;
    bool m_dstReady{false};


    Ipv4Address m_victimIp;
    uint16_t m_victimPort{0};
    uint16_t m_localPort{9003};

    double m_s{0}, m_e{0}, m_pps{0}, m_tick{0.1};
    uint32_t m_burstCount{1};
    uint32_t m_msgLen{100};
    bool m_reported{false};
};

// ---------- MAIN ----------
int main(int argc, char* argv[])
{
    uint32_t N = 60;
    double runTime = 200.0;

    uint16_t normalLocalPort = 9001, normalDstPort = 9002;
    uint32_t normalMsgLen = 100;

    uint16_t attackLocalPort = 9003, attackDstPort = 9002;
    double attackStart = 60.0, attackEnd = 90.0, attackTickInterval = 0.1;
    double perAttackerPps = 150.0;
    double attackRateAgg = 0.0;

    bool autoCrash = false;
    double overloadPps = 60.0;
    uint32_t overloadConsec = 2;
    double recoverySeconds = 10.0;

    bool packetMetadata = false;
    uint64_t maxPktsTrace = 20000000;
    double flashFactor = 0.3;

    CommandLine cmd;
    cmd.AddValue("N", "number of vehicles", N);
    cmd.AddValue("runTime", "simulation seconds", runTime);
    cmd.AddValue("attackStart", "attack start", attackStart);
    cmd.AddValue("attackEnd", "attack end", attackEnd);
    cmd.AddValue("autoCrash", "enable RSU auto-crash (1/0)", autoCrash);
    cmd.AddValue("overloadPps", "overload threshold pps", overloadPps);
    cmd.AddValue("overloadConsec", "overload consecutive seconds", overloadConsec);
    cmd.AddValue("recoverySeconds", "auto recovery seconds after crash", recoverySeconds);
    cmd.AddValue("attackRate", "aggregate attack rate (pps)", attackRateAgg);
    cmd.AddValue("packetMetadata", "enable NetAnim packet metadata (0/1)", packetMetadata);
    cmd.AddValue("flashFactor", "multiplier for normal interval during attack phase", flashFactor);
    cmd.Parse(argc, argv);

    // Vehicles and RSUs
    NodeContainer veh; veh.Create(N);
    NodeContainer rsu; rsu.Create(3);
    gVeh = veh;

    // LIVE mobility: start with constant position model (updated by UDP callback)
    MobilityHelper mh;
    mh.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mh.Install(veh);

    // Debug positions
    Simulator::Schedule(Seconds(1.0), &PrintSomePositions, veh);

    // RSUs fixed positions (IMPORTANT: keep near vehicle coordinate system!)
    MobilityHelper mobRsu;
    mobRsu.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobRsu.Install(rsu);

    // Since vehicle coordinates seem around x~3700, y~3400 (from your trace),
    // place RSUs around that range to avoid NetAnim zoom-out.
    const double RSU_Y = 3427.0;
    const double RSU1_X = 3600.0;
    const double RSU2_X = 3740.0;
    const double RSU3_X = 3880.0;

    rsu.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(RSU1_X, RSU_Y, 0.0));
    rsu.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(RSU2_X, RSU_Y, 0.0));
    rsu.Get(2)->GetObject<MobilityModel>()->SetPosition(Vector(RSU3_X, RSU_Y, 0.0));

    // 802.11p
    YansWifiChannelHelper chan = YansWifiChannelHelper::Default();
    chan.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(500.0));
    YansWifiPhyHelper phy;
    phy.SetChannel(chan.Create());

    WifiHelper wifi; wifi.SetStandard(WIFI_STANDARD_80211p);
    WifiMacHelper mac; mac.SetType("ns3::AdhocWifiMac");

    NodeContainer all; all.Add(veh); all.Add(rsu);
    NetDeviceContainer devs = wifi.Install(phy, mac, all);

    // Internet + AODV
    AodvHelper aodv;
    InternetStackHelper stack; stack.SetRoutingHelper(aodv);
    stack.Install(all);

    Ipv4AddressHelper addr;
    addr.SetBase("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer ifs = addr.Assign(devs);

    // RSU addresses and sinks
    const uint32_t idxRSU1 = N, idxRSU2 = N + 1, idxRSU3 = N + 2;
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
    sink1->SetStopTime(Seconds(runTime + 2));

    Ptr<RsuSink> sink2 = CreateObject<RsuSink>();
    rsu.Get(1)->AddApplication(sink2);
    sink2->Configure(ipRSU2, rsuPort, "rsu2_received.csv");
    sink2->SetOverloadControl(autoCrash, overloadPps, overloadConsec, recoverySeconds);
    sink2->SetAttackLocalPort(attackLocalPort);
    sink2->SetStartTime(Seconds(0.0));
    sink2->SetStopTime(Seconds(runTime + 2));

    Ptr<RsuSink> sink3 = CreateObject<RsuSink>();
    rsu.Get(2)->AddApplication(sink3);
    sink3->Configure(ipRSU3, rsuPort, "rsu3_received.csv");
    sink3->SetOverloadControl(false, 0.0, 0, 0.0);
    sink3->SetAttackLocalPort(attackLocalPort);
    sink3->SetStartTime(Seconds(0.0));
    sink3->SetStopTime(Seconds(runTime + 2));

    // Normal traffic
    Ptr<UniformRandomVariable> startRv = CreateObject<UniformRandomVariable>();
    startRv->SetAttribute("Min", DoubleValue(2.0));
    startRv->SetAttribute("Max", DoubleValue(3.0));

    Ptr<UniformRandomVariable> intervalRv = CreateObject<UniformRandomVariable>();
    intervalRv->SetAttribute("Min", DoubleValue(0.5));
    intervalRv->SetAttribute("Max", DoubleValue(1.5));

    std::vector<Ptr<NormalVehApp>> vapps(N);
    for (uint32_t i = 0; i < N; ++i) {
        auto app = CreateObject<NormalVehApp>();
        app->Configure(veh.Get(i), startRv, intervalRv,
                       ipRSU1, ipRSU2, ipRSU3,
                       normalLocalPort, normalDstPort,
                       normalMsgLen, runTime + 1,
                       RSU1_X, RSU2_X, RSU3_X);
        veh.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.1));
        app->SetStopTime(Seconds(runTime + 2));
        vapps[i] = app;
    }

    // Attackers fixed
    std::vector<uint32_t> attackers;
    if (N > 0)  attackers.push_back(0u);
    if (N > 30) attackers.push_back(30u);
    if (N > 59) attackers.push_back(59u);

    if (attackers.size() < 3 && N >= 3) {
        std::set<uint32_t> s(attackers.begin(), attackers.end());
        s.insert(N / 2);
        s.insert(N - 1);
        attackers.assign(s.begin(), s.end());
        if (attackers.size() > 3) attackers.resize(3);
    }

    if (attackRateAgg > 0.0) {
        perAttackerPps = attackRateAgg / std::max<size_t>(1, attackers.size());
    }

    std::set<uint32_t> attackerSet(attackers.begin(), attackers.end());
    for (auto idx : attackers) {
        if (idx >= N) continue;
        auto a = CreateObject<AttackerApp>();
        a->Configure(ipRSU2, attackDstPort, attackLocalPort,
                     attackStart, attackEnd,
                     perAttackerPps, attackTickInterval, 100);
        veh.Get(idx)->AddApplication(a);
        a->SetStartTime(Seconds(0.1));
        a->SetStopTime(Seconds(runTime + 2));
    }

    std::cout << "Attackers = ";
    for (auto i : attackers) std::cout << i << " ";
    std::cout << " perAttackerPps=" << perAttackerPps << "\n";

    // Crashers near RSU2 (middle third)
    std::vector<uint32_t> crashers;
    for (uint32_t i = 0; i < N; ++i) {
        if (attackerSet.count(i)) continue;
        if (i >= N / 3 && i < 2 * (N / 3)) crashers.push_back(i);
    }
    if (crashers.size() > 10) crashers.resize(10);

    // NetAnim
    const std::string xmlFile = "vanet_ddos_sumo.xml";
    AnimationInterface anim(xmlFile);
    anim.SetMobilityPollInterval(Seconds(0.25));
    anim.SetMaxPktsPerTraceFile(maxPktsTrace);
    anim.EnablePacketMetadata(packetMetadata);

    // initial coloring
    for (uint32_t i = 0; i < all.GetN(); ++i) {
        if (i < N) {
            bool isAtt = attackerSet.count(i);
            if (isAtt) {
                anim.UpdateNodeColor(i, 200, 0, 200);
                anim.UpdateNodeSize(i, 7.0, 7.0);
            } else {
                anim.UpdateNodeColor(i, 160, 160, 160);
                anim.UpdateNodeSize(i, 5.0, 5.0);
            }
        } else {
            if (i == idxRSU2) anim.UpdateNodeColor(i, 220, 0, 0);
            else anim.UpdateNodeColor(i, 0, 200, 0);
            anim.UpdateNodeSize(i, 16.0, 16.0);
        }
    }

    // Phase banners
    Simulator::Schedule(Seconds(0.2), [](){ std::cout << "Phase A: normal traffic\n"; });
    Simulator::Schedule(Seconds(attackStart), [](){ std::cout << "Phase B: ATTACK begins\n"; });
    double downStart = attackEnd;
    double downEnd   = downStart + 10.0;
    Simulator::Schedule(Seconds(downStart), [](){ std::cout << "Phase C: RSU2 DOWN\n"; });
    Simulator::Schedule(Seconds(downEnd),   [](){ std::cout << "Phase D: recovery\n"; });

    // Flash crowd
    Simulator::Schedule(Seconds(attackStart), [&]() {
        std::cout << "Flash crowd ON (factor=" << flashFactor << ")\n";
        for (uint32_t j = 0; j < N; ++j) if (vapps[j]) vapps[j]->SetIntervalScale(flashFactor);
    });
    Simulator::Schedule(Seconds(attackEnd), [&]() {
        std::cout << "Flash crowd OFF\n";
        for (uint32_t j = 0; j < N; ++j) if (vapps[j]) vapps[j]->SetIntervalScale(1.0);
    });

    if (!autoCrash) {
        Simulator::Schedule(Seconds(downStart), [sink2]() { sink2->ForceCrash(); });
        Simulator::Schedule(Seconds(downEnd),   [sink2]() { sink2->ForceRecover(); });
    }

    sink2->SetCrashCallback([&]() {
        std::cout << "RSU2 crashed at t=" << Simulator::Now().GetSeconds() << "s\n";
        for (auto i : crashers) {
            anim.UpdateNodeColor(i, 0, 0, 0);
            anim.UpdateNodeSize(i, 7.0, 7.0);
            if (vapps[i]) vapps[i]->Enable(false);
        }
    });

    sink2->SetRecoverCallback([&]() {
        std::cout << "RSU2 recovered at t=" << Simulator::Now().GetSeconds() << "s\n";
        for (auto i : crashers) {
            anim.UpdateNodeColor(i, 160, 160, 160);
            anim.UpdateNodeSize(i, 5.0, 5.0);
            if (vapps[i]) vapps[i]->Enable(true);
        }
    });

    phy.EnablePcapAll("vanet_ddos_sumo");

    // UDP listener for SUMO positions (bind on vehicle 0)
    Ptr<Socket> posSock = Socket::CreateSocket(veh.Get(0), UdpSocketFactory::GetTypeId());
    posSock->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9999));
    posSock->SetRecvCallback(MakeCallback(&HandleSumsPositions));
    std::cout << "Listening for SUMO positions on UDP port 9999\n";

    Simulator::Stop(Seconds(runTime + 2.0));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "Wrote NetAnim XML: " << xmlFile << "\n";
    return 0;
}

