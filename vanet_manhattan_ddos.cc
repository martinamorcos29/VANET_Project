// NS-3.40 VANET Manhattan DDoS simulation (FIXED build)
// - Manhattan Grid mobility (ConstantVelocity + turns at intersections)
// - 3 RSUs fixed, V2V + V2I, DDoS focused on RSU2
// - RSU2 turns RED on crash then back GREEN on recovery
// - Attack packets tagged (for NetAnim packet metadata)

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
NS_LOG_COMPONENT_DEFINE("VanetDdosManhattan");

// ---------------------------------------------------------
// RSU sink
// ---------------------------------------------------------
class RsuSink : public Application {
public:
  void Configure(Ipv4Address bindAddr, uint16_t port, const std::string &csv) {
    m_bindAddr = bindAddr; m_port = port; m_csv = csv;
  }

  void SetOverloadControl(bool autoCrash, double overloadPps, uint32_t consec, double recoverySeconds) {
    m_autoCrash = autoCrash;
    m_overloadPps = overloadPps;
    m_overloadConsec = consec;
    m_recoverySeconds = recoverySeconds;
  }

  void SetCrashCallback(std::function<void()> onCrash) { m_onCrash = std::move(onCrash); }
  void SetRecoverCallback(std::function<void()> onRecover) { m_onRecover = std::move(onRecover); }

  void SetAttackLocalPort(uint16_t p) { m_attackLocalPort = p; }

  void StartApplication() override {
    if (!m_sock) {
      m_sock = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
      m_sock->Bind(InetSocketAddress(m_bindAddr, m_port));
      m_sock->SetRecvCallback(MakeCallback(&RsuSink::HandleRead, this));
    }

    m_enabled = true;
    m_isCrashed = false;
    m_windowCount = 0;
    m_consecOver = 0;
    m_eventCounter = 0;

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
      m_isCrashed = false;
      m_consecOver = 0;
      m_windowCount = 0;

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

    double pps = m_windowCount;
    m_windowCount = 0;

    std::cout << "RSU " << m_bindAddr << " pps=" << pps
              << " at t=" << Simulator::Now().GetSeconds() << "s\n";

    if (!m_isCrashed && m_autoCrash) {
      if (pps > m_overloadPps) ++m_consecOver;
      else m_consecOver = 0;

      if (m_consecOver >= m_overloadConsec) {
        NS_LOG_WARN("RSU overload -> crash");
        ForceCrash();
      }
    }

    if (m_enabled && !m_isCrashed) ScheduleTick();
  }

private:
  Ptr<Socket> m_sock{nullptr};
  Ipv4Address m_bindAddr;
  uint16_t m_port{0};
  std::string m_csv{"rsu_received.csv"};

  bool m_enabled{false};
  bool m_isCrashed{false};

  bool m_autoCrash{false};
  double m_overloadPps{60.0};
  uint32_t m_overloadConsec{2};
  double m_recoverySeconds{10.0};

  uint32_t m_consecOver{0};
  uint32_t m_windowCount{0};
  uint64_t m_eventCounter{0};

  uint16_t m_attackLocalPort{9003};
  std::function<void()> m_onCrash;
  std::function<void()> m_onRecover;
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
                 uint32_t msgLen, double endTime,
                 Vector rsu1Pos, Vector rsu2Pos, Vector rsu3Pos)
  {
    m_self = self;
    m_startRv = startRv;
    m_intervalRv = intervalRv;

    m_rsu1 = rsu1; m_rsu2 = rsu2; m_rsu3 = rsu3;
    m_rsu1Pos = rsu1Pos; m_rsu2Pos = rsu2Pos; m_rsu3Pos = rsu3Pos;

    m_localPort = localPort;
    m_dstPort = dstPort;
    m_msgLen = msgLen;
    m_end = endTime;

    m_intervalScale = 1.0;
    m_enabled = true;
    m_firstSent = false;

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
  static double Dist2(Vector a, Vector b) {
    double dx = a.x - b.x, dy = a.y - b.y;
    return dx*dx + dy*dy;
  }

  void SendOnce() {
    if (!m_enabled || Simulator::Now().GetSeconds() >= m_end) return;

    // nearest RSU by distance
    Ipv4Address targetRsu = m_rsu2;

    Ptr<MobilityModel> mob = m_self->GetObject<MobilityModel>();
    if (mob) {
      Vector p = mob->GetPosition();
      double d1 = Dist2(p, m_rsu1Pos);
      double d2 = Dist2(p, m_rsu2Pos);
      double d3 = Dist2(p, m_rsu3Pos);

      if (d1 <= d2 && d1 <= d3) targetRsu = m_rsu1;
      else if (d2 <= d1 && d2 <= d3) targetRsu = m_rsu2;
      else targetRsu = m_rsu3;
    }

    // V2I
    m_sockRsu->SendTo(Create<Packet>(m_msgLen), 0, InetSocketAddress(targetRsu, m_dstPort));

    // V2V beacon
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

private:
  Ptr<Node> m_self;
  Ptr<Socket> m_sockRsu{nullptr};
  Ptr<Socket> m_sockV2V{nullptr};

  Ptr<UniformRandomVariable> m_startRv;
  Ptr<UniformRandomVariable> m_intervalRv;

  Ipv4Address m_rsu1, m_rsu2, m_rsu3;
  Vector m_rsu1Pos, m_rsu2Pos, m_rsu3Pos;

  uint16_t m_localPort{9001}, m_dstPort{9002};
  uint32_t m_msgLen{100};
  double m_end{1e9};

  bool m_enabled{true};
  bool m_firstSent{false};
  double m_intervalScale{1.0};

  uint16_t m_beaconPort{9004};
  uint16_t m_beaconLocalPort{9005};
  uint32_t m_beaconMsgLen{80};
};

// ---------------------------------------------------------
// ATTACK TAG (for NetAnim packet metadata)
// ---------------------------------------------------------
class AttackTag : public Tag {
public:
  static TypeId GetTypeId() {
    static TypeId tid = TypeId("AttackTag")
      .SetParent<Tag>()
      .AddConstructor<AttackTag>();
    return tid;
  }

  TypeId GetInstanceTypeId() const override { return GetTypeId(); }
  uint32_t GetSerializedSize() const override { return 1; }

  void Serialize(TagBuffer i) const override { i.WriteU8(1); }
  void Deserialize(TagBuffer i) override { (void)i.ReadU8(); }
  void Print(std::ostream &os) const override { os << "ATTACK"; }
};

// ---------------------------------------------------------
// ATTACKER
// ---------------------------------------------------------
class AttackerApp : public Application {
public:
  AttackerApp() : m_dst(Ipv4Address(),0), m_reported(false) {}

  void Configure(Ipv4Address rsu2, uint16_t port, uint16_t localPort,
                 double start, double end, double pps,
                 double tickInterval = 0.1, uint32_t msgLen = 100)
  {
    m_dst = InetSocketAddress(rsu2, port);
    m_s = start; m_e = end; m_pps = pps; m_tick = tickInterval;
    m_localPort = localPort; m_msgLen = msgLen;
    m_burstCount = std::max<uint32_t>(1, (uint32_t)std::lrint(m_pps * m_tick));
  }

  void StartApplication() override {
    if (!m_sock) {
      m_sock = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
      m_sock->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_localPort));
    }
    double delay = m_s - Simulator::Now().GetSeconds();
    if (delay < 0.0) delay = 0.0;
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
      AttackTag tag; p->AddPacketTag(tag);
      m_sock->SendTo(p, 0, m_dst);
    }
    Simulator::Schedule(Seconds(m_tick), &AttackerApp::Loop, this);
  }

private:
  Ptr<Socket> m_sock{nullptr};
  InetSocketAddress m_dst;
  double m_s{0}, m_e{0}, m_pps{0}, m_tick{0.1};
  uint32_t m_burstCount{1};
  bool m_reported{false};
  uint16_t m_localPort{9003};
  uint32_t m_msgLen{100};
};

// ---------------------------------------------------------
// Manhattan mobility step (FIXED scheduling via lambda)
// ---------------------------------------------------------
static void
ManhattanStep(NodeContainer* veh,
              double minX, double maxX,
              double minY, double maxY,
              double gridStep,
              double interval)
{
  static bool init = false;
  static std::vector<double> baseSpeed;

  if (!init) {
    baseSpeed.resize(veh->GetN(), 20.0);
    for (uint32_t i = 0; i < veh->GetN(); ++i) {
      auto mm = veh->Get(i)->GetObject<ConstantVelocityMobilityModel>();
      if (!mm) continue;
      Vector v = mm->GetVelocity();
      double s = std::sqrt(v.x*v.x + v.y*v.y);
      if (s < 1.0) s = 15.0;
      baseSpeed[i] = s;
    }
    init = true;
  }

  Ptr<UniformRandomVariable> rv = CreateObject<UniformRandomVariable>();

  for (uint32_t i = 0; i < veh->GetN(); ++i) {
    auto mm = veh->Get(i)->GetObject<ConstantVelocityMobilityModel>();
    if (!mm) continue;

    Vector p = mm->GetPosition();
    Vector v = mm->GetVelocity();
    double speed = baseSpeed[i];

    // bounce if out of bounds
    if (p.x < minX) { p.x = minX; v.x = +speed; v.y = 0; mm->SetPosition(p); }
    if (p.x > maxX) { p.x = maxX; v.x = -speed; v.y = 0; mm->SetPosition(p); }
    if (p.y < minY) { p.y = minY; v.y = +speed; v.x = 0; mm->SetPosition(p); }
    if (p.y > maxY) { p.y = maxY; v.y = -speed; v.x = 0; mm->SetPosition(p); }

    // detect near intersection (~2m from grid lines)
    double rx = std::fmod(std::fabs(p.x - minX), gridStep);
    double ry = std::fmod(std::fabs(p.y - minY), gridStep);
    bool nearIntersection = (rx < 2.0 || rx > (gridStep - 2.0)) &&
                            (ry < 2.0 || ry > (gridStep - 2.0));

    if (nearIntersection) {
      double u = rv->GetValue(0.0, 1.0);
      bool movingX = (std::fabs(v.x) > std::fabs(v.y));

      if (u >= 0.60) {
        // 40% turn: switch axis, random sign
        double sign = (rv->GetValue(0.0, 1.0) < 0.5) ? -1.0 : +1.0;
        if (movingX) { v.x = 0; v.y = sign * speed; }
        else         { v.y = 0; v.x = sign * speed; }
      }
      // 60% straight: keep direction
    }

    // normalize to exact speed
    double mag = std::sqrt(v.x*v.x + v.y*v.y);
    if (mag < 0.1) { v.x = speed; v.y = 0; mag = speed; }
    v.x = (v.x / mag) * speed;
    v.y = (v.y / mag) * speed;

    mm->SetVelocity(v);
  }

  // ✅ schedule next tick using lambda (no overload confusion)
  Simulator::Schedule(Seconds(interval), [=]() {
    ManhattanStep(veh, minX, maxX, minY, maxY, gridStep, interval);
  });
}

// ---------------------------------------------------------
// MAIN
// ---------------------------------------------------------
int main(int argc, char* argv[]) {
  uint32_t N = 40;
  double runTime = 60.0;

  uint16_t normalLocalPort = 9001, normalDstPort = 9002;
  uint32_t normalMsgLen = 100;

  uint16_t attackLocalPort = 9003, attackDstPort = 9002;
  double attackStart = 20.0, attackEnd = 40.0, attackTickInterval = 0.1;
  double perAttackerPps = 200.0;
  double attackRateAgg = 0.0;

  bool autoCrash = false;
  double overloadPps = 60.0;
  uint32_t overloadConsec = 2;
  double recoverySeconds = 10.0;

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

  // RSUs fixed
  MobilityHelper mobRsu;
  mobRsu.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobRsu.Install(rsu);

  const double RSU_X = 200.0;
  const double RSU_DY = 100.0;
  const double CRASH_RADIUS_Y = 40.0;

  Vector rsu1Pos(RSU_X, 1*RSU_DY, 0.0);
  Vector rsu2Pos(RSU_X, 2*RSU_DY, 0.0);
  Vector rsu3Pos(RSU_X, 3*RSU_DY, 0.0);

  rsu.Get(0)->GetObject<MobilityModel>()->SetPosition(rsu1Pos);
  rsu.Get(1)->GetObject<MobilityModel>()->SetPosition(rsu2Pos);
  rsu.Get(2)->GetObject<MobilityModel>()->SetPosition(rsu3Pos);

  // Vehicles: Manhattan init positions + constant velocity
  MobilityHelper mobVeh;
  mobVeh.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobVeh.Install(veh);

  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> speedRv(15.0, 28.0);
  std::uniform_real_distribution<double> jitter(-2.0, 2.0);
  std::uniform_int_distribution<int> dirRv(0, 3);

  const double minX = 80.0,  maxX = 320.0;
  const double minY = 60.0,  maxY = 340.0;
  const double gridStep = 40.0;

  int gx = (int)((maxX - minX) / gridStep);
  int gy = (int)((maxY - minY) / gridStep);

  for (uint32_t i = 0; i < N; ++i) {
    auto mm = veh.Get(i)->GetObject<ConstantVelocityMobilityModel>();
    if (!mm) continue;

    int ix = (int)(rng() % (gx + 1));
    int iy = (int)(rng() % (gy + 1));
    double x = minX + ix * gridStep + jitter(rng);
    double y = minY + iy * gridStep + jitter(rng);

    double s = speedRv(rng);
    int d = dirRv(rng);

    Vector vel(0,0,0);
    if (d == 0) vel = Vector(+s, 0, 0);
    if (d == 1) vel = Vector(-s, 0, 0);
    if (d == 2) vel = Vector(0, +s, 0);
    if (d == 3) vel = Vector(0, -s, 0);

    mm->SetPosition(Vector(x, y, 0.0));
    mm->SetVelocity(vel);
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

  // RSU addresses
  const uint32_t idxRSU1 = N, idxRSU2 = N+1, idxRSU3 = N+2;
  Ipv4Address ipRSU1 = ifs.GetAddress(idxRSU1);
  Ipv4Address ipRSU2 = ifs.GetAddress(idxRSU2);
  Ipv4Address ipRSU3 = ifs.GetAddress(idxRSU3);
  const uint16_t rsuPort = normalDstPort;

  // RSU sinks
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

  // normal app random times
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
                   normalMsgLen, runTime+1,
                   rsu1Pos, rsu2Pos, rsu3Pos);
    veh.Get(i)->AddApplication(app);
    app->SetStartTime(Seconds(0.1));
    app->SetStopTime(Seconds(runTime+2));
    vapps[i] = app;
  }

  // attackers list (same logic)
  std::vector<uint32_t> attackers;

  // pick 5 attackers spread across the vehicles (avoid duplicates)
  std::set<uint32_t> s;
  if (N > 0) s.insert(0);
  if (N > 10) s.insert(N/4);
  if (N > 20) s.insert(N/2);
  if (N > 30) s.insert(3*N/4);
  if (N > 1) s.insert(N-1);

    attackers.assign(s.begin(), s.end());

    // make sure we don't exceed 5
    if (attackers.size() > 5) attackers.resize(5);


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

  std::cout << "Attackers = ";
  for (auto i : attackers) std::cout << i << " ";
  std::cout << " perAttackerPps=" << perAttackerPps << "\n";

  std::set<uint32_t> attackerSet(attackers.begin(), attackers.end());

  // saved velocities for "accident stop/resume"
  std::vector<Vector> savedVel(N);
  for (uint32_t i=0;i<N;++i) {
    auto mm = veh.Get(i)->GetObject<ConstantVelocityMobilityModel>();
    if (mm) savedVel[i] = mm->GetVelocity();
  }
  std::vector<uint32_t> crashers;

  // NetAnim
  const std::string xmlFile = "vanet_manhattan.xml";
  AnimationInterface anim(xmlFile);
  anim.SetMobilityPollInterval(Seconds(0.25));
  anim.SetMaxPktsPerTraceFile(maxPktsTrace);
  anim.EnablePacketMetadata(packetMetadata);

  // visuals
  for (uint32_t i=0;i<all.GetN();++i) {
    anim.UpdateNodeSize(i, 4.0, 4.0);
    if (i < N) {
      bool isAtt = attackerSet.count(i);
      if (isAtt) {
        anim.UpdateNodeColor(i, 200,0,200);
        anim.UpdateNodeSize(i, 6.0, 6.0);
      } else {
        auto mm = veh.Get(i)->GetObject<MobilityModel>();
        double y = mm ? mm->GetPosition().y : 0.0;
        if (y < 1.5*RSU_DY)       anim.UpdateNodeColor(i,160,160,160);
        else if (y < 2.5*RSU_DY)  anim.UpdateNodeColor(i,255,128,0);
        else                      anim.UpdateNodeColor(i,0,120,255);
      }
    } else {
      anim.UpdateNodeColor(i, 0,200,0);
      anim.UpdateNodeSize(i, 10.0, 10.0);
    }
  }

  // phases
  Simulator::Schedule(Seconds(0.2), [](){ std::cout << "Phase A: normal traffic\n"; });
  Simulator::Schedule(Seconds(attackStart), [](){ std::cout << "Phase B: ATTACK begins\n"; });

  double downStart = attackEnd;
  double downEnd   = downStart + 10.0;
  Simulator::Schedule(Seconds(downStart), [](){ std::cout << "Phase C: RSU2 DOWN + accidents\n"; });
  Simulator::Schedule(Seconds(downEnd),   [](){ std::cout << "Phase D: recovery\n"; });

  // flash crowd
  Simulator::Schedule(Seconds(attackStart), [&]() {
    std::cout << "Flash crowd ON at t=" << Simulator::Now().GetSeconds()
              << " factor=" << flashFactor << "\n";
    for (uint32_t j=0;j<N;++j) if (vapps[j]) vapps[j]->SetIntervalScale(flashFactor);
  });
  Simulator::Schedule(Seconds(attackEnd), [&]() {
    std::cout << "Flash crowd OFF at t=" << Simulator::Now().GetSeconds() << "\n";
    for (uint32_t j=0;j<N;++j) if (vapps[j]) vapps[j]->SetIntervalScale(1.0);
  });

  if (!autoCrash) {
    Simulator::Schedule(Seconds(downStart), [sink2]() { sink2->ForceCrash(); });
    Simulator::Schedule(Seconds(downEnd),   [sink2]() { sink2->ForceRecover(); });
  }

  // crash callbacks: RSU2 red + stop cars near RSU2 in Y band
  sink2->SetCrashCallback([&]() {
    std::cout << "⚠️ RSU2 crashed at t=" << Simulator::Now().GetSeconds() << "s\n";
    anim.UpdateNodeColor(idxRSU2, 255, 0, 0);

    crashers.clear();
    double rsu2Y = rsu2Pos.y;

    for (uint32_t i=0;i<N;++i) {
      if (attackerSet.count(i)) continue;
      auto mm = veh.Get(i)->GetObject<MobilityModel>();
      if (!mm) continue;
      double dy = std::fabs(mm->GetPosition().y - rsu2Y);
      if (dy <= CRASH_RADIUS_Y) crashers.push_back(i);
    }

    for (auto i : crashers) {
      anim.UpdateNodeColor(i, 0,0,0);
      anim.UpdateNodeSize(i, 6.0,6.0);
      if (vapps[i]) vapps[i]->Enable(false);

      auto mm = veh.Get(i)->GetObject<ConstantVelocityMobilityModel>();
      if (mm) {
        savedVel[i] = mm->GetVelocity();
        mm->SetVelocity(Vector(0,0,0));
      }
    }
  });

  sink2->SetRecoverCallback([&]() {
    std::cout << "ℹ️ RSU2 recovered at t=" << Simulator::Now().GetSeconds() << "s\n";
    anim.UpdateNodeColor(idxRSU2, 0,200,0);

    for (auto i : crashers) {
      auto mm2 = veh.Get(i)->GetObject<MobilityModel>();
      double y = mm2 ? mm2->GetPosition().y : 0.0;

      if (y < 1.5*RSU_DY)       anim.UpdateNodeColor(i,160,160,160);
      else if (y < 2.5*RSU_DY)  anim.UpdateNodeColor(i,255,128,0);
      else                      anim.UpdateNodeColor(i,0,120,255);

      anim.UpdateNodeSize(i, 4.5,4.5);
      if (vapps[i]) vapps[i]->Enable(true);

      auto mm = veh.Get(i)->GetObject<ConstantVelocityMobilityModel>();
      if (mm) mm->SetVelocity(savedVel[i]);
    }
  });

  // Start Manhattan loop
  ManhattanStep(&veh, minX, maxX, minY, maxY, gridStep, 0.5);

  phy.EnablePcapAll("vanet_manhattan");

  Simulator::Stop(Seconds(runTime + 2.0));
  Simulator::Run();
  Simulator::Destroy();

  std::cout << "Wrote NetAnim XML: " << xmlFile << "\n";
  return 0;
}

