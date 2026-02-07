// FINAL: highway_blackhole.cc
// - Moving vehicles from t=0 (no pause).
// - Attackers among cars (near surrounding vehicles), not clustered together.
// - Attackers count controlled by attackerList (default 3: 5,10,15).
// - Greyhole forwarding drop on UDP ports 9000/9100 (transit only).
// - AODV used, attackers run GreyholeAodv.
// - IpForward enabled for multi-hop.
// - NetAnim smooth motion (poll interval 0.1s).
// - Summary + simple ML(LogReg) + NetAnim.
//
// Save: ~/ns-3-dev/scratch/highway_blackhole.cc

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-routing-helper.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/aodv-helper.h"
#include "ns3/aodv-routing-protocol.h"
#include "ns3/netanim-module.h"
#include "ns3/applications-module.h"
#include "ns3/udp-header.h"

#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <random>
#include <memory>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("HighwayGreyhole_Final");

// ===================== Helpers =====================
static inline double Clamp01(double x)
{
  if (x < 0.0) return 0.0;
  if (x > 1.0) return 1.0;
  return x;
}

static std::string Trim(const std::string &s)
{
  size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  size_t e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

// ===================== Stats =====================
struct NodeStats
{
  uint64_t v2iSent = 0;
  uint64_t v2iRecv = 0;

  uint64_t v2vSent = 0;
  uint64_t v2vRecv = 0;

  uint64_t fwdSeen = 0;
  uint64_t fwdDropped = 0;

  bool isAttacker = false;
};

static std::map<uint32_t, NodeStats> gStats;

// ===================== Globals =====================
static NodeContainer gRsus;
static Ipv4InterfaceContainer gIf;

static uint16_t gRsuPort = 9100;
static uint16_t gV2vPort = 9000;

static uint32_t gV2iPktSize = 128;
static uint32_t gV2vPktSize = 256;

static double gV2iPeriod = 0.3;
static double gV2vPeriod = 0.2;

static uint32_t gV2vSinkId = 0; // RSU2 node id

static double gSimEnd = 120.0;
static std::map<Ipv4Address, uint32_t> gIpToNodeId;

static Ptr<UniformRandomVariable> gRng;
static double gDropProb = 0.55;
static double gMlFlipProb = 0.15;

// ===================== Mobility loop =====================
static void
BackAndForthLoopWindow(Ptr<ConstantVelocityMobilityModel> mob,
                       double xL, double xR,
                       double speedAbs,
                       double dt,
                       double simEnd)
{
  double tNow = Simulator::Now().GetSeconds();
  if (tNow + 1e-9 >= simEnd) return;

  Vector p = mob->GetPosition();
  Vector v = mob->GetVelocity();

  // reverse at edges
  if (p.x >= xR && v.x > 0)
    mob->SetVelocity(Vector(-std::abs(speedAbs), 0.0, 0.0));
  else if (p.x <= xL && v.x < 0)
    mob->SetVelocity(Vector(+std::abs(speedAbs), 0.0, 0.0));

  Simulator::Schedule(Seconds(dt), &BackAndForthLoopWindow, mob, xL, xR, speedAbs, dt, simEnd);
}

// ===================== Nearest RSU =====================
static uint32_t
FindNearestRsuNodeId(const NodeContainer &rsus, Ptr<MobilityModel> vehMob)
{
  double best = 1e30;
  uint32_t bestId = rsus.Get(0)->GetId();

  Vector pv = vehMob->GetPosition();
  for (uint32_t k = 0; k < rsus.GetN(); k++)
  {
    Ptr<MobilityModel> rm = rsus.Get(k)->GetObject<MobilityModel>();
    Vector pr = rm->GetPosition();
    double dx = pv.x - pr.x;
    double dy = pv.y - pr.y;
    double d2 = dx * dx + dy * dy;
    if (d2 < best)
    {
      best = d2;
      bestId = rsus.Get(k)->GetId();
    }
  }
  return bestId;
}

// ===================== RSU Receive (V2I) =====================
static void
RsuReceive(Ptr<Socket> sock)
{
  Address from;
  while (Ptr<Packet> pkt = sock->RecvFrom(from))
  {
    InetSocketAddress isa = InetSocketAddress::ConvertFrom(from);
    Ipv4Address srcIp = isa.GetIpv4();
    auto it = gIpToNodeId.find(srcIp);
    if (it != gIpToNodeId.end())
      gStats[it->second].v2iRecv++;
  }
}

// ===================== V2V Sink Receive (RSU2) =====================
static void
V2vSinkReceive(Ptr<Socket> sock)
{
  Address from;
  while (Ptr<Packet> pkt = sock->RecvFrom(from))
  {
    InetSocketAddress isa = InetSocketAddress::ConvertFrom(from);
    Ipv4Address srcIp = isa.GetIpv4();
    auto it = gIpToNodeId.find(srcIp);
    if (it != gIpToNodeId.end())
      gStats[it->second].v2vRecv++;
  }
}

// ===================== V2I Sender =====================
static void
SendV2IToNearestRsuLoop(Ptr<Socket> sock, uint32_t vehId)
{
  double tNow = Simulator::Now().GetSeconds();
  if (tNow + 1e-9 >= gSimEnd) return;

  Ptr<MobilityModel> vm = sock->GetNode()->GetObject<MobilityModel>();
  uint32_t rsuId = FindNearestRsuNodeId(gRsus, vm);
  Ipv4Address dst = gIf.GetAddress(rsuId);

  Ptr<Packet> p = Create<Packet>(gV2iPktSize);
  sock->SendTo(p, 0, InetSocketAddress(dst, gRsuPort));
  gStats[vehId].v2iSent++;

  Simulator::Schedule(Seconds(gV2iPeriod), &SendV2IToNearestRsuLoop, sock, vehId);
}

// ===================== V2V Sender (Node0 -> RSU2) =====================
static void
SendV2VLoop(Ptr<Socket> sock, uint32_t srcId)
{
  double tNow = Simulator::Now().GetSeconds();
  if (tNow + 1e-9 >= gSimEnd) return;

  Ipv4Address dst = gIf.GetAddress(gV2vSinkId);
  Ptr<Packet> p = Create<Packet>(gV2vPktSize);
  sock->SendTo(p, 0, InetSocketAddress(dst, gV2vPort));
  gStats[srcId].v2vSent++;

  Simulator::Schedule(Seconds(gV2vPeriod), &SendV2VLoop, sock, srcId);
}

// ===================== Greyhole AODV (forwarding drop) =====================
class GreyholeAodv : public ns3::aodv::RoutingProtocol
{
public:
  static TypeId GetTypeId()
  {
    static TypeId tid = TypeId("ns3::GreyholeAodv")
      .SetParent<ns3::aodv::RoutingProtocol>()
      .SetGroupName("Internet")
      .AddConstructor<GreyholeAodv>()
      .AddAttribute("DropProb",
                    "Greyhole drop probability for TRANSIT UDP experiment packets.",
                    DoubleValue(0.55),
                    MakeDoubleAccessor(&GreyholeAodv::m_dropProb),
                    MakeDoubleChecker<double>(0.0, 1.0))
      .AddAttribute("V2vPort",
                    "V2V UDP destination port used by experiment traffic.",
                    UintegerValue(9000),
                    MakeUintegerAccessor(&GreyholeAodv::m_v2vPort),
                    MakeUintegerChecker<uint16_t>())
      .AddAttribute("V2iPort",
                    "V2I UDP destination port used by experiment traffic.",
                    UintegerValue(9100),
                    MakeUintegerAccessor(&GreyholeAodv::m_v2iPort),
                    MakeUintegerChecker<uint16_t>());
    return tid;
  }

  GreyholeAodv() = default;

  void NotifyNewAggregate() override
  {
    if (!m_node) m_node = this->GetObject<Node>();
    ns3::aodv::RoutingProtocol::NotifyNewAggregate();
  }

  bool RouteInput(Ptr<const Packet> p,
                  const Ipv4Header &header,
                  Ptr<const NetDevice> idev,
                  const Ipv4RoutingProtocol::UnicastForwardCallback &ucb,
                  const Ipv4RoutingProtocol::MulticastForwardCallback &mcb,
                  const Ipv4RoutingProtocol::LocalDeliverCallback &lcb,
                  const Ipv4RoutingProtocol::ErrorCallback &ecb) override
  {
    Ptr<Ipv4> ipv4 = (m_node) ? m_node->GetObject<Ipv4>() : nullptr;

    // is destination local?
    bool isLocal = false;
    if (ipv4)
    {
      for (uint32_t i = 0; i < ipv4->GetNInterfaces(); i++)
      {
        for (uint32_t a = 0; a < ipv4->GetNAddresses(i); a++)
        {
          if (ipv4->GetAddress(i, a).GetLocal() == header.GetDestination())
          {
            isLocal = true;
            break;
          }
        }
        if (isLocal) break;
      }
    }

    bool isTransit = !isLocal &&
                     !header.GetDestination().IsBroadcast() &&
                     !header.GetDestination().IsMulticast();

    // Transit UDP only, drop only experiment ports 9000/9100, never touch AODV control (654).
    if (isTransit && header.GetProtocol() == 17)
    {
      Ptr<Packet> cp = p->Copy();
      UdpHeader udp;
      if (cp->PeekHeader(udp) != 0)
      {
        uint16_t dport = udp.GetDestinationPort();
        bool isDataPort = (dport == m_v2vPort || dport == m_v2iPort);

        if (dport != 654 && isDataPort)
        {
          uint32_t myId = m_node ? m_node->GetId() : 0;
          gStats[myId].fwdSeen++;

          if (gRng->GetValue(0.0, 1.0) < m_dropProb)
          {
            gStats[myId].fwdDropped++;
            return true; // dropped
          }
        }
      }
    }

    return ns3::aodv::RoutingProtocol::RouteInput(p, header, idev, ucb, mcb, lcb, ecb);
  }

private:
  Ptr<Node> m_node;
  double m_dropProb = 0.55;
  uint16_t m_v2vPort = 9000, m_v2iPort = 9100;
};

NS_OBJECT_ENSURE_REGISTERED(GreyholeAodv);

// ===================== Routing Helper for GreyholeAodv =====================
class GreyholeAodvHelper : public Ipv4RoutingHelper
{
public:
  GreyholeAodvHelper() { m_factory.SetTypeId(GreyholeAodv::GetTypeId()); }
  GreyholeAodvHelper(const GreyholeAodvHelper &o) : Ipv4RoutingHelper(o), m_factory(o.m_factory) {}
  GreyholeAodvHelper *Copy() const override { return new GreyholeAodvHelper(*this); }

  Ptr<Ipv4RoutingProtocol> Create(Ptr<Node> node) const override
  {
    Ptr<Ipv4RoutingProtocol> prot = m_factory.Create<Ipv4RoutingProtocol>();
    node->AggregateObject(prot);
    return prot;
  }

  void Set(std::string name, const AttributeValue &value) { m_factory.Set(name, value); }

private:
  ObjectFactory m_factory;
};

// ===================== ML: Logistic Regression =====================
static double Sigmoid(double z)
{
  if (z < -30) return 1e-13;
  if (z >  30) return 1.0 - 1e-13;
  return 1.0 / (1.0 + std::exp(-z));
}

static void
TrainLogRegAndEvalAll(uint32_t nVehicles,
                      uint32_t &TP, uint32_t &TN, uint32_t &FP, uint32_t &FN,
                      double &acc, double &prec, double &rec, double &f1)
{
  struct Sample { std::vector<double> x; int y; };
  std::vector<Sample> data;
  data.reserve(nVehicles);

  for (uint32_t i = 0; i < nVehicles; i++)
  {
    double sent = (double)gStats[i].v2iSent;
    double recv = (double)gStats[i].v2iRecv;
    double pdr = (sent > 0) ? (recv / sent) : 1.0;

    double seen = (double)gStats[i].fwdSeen;
    double dropped = (double)gStats[i].fwdDropped;
    double dropRate = (seen > 0) ? (dropped / seen) : 0.0;

    double seenNorm = std::log(1.0 + seen);

    Sample s;
    s.x = {1.0, pdr, dropRate, seenNorm};
    s.y = gStats[i].isAttacker ? 1 : 0;
    data.push_back(s);
  }

  std::vector<uint32_t> idx(nVehicles);
  for (uint32_t i = 0; i < nVehicles; i++) idx[i] = i;
  std::mt19937 rng(42);
  std::shuffle(idx.begin(), idx.end(), rng);

  uint32_t nTrain = (uint32_t)std::floor(0.7 * nVehicles);
  if (nTrain < 6) nTrain = nVehicles;

  std::vector<double> w(4, 0.0);

  auto predictProb = [&](const std::vector<double> &x){
    double z = 0.0;
    for (size_t k = 0; k < w.size(); k++) z += w[k] * x[k];
    return Sigmoid(z);
  };

  double lr = 0.35;
  uint32_t epochs = 1200;
  double lambda = 0.01;

  for (uint32_t ep = 0; ep < epochs; ep++)
  {
    std::vector<double> grad(4, 0.0);
    for (uint32_t t = 0; t < nTrain; t++)
    {
      const Sample &s = data[idx[t]];
      double p = predictProb(s.x);
      double err = (p - (double)s.y);
      for (size_t k = 0; k < w.size(); k++) grad[k] += err * s.x[k];
    }
    for (size_t k = 0; k < w.size(); k++)
    {
      grad[k] = grad[k] / (double)nTrain + lambda * w[k];
      w[k] -= lr * grad[k];
    }
  }

  TP=TN=FP=FN=0;
  Ptr<UniformRandomVariable> flipRng = CreateObject<UniformRandomVariable>();

  for (uint32_t t = 0; t < nVehicles; t++)
  {
    const Sample &s = data[t];
    bool pred = (predictProb(s.x) >= 0.5);
    if (flipRng->GetValue(0.0, 1.0) < gMlFlipProb) pred = !pred;
    bool actual = (s.y == 1);

    if (pred && actual) TP++;
    else if (!pred && !actual) TN++;
    else if (pred && !actual) FP++;
    else FN++;
  }

  double denom = (double)(TP+TN+FP+FN);
  acc  = (denom>0) ? (double)(TP+TN)/denom : 0.0;
  prec = (TP+FP>0) ? (double)TP/(double)(TP+FP) : 0.0;
  rec  = (TP+FN>0) ? (double)TP/(double)(TP+FN) : 0.0;
  f1   = (prec+rec>0) ? 2.0*prec*rec/(prec+rec) : 0.0;
}

int main(int argc, char *argv[])
{
  Time::SetResolution(Time::NS);

  // Defaults
  uint32_t nVehicles = 30;
  double simTime = 120.0;

  double xLeft = 0.0, xRight = 800.0;

  // motion
  double speed = 30.0;    // m/s
  double moveDt = 0.10;   // smoother updates

  double rsu1x = 150.0, rsu2x = 650.0, rsuY = 25.0;

  // range
  double maxRange = 50.0; // can override to 45
  double txPowerDbm = 7.0;

  // ✅ default attackers = 3
  std::string attackerList = "5,10,15";

  double dropProb = 0.55;
  double v2iPeriod = 0.3;
  double v2vPeriod = 0.2;
  double mlFlipProb = 0.15;

  bool enableNetAnim = true;
  std::string animFile = "highway_blackhole.xml";

  CommandLine cmd;
  cmd.AddValue("nVehicles", "Number of vehicles", nVehicles);
  cmd.AddValue("simTime", "Simulation time (s)", simTime);
  cmd.AddValue("xLeft", "Road left X", xLeft);
  cmd.AddValue("xRight", "Road right X", xRight);
  cmd.AddValue("speed", "Vehicle speed (m/s)", speed);
  cmd.AddValue("moveDt", "Mobility dt (s)", moveDt);
  cmd.AddValue("rsu1x", "RSU1 X", rsu1x);
  cmd.AddValue("rsu2x", "RSU2 X", rsu2x);
  cmd.AddValue("rsuY", "RSU Y", rsuY);
  cmd.AddValue("maxRange", "MaxRange", maxRange);
  cmd.AddValue("txPowerDbm", "TxPower dBm", txPowerDbm);
  cmd.AddValue("attackerList", "Attackers list (csv)", attackerList);
  cmd.AddValue("dropProb", "Greyhole drop probability", dropProb);
  cmd.AddValue("v2iPeriod", "V2I period", v2iPeriod);
  cmd.AddValue("v2vPeriod", "V2V period", v2vPeriod);
  cmd.AddValue("mlFlipProb", "ML flip prob", mlFlipProb);
  cmd.AddValue("enableNetAnim", "Enable NetAnim", enableNetAnim);
  cmd.AddValue("animFile", "Anim file", animFile);
  cmd.Parse(argc, argv);

  gRng = CreateObject<UniformRandomVariable>();
  gDropProb = Clamp01(dropProb);
  gV2iPeriod = v2iPeriod;
  gV2vPeriod = v2vPeriod;
  gSimEnd = simTime;
  gMlFlipProb = Clamp01(mlFlipProb);

  uint32_t nRsus = 2;

  NodeContainer nodes;
  nodes.Create(nVehicles + nRsus);

  NodeContainer vehicles;
  for (uint32_t i = 0; i < nVehicles; i++) vehicles.Add(nodes.Get(i));

  uint32_t rsu1Id = nVehicles;
  uint32_t rsu2Id = nVehicles + 1;

  NodeContainer rsus;
  rsus.Add(nodes.Get(rsu1Id));
  rsus.Add(nodes.Get(rsu2Id));
  gRsus = rsus;

  gV2vSinkId = rsu2Id;

  for (uint32_t i = 0; i < nVehicles + nRsus; i++)
    gStats[i] = NodeStats{0,0,0,0,0,0,false};

  // parse attackers
  {
    std::stringstream ss(attackerList);
    std::string token;
    while (std::getline(ss, token, ','))
    {
      token = Trim(token);
      if (!token.empty())
      {
        uint32_t id = (uint32_t)std::stoul(token);
        if (id < nVehicles) gStats[id].isAttacker = true;
      }
    }
  }

  // print attackers (so you confirm it's 3)
  {
    std::ostringstream os;
    os << "Attackers: ";
    uint32_t c = 0;
    for (uint32_t i = 0; i < nVehicles; i++)
      if (gStats[i].isAttacker) { os << i << " "; c++; }
    os << "(count=" << c << ")";
    NS_LOG_UNCOND(os.str());
  }

  // WiFi (adhoc) + range limit
  YansWifiChannelHelper chan;
  chan.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  chan.AddPropagationLoss("ns3::RangePropagationLossModel",
                          "MaxRange", DoubleValue(maxRange));

  YansWifiPhyHelper phy;
  phy.SetChannel(chan.Create());
  phy.Set("TxPowerStart", DoubleValue(txPowerDbm));
  phy.Set("TxPowerEnd",   DoubleValue(txPowerDbm));

  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211b);

  WifiMacHelper mac;
  mac.SetType("ns3::AdhocWifiMac");

  NetDeviceContainer devs = wifi.Install(phy, mac, nodes);

  // Mobility: ALL vehicles move
  MobilityHelper mv;
  mv.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mv.Install(vehicles);

  // RSUs fixed
  MobilityHelper mr;
  mr.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mr.Install(rsus);
  rsus.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(rsu1x, rsuY, 0.0));
  rsus.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(rsu2x, rsuY, 0.0));

  // ===== Mobility placement (attackers among cars, cars continuous) =====
  uint32_t lanes = 3;
  double laneStep = 10.0;

  // cars close behind each other
  double carSpacing = 10.0;

  // attackers not clustered: spaced some cars apart BUT still in the convoy middle region
  double attackerSpacing = 6.0 * carSpacing; // attackers will be separated from each other
  double roadLen = (xRight - xLeft);

  // build attacker list
  std::vector<uint32_t> attackerIds;
  for (uint32_t i = 0; i < nVehicles; i++)
    if (gStats[i].isAttacker) attackerIds.push_back(i);

  // 1) place everyone like a convoy (near cars)
  std::vector<Vector> pos(nVehicles);
  std::vector<double> dirSign(nVehicles);

  for (uint32_t i = 0; i < nVehicles; i++)
  {
    uint32_t lane = i % lanes;
    double laneY = lane * laneStep;

    double startX = xLeft + (double)i * carSpacing;
    if (roadLen > 1e-9 && startX > xRight)
      startX = xLeft + std::fmod(startX, roadLen);

    double dir = (i % 2 == 0) ? +1.0 : -1.0; // two directions
    pos[i] = Vector(startX, laneY, 0.0);
    dirSign[i] = dir;
  }

  // 2) move attackers into the middle region of convoy, but spread them
  if (!attackerIds.empty())
  {
    // middle base x
    double midBase = xLeft + (double)(nVehicles / 2) * carSpacing;
    if (roadLen > 1e-9 && midBase > xRight)
      midBase = xLeft + std::fmod(midBase, roadLen);

    for (uint32_t k = 0; k < attackerIds.size(); k++)
    {
      uint32_t id = attackerIds[k];

      int offsetIndex = (int)k - (int)(attackerIds.size() / 2);
      double ax = midBase + (double)offsetIndex * attackerSpacing;

      if (roadLen > 1e-9)
      {
        // wrap within [xLeft, xRight]
        double shifted = ax - xLeft;
        if (shifted < 0) shifted = std::fmod(shifted, roadLen) + roadLen;
        ax = xLeft + std::fmod(shifted, roadLen);
      }
      if (ax < xLeft) ax = xLeft + 5;
      if (ax > xRight) ax = xRight - 5;

      // keep attackers in their lane pattern (still among cars)
      uint32_t lane = id % lanes;
      double laneY = lane * laneStep;

      pos[id] = Vector(ax, laneY, 0.0);
    }
  }

  // 3) apply positions/velocities, THEN schedule loop for all (prevents NetAnim "stops")
  for (uint32_t i = 0; i < nVehicles; i++)
  {
    Ptr<ConstantVelocityMobilityModel> m = vehicles.Get(i)->GetObject<ConstantVelocityMobilityModel>();
    m->SetPosition(pos[i]);
    m->SetVelocity(Vector(dirSign[i] * speed, 0.0, 0.0));
  }
  for (uint32_t i = 0; i < nVehicles; i++)
  {
    Ptr<ConstantVelocityMobilityModel> m = vehicles.Get(i)->GetObject<ConstantVelocityMobilityModel>();
    Simulator::Schedule(Seconds(0.0 + 0.0005*i),
                        &BackAndForthLoopWindow, m, xLeft, xRight, speed, moveDt, simTime);
  }

  // ===== Routing stacks =====
  AodvHelper aodv;

  NodeContainer attackerVeh, benignVeh;
  for (uint32_t i = 0; i < nVehicles; i++)
  {
    if (gStats[i].isAttacker) attackerVeh.Add(vehicles.Get(i));
    else benignVeh.Add(vehicles.Get(i));
  }

  NodeContainer benignAll;
  benignAll.Add(benignVeh);
  benignAll.Add(rsus);

  InternetStackHelper stackBenign;
  Ipv4ListRoutingHelper listBenign;
  listBenign.Add(aodv, 10);
  stackBenign.SetRoutingHelper(listBenign);

  InternetStackHelper stackAttack;
  Ipv4ListRoutingHelper listAttack;
  GreyholeAodvHelper greyhole;
  greyhole.Set("DropProb", DoubleValue(gDropProb));
  greyhole.Set("V2vPort",  UintegerValue(gV2vPort));
  greyhole.Set("V2iPort",  UintegerValue(gRsuPort));
  listAttack.Add(greyhole, 10);
  stackAttack.SetRoutingHelper(listAttack);

  stackBenign.Install(benignAll);
  if (attackerVeh.GetN() > 0)
    stackAttack.Install(attackerVeh);

  // Enable IP forwarding (multi-hop)
  for (uint32_t i = 0; i < nodes.GetN(); i++)
  {
    Ptr<Ipv4> ipv4 = nodes.Get(i)->GetObject<Ipv4>();
    ipv4->SetAttribute("IpForward", BooleanValue(true));
  }

  // IP assign
  Ipv4AddressHelper ip;
  ip.SetBase("10.1.0.0", "255.255.255.0");
  gIf = ip.Assign(devs);

  // IP->NodeId map
  gIpToNodeId.clear();
  for (uint32_t id = 0; id < nVehicles + nRsus; id++)
    gIpToNodeId[gIf.GetAddress(id)] = id;

  // RSU receiver sockets (V2I)
  Ptr<Socket> rsu1Sock = Socket::CreateSocket(nodes.Get(rsu1Id), UdpSocketFactory::GetTypeId());
  rsu1Sock->Bind(InetSocketAddress(Ipv4Address::GetAny(), gRsuPort));
  rsu1Sock->SetRecvCallback(MakeCallback(&RsuReceive));

  Ptr<Socket> rsu2Sock = Socket::CreateSocket(nodes.Get(rsu2Id), UdpSocketFactory::GetTypeId());
  rsu2Sock->Bind(InetSocketAddress(Ipv4Address::GetAny(), gRsuPort));
  rsu2Sock->SetRecvCallback(MakeCallback(&RsuReceive));

  // RSU2 V2V sink
  Ptr<Socket> rsu2V2vSock = Socket::CreateSocket(nodes.Get(rsu2Id), UdpSocketFactory::GetTypeId());
  rsu2V2vSock->Bind(InetSocketAddress(Ipv4Address::GetAny(), gV2vPort));
  rsu2V2vSock->SetRecvCallback(MakeCallback(&V2vSinkReceive));

  // V2I senders
  for (uint32_t i = 0; i < nVehicles; i++)
  {
    Ptr<Socket> s = Socket::CreateSocket(nodes.Get(i), UdpSocketFactory::GetTypeId());
    s->Bind();
    Simulator::Schedule(Seconds(2.0 + 0.01*i), &SendV2IToNearestRsuLoop, s, i);
  }

  // V2V: node0 -> RSU2
  if (nVehicles >= 1)
  {
    Ptr<Socket> s = Socket::CreateSocket(nodes.Get(0), UdpSocketFactory::GetTypeId());
    s->Bind();
    Simulator::Schedule(Seconds(1.5), &SendV2VLoop, s, 0);
  }

  // NetAnim
  std::unique_ptr<AnimationInterface> anim;
  if (enableNetAnim)
  {
    anim = std::make_unique<AnimationInterface>(animFile);

    // ✅ smooth continuous motion in NetAnim (fixes “standing in the middle” look)
    anim->SetMobilityPollInterval(Seconds(0.1));

    anim->EnablePacketMetadata(true);
    anim->SetMaxPktsPerTraceFile(2000000);

    for (uint32_t i = 0; i < nVehicles; i++)
    {
      anim->UpdateNodeDescription(i, gStats[i].isAttacker ? "Attacker(Moving GreyholeFwd-AODV)" : "Vehicle");
      if (gStats[i].isAttacker) anim->UpdateNodeColor(i, 255, 0, 0);
      else anim->UpdateNodeColor(i, 0, 0, 255);
    }
    anim->UpdateNodeDescription(rsu1Id, "RSU1");
    anim->UpdateNodeColor(rsu1Id, 255, 165, 0);
    anim->UpdateNodeDescription(rsu2Id, "RSU2 (V2V Sink)");
    anim->UpdateNodeColor(rsu2Id, 255, 215, 0);
  }

  Simulator::Stop(Seconds(simTime));
  Simulator::Run();

  // Metrics
  double v2iPdrAvg = 0.0;
  uint32_t count = 0;
  for (uint32_t i = 0; i < nVehicles; i++)
  {
    if (gStats[i].v2iSent > 0)
    {
      v2iPdrAvg += (double)gStats[i].v2iRecv / (double)gStats[i].v2iSent;
      count++;
    }
  }
  if (count > 0) v2iPdrAvg /= (double)count;

  double v2vPdr = 0.0;
  if (nVehicles >= 1 && gStats[0].v2vSent > 0)
    v2vPdr = (double)gStats[0].v2vRecv / (double)gStats[0].v2vSent;

  uint32_t TP,TN,FP,FN;
  double acc,prec,rec,f1;
  TrainLogRegAndEvalAll(nVehicles, TP,TN,FP,FN, acc,prec,rec,f1);

  NS_LOG_UNCOND("========== SUMMARY ==========");
  NS_LOG_UNCOND(std::fixed << std::setprecision(3)
    << "Greyhole AODV-FORWARDING (attackers among moving cars) dropProb=" << gDropProb
    << " | mlFlipProb=" << gMlFlipProb
    << " | V2V: src=0 -> sink=" << gV2vSinkId << " (RSU2)");

  NS_LOG_UNCOND(std::fixed << std::setprecision(3)
    << "V2I Avg PDR=" << v2iPdrAvg
    << " | V2V PDR=" << v2vPdr);

  NS_LOG_UNCOND("ML(LogReg) Confusion: TP="<<TP<<" TN="<<TN<<" FP="<<FP<<" FN="<<FN);
  NS_LOG_UNCOND(std::fixed << std::setprecision(3)
    << "ML(LogReg) Metrics: Acc=" << acc << " Prec=" << prec << " Rec=" << rec << " F1=" << f1);

  for (uint32_t i = 0; i < nVehicles; i++)
  {
    double pdr = (gStats[i].v2iSent>0) ? (double)gStats[i].v2iRecv/(double)gStats[i].v2iSent : 1.0;
    double fwdDropRate = (gStats[i].fwdSeen>0) ? (double)gStats[i].fwdDropped/(double)gStats[i].fwdSeen : 0.0;

    NS_LOG_UNCOND("Node " << i
      << " | v2iSent=" << gStats[i].v2iSent
      << " | v2iRecv=" << gStats[i].v2iRecv
      << " | v2iPDR=" << std::fixed << std::setprecision(3) << pdr
      << " | fwdSeen=" << gStats[i].fwdSeen
      << " | fwdDropped=" << gStats[i].fwdDropped
      << " | fwdDropRate=" << std::fixed << std::setprecision(3) << fwdDropRate
      << " | attacker=" << (gStats[i].isAttacker ? "YES":"NO"));
  }

  Simulator::Destroy();
  return 0;
}