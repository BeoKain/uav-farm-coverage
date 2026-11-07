// Three-collector offline ns-3.43 replay; scheduled upload, deduplicated buffer. No OS sockets, ROS, PX4 or real UAV control.
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/propagation-module.h"
#include "ns3/seq-ts-header.h"
#include "terrain_mesh.h"
#include <algorithm>
#include <array>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

using namespace ns3;

class FarmReplayLoss : public PropagationLossModel {
public:
    static TypeId GetTypeId() {
        static TypeId id=TypeId("ns3::FarmReplayLoss").SetParent<PropagationLossModel>()
            .SetGroupName("Propagation").AddConstructor<FarmReplayLoss>();
        return id;
    }
    double penaltyDb=25;
    const TerrainMesh* mesh=nullptr;
    struct Entry { Vector a,b;bool blocked; };
    mutable std::map<std::pair<uint32_t,uint32_t>,Entry> cache;
private:
    double DoCalcRxPower(double tx,Ptr<MobilityModel> a,Ptr<MobilityModel> b) const override {
        if(penaltyDb==0)return tx;
        auto i=a->GetObject<Node>()->GetId(),j=b->GetObject<Node>()->GetId();
        auto key=std::minmax(i,j);
        auto k=std::make_pair(key.first,key.second);
        auto p=a->GetPosition(),q=b->GetPosition();if(i>j)std::swap(p,q);
        auto same=[](Vector x,Vector y){return x.x==y.x&&x.y==y.y&&x.z==y.z;};
        auto it=cache.find(k);
        if(it==cache.end()||!same(it->second.a,p)||!same(it->second.b,q)) {
            bool blocked=mesh->MinimumClearance(p,q)<=1e-9;
            cache[k]={p,q,blocked};
        }
        return tx-(cache.at(k).blocked?penaltyDb:0.);
    }
    int64_t DoAssignStreams(int64_t) override { return 0; }
};
NS_OBJECT_ENSURE_REGISTERED(FarmReplayLoss);

struct Location { std::string label; Vector position; };
struct Flow {
    std::string sensor;
    uint32_t generated=0,submitted=0,sendErrors=0,rx=0,duplicates=0,rxBeforeDeadline=0;
    uint64_t bytes=0;
    std::set<uint32_t> sequences;
    std::array<std::set<uint32_t>,3> collectorSequences;
    std::vector<double> delaysMs;
    double completeTime=-1,firstSeen=-1;
    uint32_t arpPendingExpiryDrops=0;
};
static std::vector<Flow> flows;
static std::map<uint32_t,uint32_t> sourceIndex;
static uint32_t packets=100,payload=1000;
static double armOffset=0,deadline=300;
static double interval=.2,missionDuration=0;


static void ArpPendingDrop(uint32_t index,Ptr<const Packet>) {
    flows.at(index).arpPendingExpiryDrops++;
}

static void Receive(uint32_t collector,Ptr<Socket> socket) {
    Address from;
    while(auto p=socket->RecvFrom(from)) {
        auto addr=InetSocketAddress::ConvertFrom(from).GetIpv4().Get();
        auto it=sourceIndex.find(addr);
        if(it==sourceIndex.end())continue;
        auto &f=flows[it->second];
        SeqTsHeader header;
        if(p->GetSize()<header.GetSerializedSize())continue;
        p->RemoveHeader(header);
        f.collectorSequences.at(collector).insert(header.GetSeq());
        if(!f.sequences.insert(header.GetSeq()).second){f.duplicates++;continue;}
        if(f.firstSeen<0)f.firstSeen=Simulator::Now().GetSeconds()-1.;
        f.rx++; f.bytes+=p->GetSize();
        if(Simulator::Now().GetSeconds()-1.+armOffset<=deadline)f.rxBeforeDeadline++;
        f.delaysMs.push_back((Simulator::Now()-header.GetTs()).GetSeconds()*1000.);
        if(f.rx==packets)f.completeTime=Simulator::Now().GetSeconds()-1.;
    }
}

static void Send(Ptr<Socket> socket,uint32_t sensor,uint32_t sequence, double stop) {
    auto &f=flows[sensor];f.generated++;
    auto p=Create<Packet>(payload);SeqTsHeader header;header.SetSeq(sequence%packets);p->AddHeader(header);
    if(socket->Send(p)>=0)f.submitted++;else f.sendErrors++;
    if(sequence+1<packets && Simulator::Now()+Seconds(interval)<Seconds(stop))
        Simulator::Schedule(Seconds(interval),&Send,socket,sensor,sequence+1,stop);
}

static std::vector<Location> ReadLocations(const std::string& path) {
    std::ifstream input(path);if(!input)throw std::runtime_error("Cannot read positions");
    std::string line;std::getline(input,line);std::vector<Location> result;
    while(std::getline(input,line)) {
        std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);
        Location p;if(!(row>>p.label>>p.position.x>>p.position.y>>p.position.z))
            throw std::runtime_error("Malformed position row");
        result.push_back(p);
    }
    if(result.size()<2 || result.back().label!="UAV3")
        throw std::runtime_error("Expected sensors followed by UAV");
    return result;
}

int main(int argc,char** argv) {
    std::string positionsFile,terrainFile,trajectoryFile,outputFile,timelineFile,diagnosticsFile,assignmentsFile;
    double extraLoss=25,txDbm=0,pathExponent=2.2,arpDeadTimeout=100;
    uint32_t seed=12345,run=1;
    CommandLine cmd(__FILE__);
    cmd.AddValue("positions","CSV: label,x,y,z; UAV last",positionsFile);
    cmd.AddValue("terrain","Terrain grid CSV",terrainFile);
    cmd.AddValue("trajectory","CSV: time_s,east_m,north_m,up_m",trajectoryFile);
    cmd.AddValue("timeline","Per-second per-sensor time series CSV",timelineFile);
    cmd.AddValue("diagnostics","Optional per-second ARP state CSV",diagnosticsFile);
    cmd.AddValue("arpDeadTimeout","ARP DeadTimeout seconds; other settings unchanged",arpDeadTimeout);
    cmd.AddValue("output","Per-sensor metrics CSV",outputFile);
    cmd.AddValue("assignments","CSV sensor,collector index 0..2,start_s,end_s",assignmentsFile);
    cmd.AddValue("extraLossDb","Assumed extra loss on terrain-blocked links",extraLoss);
    cmd.AddValue("txDbm","Transmit power, all nodes",txDbm);
    cmd.AddValue("pathExponent","Log-distance exponent",pathExponent);
    cmd.AddValue("packets","Unique chunks per sensor; one application attempt each",packets);
    cmd.AddValue("payload","Application data bytes per packet, excludes SeqTsHeader",payload);
    cmd.AddValue("interval","Seconds between packets per sensor",interval);
    cmd.AddValue("seed","Global RNG seed",seed);cmd.AddValue("run","RNG run",run);
    cmd.AddValue("armOffset","Seconds from ARM to MISSION",armOffset);
    cmd.AddValue("deadline","Service deadline from ARM, seconds",deadline);
    cmd.Parse(argc,argv);
    NS_ABORT_MSG_IF(!std::isfinite(armOffset)||armOffset<0||!std::isfinite(deadline)||deadline<=armOffset,"Invalid deadline clock");
    NS_ABORT_MSG_IF(assignmentsFile.empty()||positionsFile.empty()||terrainFile.empty()||trajectoryFile.empty()||outputFile.empty()||timelineFile.empty(),"Missing file arguments");
    NS_ABORT_MSG_IF(packets==0||payload==0||payload>1400||interval<=0||extraLoss<0
                    ||!std::isfinite(arpDeadTimeout)||arpDeadTimeout<=0,"Invalid traffic/model parameters");
    Config::SetDefault("ns3::ArpCache::DeadTimeout",TimeValue(Seconds(arpDeadTimeout)));
    RngSeedManager::SetSeed(seed);RngSeedManager::SetRun(run);
    auto locations=ReadLocations(positionsFile);
    const uint32_t sensors=locations.size()-3;
    NodeContainer nodes;nodes.Create(locations.size());
    MobilityHelper mobility;mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    for(uint32_t i=0;i<sensors;i++) {
        mobility.Install(nodes.Get(i));
        nodes.Get(i)->GetObject<MobilityModel>()->SetPosition(locations[i].position);
    }
    std::vector<uint32_t> owners(sensors,99);
    std::vector<std::pair<double,double>> windows(sensors);
    std::ifstream ownerInput(assignmentsFile);NS_ABORT_MSG_IF(!ownerInput,"Cannot read assignments");
    std::string line;std::getline(ownerInput,line);
    while(std::getline(ownerInput,line)) {
        std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);
        std::string label; uint32_t owner; double start,end;
        NS_ABORT_MSG_IF(!(row>>label>>owner>>start>>end)||owner>=3||!std::isfinite(start)||!std::isfinite(end)||start<0||end<=start,"Invalid assignment window");
        uint32_t i=0;while(i<sensors&&locations[i].label!=label)i++;
        NS_ABORT_MSG_IF(i==sensors||owners[i]!=99,"Unknown/duplicate sensor assignment");owners[i]=owner; windows[i]={start,end};
    }
    for(auto owner:owners)NS_ABORT_MSG_IF(owner>=3,"Missing assignment");
    std::array<Ptr<WaypointMobilityModel>,3> uavs;
    for(uint32_t k=0;k<3;k++){uavs[k]=CreateObject<WaypointMobilityModel>();nodes.Get(sensors+k)->AggregateObject(uavs[k]);}
    std::ifstream routeInput(trajectoryFile);NS_ABORT_MSG_IF(!routeInput,"Cannot read trajectories");
    std::getline(routeInput,line);std::array<double,3> previous={-1,-1,-1};
    while(std::getline(routeInput,line)) {
        std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);
        double t,x,y,z;uint32_t k;
        NS_ABORT_MSG_IF(!(row>>t>>k>>x>>y>>z)||k>=3,"Malformed trajectory row");
        NS_ABORT_MSG_IF(!std::isfinite(t)||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||t<0||t<=previous[k],"Invalid trajectory");
        if(previous[k]<0){NS_ABORT_MSG_IF(t!=0,"First timestamp must be 0");uavs[k]->AddWaypoint(Waypoint(Seconds(0),Vector(x,y,z)));}
        uavs[k]->AddWaypoint(Waypoint(Seconds(1+t),Vector(x,y,z)));previous[k]=t;
    }
    missionDuration=previous[0];
    for(double t:previous)NS_ABORT_MSG_IF(t!=missionDuration||t<.2,"Mismatched member durations");
    for(auto [start,end]:windows) NS_ABORT_MSG_IF(end>missionDuration,"Window exceeds trajectory");
    TerrainMesh mesh;mesh.Load(terrainFile);
    auto terrain=CreateObject<FarmReplayLoss>();terrain->penaltyDb=extraLoss;
    terrain->mesh=&mesh;
    auto loss=CreateObject<LogDistancePropagationLossModel>();
    loss->SetAttribute("Exponent",DoubleValue(pathExponent));
    loss->SetAttribute("ReferenceDistance",DoubleValue(1.));
    loss->SetAttribute("ReferenceLoss",DoubleValue(40.05));
    loss->SetNext(terrain);
    auto channel=CreateObject<YansWifiChannel>();channel->SetPropagationLossModel(loss);
    channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
    WifiHelper wifi;wifi.SetStandard(WIFI_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager","DataMode",StringValue("DsssRate1Mbps"),
                                "ControlMode",StringValue("DsssRate1Mbps"));
    YansWifiPhyHelper phy;phy.SetChannel(channel);
    phy.Set("TxPowerStart",DoubleValue(txDbm));phy.Set("TxPowerEnd",DoubleValue(txDbm));
    phy.Set("RxSensitivity",DoubleValue(-95.));phy.Set("RxNoiseFigure",DoubleValue(7.));
    WifiMacHelper mac;mac.SetType("ns3::AdhocWifiMac");
    auto devices=wifi.Install(phy,mac,nodes);wifi.AssignStreams(devices,100);
    InternetStackHelper internet;internet.Install(nodes);
    Ipv4AddressHelper addresses;addresses.SetBase("10.1.1.0","255.255.255.0");
    auto interfaces=addresses.Assign(devices);
    std::vector<Ptr<Socket>> sinks;
    for(uint32_t k=0;k<3;k++){
        auto sink=Socket::CreateSocket(nodes.Get(sensors+k),UdpSocketFactory::GetTypeId());
        NS_ABORT_MSG_IF(sink->Bind(InetSocketAddress(Ipv4Address::GetAny(),9000))<0,"Cannot bind collector sink");
        sink->SetRecvCallback(MakeBoundCallback(&Receive,k));sinks.push_back(sink);
    }
    flows.resize(sensors);
    std::vector<Ptr<ArpCache>> arpCaches;
    for(uint32_t i=0;i<sensors;i++) {
        auto cache=nodes.Get(i)->GetObject<Ipv4L3Protocol>()->GetInterface(1)->GetArpCache();
        NS_ABORT_MSG_IF(!cache,"Missing sensor ARP cache");
        arpCaches.push_back(cache);
        if(!diagnosticsFile.empty())cache->TraceConnectWithoutContext("Drop",MakeBoundCallback(&ArpPendingDrop,i));
    }
    auto jitter=CreateObject<UniformRandomVariable>();jitter->SetStream(3000);
    std::vector<Ptr<Socket>> senders;
    for(uint32_t i=0;i<sensors;i++) {
        flows[i].sensor=locations[i].label;sourceIndex[interfaces.GetAddress(i).Get()]=i;
        auto sender=Socket::CreateSocket(nodes.Get(i),UdpSocketFactory::GetTypeId());sender->Bind();
        sender->Connect(InetSocketAddress(interfaces.GetAddress(sensors+owners[i]),9000));senders.push_back(sender);
        auto [start,end]=windows[i];
        double sendAt=1.+start+jitter->GetValue(0,.01);
        NS_ABORT_MSG_IF(sendAt>=1.+end,"Window too short for initial jitter");
        Simulator::Schedule(Seconds(sendAt),&Send,sender,i,0,1.+end);
    }
    const double window=missionDuration+2.; // final position held during 2 s drain
    std::ofstream timeline(timelineFile);NS_ABORT_MSG_IF(!timeline,"Cannot create timeline");
    timeline<<std::setprecision(12);
    timeline<<"time_s,sensor,east_m,north_m,up_m,unique_bytes,rx_unique,rx_duplicates,tx_attempts,clearance_m,distance_m,predicted_rx_dbm\n";
    std::ofstream diagnostics;
    if(!diagnosticsFile.empty()) {
        diagnostics.open(diagnosticsFile);NS_ABORT_MSG_IF(!diagnostics,"Cannot create diagnostics");
        diagnostics<<std::setprecision(12);
        diagnostics<<"time_s,sensor,arp_state,arp_expired,arp_pending_expiry_drops\n";
    }
    auto snapshot=[&](double t) {

        for(uint32_t i=0;i<sensors;i++) {
            auto uav=uavs[owners[i]];auto p=uav->GetPosition();
            auto a=nodes.Get(i)->GetObject<MobilityModel>();auto q=a->GetPosition();auto &f=flows[i];
            timeline<<t<<','<<f.sensor<<','<<p.x<<','<<p.y<<','<<p.z<<','<<f.bytes<<','<<f.rx<<','
                    <<f.duplicates<<','<<f.generated<<','<<mesh.MinimumClearance(q,p)<<','
                    <<CalculateDistance(q,p)<<','<<loss->CalcRxPower(txDbm,a,uav)<<'\n';
            if(diagnostics.is_open()) {
                auto entry=arpCaches[i]->Lookup(interfaces.GetAddress(sensors+owners[i]));
                std::string state=!entry?"NONE":entry->IsDead()?"DEAD":entry->IsWaitReply()?"WAIT_REPLY":
                                  entry->IsAlive()?"ALIVE":"OTHER";
                diagnostics<<t<<','<<f.sensor<<','<<state<<','<<(entry&&entry->IsExpired()?1:0)
                           <<','<<f.arpPendingExpiryDrops<<'\n';
            }
        }
    };
    for(double t=0;t<window;t+=1.)Simulator::Schedule(Seconds(1+t),[&,t](){snapshot(t);});
    Simulator::Schedule(Seconds(1+window-1e-6),[&](){snapshot(window-1e-6);});
    Simulator::Stop(Seconds(1.+window));Simulator::Run();timeline.close();
    std::ofstream out(outputFile);NS_ABORT_MSG_IF(!out,"Cannot create metrics file");
    out<<"sensor,tx_attempts,tx_submitted,send_errors,rx_total,rx_unique,rx_duplicates,unique_bytes,collection_ratio,attempt_delivery_ratio,unique_goodput_bps,mean_unique_delay_ms,p95_unique_delay_ms,complete,first_seen_s,completion_s,receiver,uav1_unique,uav2_unique,uav3_unique,unique_bytes_by_arm_deadline\n";
    out<<std::setprecision(12);
    for(uint32_t i=0;i<sensors;i++) {
        auto &f=flows[i];double sum=0;for(auto d:f.delaysMs)sum+=d;
        std::sort(f.delaysMs.begin(),f.delaysMs.end());
        out<<f.sensor<<','<<f.generated<<','<<f.submitted<<','<<f.sendErrors<<','<<f.rx+f.duplicates<<','
           <<f.rx<<','<<f.duplicates<<','<<f.bytes<<','<<double(f.rx)/packets<<','
           <<(f.generated?double(f.rx+f.duplicates)/f.generated:0)<<','<<f.bytes*8./window<<',';
        if(f.rx)out<<sum/f.rx;out<<',';
        if(f.rx)out<<f.delaysMs[static_cast<size_t>(std::ceil(.95*f.delaysMs.size()))-1];out<<',';
        out<<(f.rx==packets?1:0)<<',';
        if(f.firstSeen>=0)out<<f.firstSeen;out<<',';
        if(f.completeTime>=0)out<<f.completeTime;out<<','<<owners[i];
        for(auto &set:f.collectorSequences)out<<','<<set.size();out<<','<<uint64_t(f.rxBeforeDeadline)*payload<<'\n';
    }
    out.close();Simulator::Destroy();return 0;
}
