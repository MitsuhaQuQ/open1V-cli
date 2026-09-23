#include "open1v/bridge_client.hpp"
#include "open1v/camera_protocol.hpp"
#include "open1v/camera_data.hpp"
#ifdef OPEN1V_DEBUG_CLI
#include "open1v/self_test.hpp"
#endif
#include "open1v/serial_transport.hpp"
#include "open1v/winusb_transport.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
std::uint64_t parseUnsigned(const std::string& text,std::uint64_t minimum,std::uint64_t maximum,const char* field){
    if(text.empty())throw std::runtime_error(std::string(field)+" is empty");
    std::uint64_t value=0;const auto result=std::from_chars(text.data(),text.data()+text.size(),value,10);
    if(result.ec==std::errc::result_out_of_range)throw std::runtime_error(std::string(field)+" is too large");
    if(result.ec!=std::errc{}||result.ptr!=text.data()+text.size())throw std::runtime_error(std::string(field)+" must contain digits only");
    if(value<minimum||value>maximum)throw std::runtime_error(std::string(field)+" is outside its valid range");
    return value;
}
#ifdef OPEN1V_DEBUG_CLI
std::vector<std::uint8_t> parseHex(std::string text) {
    text.erase(std::remove_if(text.begin(), text.end(),
        [](char c) { return c == ' ' || c == ':' || c == '-'; }), text.end());
    if (text.empty() || text.size() % 2 != 0)
        throw std::runtime_error("hex payload must contain complete bytes");
    if (text.size() > 510) throw std::runtime_error("hex payload exceeds 255 bytes");
    std::vector<std::uint8_t> result;
    for (std::size_t i = 0; i < text.size(); i += 2) {
        std::size_t parsed = 0;
        const auto value = std::stoul(text.substr(i, 2), &parsed, 16);
        if (parsed != 2 || value > 0xff)
            throw std::runtime_error("hex payload contains a non-hex byte");
        result.push_back(static_cast<std::uint8_t>(value));
    }
    return result;
}
#endif

void usage() {
    std::cout << "open1V commands:\n"
#ifdef OPEN1V_DEBUG_CLI
                 "  self-test          Run tests without hardware\n"
                 "  frame ping         Print an encoded bridge ping frame\n"
                 "  bridge ping        Check the WinUSB bridge\n"
                 "  bridge status      Read bridge status bytes\n"
                 "  bridge link-status Diagnose the S3-to-RA4 link\n"
                 "  camera identify    Read camera identity (no setting changes)\n";
#else
                 "  (no arguments)     Start the interactive settings session\n"
                 "  camera console     Interactive settings session\n";
#endif
    std::cout << "  camera id          Read camera identity in readable form\n"
                 "  camera cfn         Read all C.Fn groups as option numbers\n"
                 "  camera pfn         Read P.Fn states and known parameters\n"
                 "  camera clock       Read camera local time\n";
    std::cout << "  camera info        Read ID, C.Fn, P.Fn and clock in one session\n";
#ifdef OPEN1V_DEBUG_CLI
    std::cout << "  camera console     Interactive settings session; F2 is sent on exit\n";
    std::cout << "  camera set-id --id N\n"
                 "  camera set-clock --clock YYMMDDhhmmss\n";
    std::cout << "  camera read-settings  Read fixed status blocks (read-only)\n";
    std::cout << "  camera <flow>-debug   Run a validated diagnostic flow\n"
                 "    flows: handshake, settings, cfn, pfn, clock, unknown-read,\n"
                 "           film-header, film-download, continuous, exit\n"
                 "    debug flows keep PC mode open; only exit-debug sends F2\n";
    std::cout << "  write debug flows require --allow-write-debug; busy cameras are waited for:\n"
                 "    pfn3/4/5/12-roundtrip, pfn-global-roundtrip,\n"
                 "    cfn19-roundtrip, camera-id-roundtrip\n";
    std::cout << "    shooting-mask/width16/width8-roundtrip\n";
    std::cout << "  camera clock-set-debug --clock YYMMDDhhmmss --allow-write-debug\n";
    std::cout << "  camera pfn-write-debug --block C3 --expect A010 --value 9810 --allow-write-debug\n"
                 "  camera cfn-write-debug --block D1 --expect <hex> --value <hex> --allow-write-debug\n";
    std::cout << "options:\n"
                 "  --port PATH        Use an explicit serial port\n"
                 "  --winusb           Use the reserved WinUSB transport\n";
#else
    std::cout << "  camera set-id --id N\n"
                 "  camera set-clock --clock YYMMDDhhmmss\n";
    std::cout << "options:\n"
                 "  --port PATH        Use an explicit serial port\n"
                 "  --winusb           Use the WinUSB transport\n";
#endif
}

void printHex(const std::vector<std::uint8_t>& bytes) {
    for (const auto byte : bytes) {
        std::cout << std::hex << std::setfill('0') << std::setw(2)
                  << static_cast<unsigned>(byte) << ' ';
    }
    std::cout << std::dec << '\n';
}

std::string hexByte(std::uint8_t v) {
    char text[5]{}; std::snprintf(text, sizeof(text), "%02X", v); return text;
}
int oneHot(std::uint8_t v) {
    if (!v || (v & (v - 1))) return -1;
    int n=0; while ((v >>= 1) != 0) ++n; return n;
}
const std::vector<std::uint8_t>& findPacket(
    const std::vector<open1v::CameraPacket>& packets, std::uint8_t command) {
    return open1v::cameraBlock(packets, command);
}
void printCfnBank(const std::vector<open1v::CameraPacket>& packets,
                  std::uint8_t command, const char* name,
                  const std::vector<open1v::CameraPacket>* original = nullptr) {
        const auto options=open1v::decodeCfnOptions(findPacket(packets,command));
        std::array<bool,19> changed{};
        if(original){const auto old=open1v::decodeCfnOptions(findPacket(*original,command));for(int fn=1;fn<=19;++fn)changed[fn-1]=old[fn-1]!=options[fn-1];}
        std::cout<<name<<":\n  No. |";
        for(int fn=1;fn<=19;++fn)std::cout<<(changed[fn-1]?'*':' ')<<std::setfill('0')<<std::setw(2)<<fn<<std::setfill(' ')<<" |";
        std::cout<<"\n  Opt |";
        for(std::size_t i=0;i<options.size();++i){std::cout<<(changed[i]?'*':' ')<<std::setw(2);if(options[i]!=0xff)std::cout<<options[i];else std::cout<<'?';std::cout<<" |";}
        std::cout<<'\n';
}
void printCfn(const std::vector<open1v::CameraPacket>& packets) {
    static constexpr std::array<std::pair<std::uint8_t,const char*>,4> groups{{
        {std::uint8_t{0xd1},"Current"},{std::uint8_t{0xd5},"Registered 1"},
        {std::uint8_t{0xd7},"Registered 2"},{std::uint8_t{0xd9},"Registered 3"}}};
    for (const auto& [command,name] : groups) {
        printCfnBank(packets,command,name);
    }
}
std::vector<int> pfnBits(const std::vector<std::uint8_t>& b){
    std::vector<int> r; const auto enabled=open1v::decodePfnEnabled(b);
    for(int n=1;n<=30;++n)if(enabled[n-1])r.push_back(n); return r;
}
void printList(const char* label,const std::vector<int>& v){std::cout<<label;for(int n:v)std::cout<<' '<<n;if(v.empty())std::cout<<" none";std::cout<<'\n';}
double fixed16(std::uint8_t h,std::uint8_t l){return ((unsigned(h)<<8)|l)/16.0;}
using PfnChoice=std::pair<std::string,std::uint8_t>;
PfnChoice pfnChoice(const char* name,int value){return {name,static_cast<std::uint8_t>(value)};}
const std::vector<PfnChoice>& shutterChoices(){
    static const std::vector<PfnChoice> values{
        pfnChoice("30 s",0x10),pfnChoice("25 s",0x13),pfnChoice("20 s",0x15),pfnChoice("15 s",0x18),pfnChoice("13 s",0x1b),pfnChoice("10 s",0x1d),pfnChoice("8 s",0x20),pfnChoice("6 s",0x23),pfnChoice("5 s",0x25),pfnChoice("4 s",0x28),pfnChoice("3.2 s",0x2b),pfnChoice("2.5 s",0x2d),pfnChoice("2 s",0x30),pfnChoice("1.6 s",0x33),pfnChoice("1.3 s",0x35),pfnChoice("1 s",0x38),pfnChoice("0.8 s",0x3b),pfnChoice("0.6 s",0x3d),pfnChoice("0.5 s",0x40),pfnChoice("0.4 s",0x43),pfnChoice("0.3 s",0x45),
        pfnChoice("1/4 s",0x48),pfnChoice("1/5 s",0x4b),pfnChoice("1/6 s",0x4d),pfnChoice("1/8 s",0x50),pfnChoice("1/10 s",0x53),pfnChoice("1/13 s",0x55),pfnChoice("1/15 s",0x58),pfnChoice("1/20 s",0x5b),pfnChoice("1/25 s",0x5d),pfnChoice("1/30 s",0x60),pfnChoice("1/40 s",0x63),pfnChoice("1/50 s",0x65),pfnChoice("1/60 s",0x68),pfnChoice("1/80 s",0x6b),pfnChoice("1/100 s",0x6d),pfnChoice("1/125 s",0x70),pfnChoice("1/160 s",0x73),pfnChoice("1/200 s",0x75),pfnChoice("1/250 s",0x78),pfnChoice("1/320 s",0x7b),pfnChoice("1/400 s",0x7d),pfnChoice("1/500 s",0x80),pfnChoice("1/640 s",0x83),pfnChoice("1/800 s",0x85),pfnChoice("1/1000 s",0x88),pfnChoice("1/1250 s",0x8b),pfnChoice("1/1600 s",0x8d),pfnChoice("1/2000 s",0x90),pfnChoice("1/2500 s",0x93),pfnChoice("1/3200 s",0x95),pfnChoice("1/4000 s",0x98),pfnChoice("1/5000 s",0x9b),pfnChoice("1/6400 s",0x9d),pfnChoice("1/8000 s",0xa0)};
    return values;
}
const std::vector<PfnChoice>& apertureChoices(){
    static const std::vector<PfnChoice> values{
        pfnChoice("f/1.0",0x08),pfnChoice("f/1.1",0x0b),pfnChoice("f/1.2",0x0d),pfnChoice("f/1.4",0x10),pfnChoice("f/1.6",0x13),pfnChoice("f/1.8",0x15),pfnChoice("f/2.0",0x18),pfnChoice("f/2.2",0x1b),pfnChoice("f/2.5",0x1d),pfnChoice("f/2.8",0x20),pfnChoice("f/3.2",0x23),pfnChoice("f/3.5",0x25),pfnChoice("f/4.0",0x28),pfnChoice("f/4.5",0x2b),pfnChoice("f/5.0",0x2d),pfnChoice("f/5.6",0x30),pfnChoice("f/6.3",0x33),pfnChoice("f/7.1",0x35),pfnChoice("f/8.0",0x38),pfnChoice("f/9.0",0x3b),pfnChoice("f/10",0x3d),pfnChoice("f/11",0x40),pfnChoice("f/13",0x43),pfnChoice("f/14",0x45),pfnChoice("f/16",0x48),pfnChoice("f/18",0x4b),pfnChoice("f/20",0x4d),pfnChoice("f/22",0x50),pfnChoice("f/25",0x53),pfnChoice("f/29",0x55),pfnChoice("f/32",0x58),pfnChoice("f/36",0x5b),pfnChoice("f/40",0x5d),pfnChoice("f/45",0x60),pfnChoice("f/51",0x63),pfnChoice("f/57",0x65),pfnChoice("f/64",0x68),pfnChoice("f/72",0x6b),pfnChoice("f/81",0x6d),pfnChoice("f/91",0x70)};
    return values;
}
std::string pfnChoiceName(std::uint8_t value,const std::vector<PfnChoice>& choices){for(const auto& choice:choices)if(choice.second==value)return choice.first;return "wire 0x"+hexByte(value);}
std::string compactLower(std::string text){text.erase(std::remove_if(text.begin(),text.end(),[](unsigned char c){return std::isspace(c)!=0;}),text.end());std::transform(text.begin(),text.end(),text.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});return text;}
double positiveNumber(const std::string& text,const char* field){if(text.empty()||text.size()>32)throw std::runtime_error(std::string(field)+" has an invalid length");bool dot=false,digit=false;for(const unsigned char c:text){if(std::isdigit(c)){digit=true;continue;}if(c=='.'&&!dot){dot=true;continue;}throw std::runtime_error(std::string(field)+" must contain only digits and one decimal point");}if(!digit)throw std::runtime_error(std::string(field)+" must contain a digit");std::size_t used=0;const auto value=std::stod(text,&used);if(used!=text.size()||!std::isfinite(value)||value<=0)throw std::runtime_error(std::string(field)+" must be a positive number");return value;}
double parseShutterSeconds(std::string text){text=compactLower(std::move(text));if(!text.empty()&&(text.back()=='s'||text.back()=='\"'))text.pop_back();const auto slash=text.find('/');double value=0;if(slash!=std::string::npos){if(text.find('/',slash+1)!=std::string::npos)throw std::runtime_error("shutter speed contains more than one slash");const auto numerator=positiveNumber(text.substr(0,slash),"shutter numerator");const auto denominator=positiveNumber(text.substr(slash+1),"shutter denominator");value=numerator/denominator;}else value=positiveNumber(text,"shutter speed");if(!std::isfinite(value)||value<1.0/8000.0||value>30.0)throw std::runtime_error("shutter speed must be between 1/8000 s and 30 s");return value;}
double parseAperture(std::string text){text=compactLower(std::move(text));if(text.starts_with("f/"))text.erase(0,2);else if(text.starts_with('f'))text.erase(0,1);const auto value=positiveNumber(text,"aperture");if(value<1.0||value>91.0)throw std::runtime_error("aperture must be between f/1.0 and f/91");return value;}
const PfnChoice& nearestExposureChoice(double target,const std::vector<PfnChoice>& choices,bool shutter){return *std::min_element(choices.begin(),choices.end(),[=](const PfnChoice& left,const PfnChoice& right){const auto physical=[=](const PfnChoice& choice){return shutter?std::exp2((0x38-choice.second)/8.0):std::exp2((choice.second-0x08)/16.0);};return std::abs(std::log2(target/physical(left)))<std::abs(std::log2(target/physical(right)));});}
bool pfnEnabled(const std::vector<std::uint8_t>& b,int n){const int g=(n-1)/8,bit=(n-1)%8;return (b.at(2+3-g)&(1u<<bit))!=0;}
const char* pfnSwitchDescription(int n){switch(n){
case 6:return "Register/switch shooting and metering modes";case 7:return "Repeat AEB during continuous shooting";
case 8:return "Use only two AEB frames";case 9:return "Reverse the AEB sequence";case 10:return "Retain program-shift amount";
case 11:return "Retain multiple-exposure setting";case 13:return "AI Servo continuous shooting follows drive speed";
case 14:return "Disable lens focus-search drive";case 15:return "Disable AF-assist beam";case 16:return "Automatic release when focus is achieved";
case 17:return "Disable automatic AF-point selection";case 18:return "Enable automatic AF-point selection with C.Fn-11-2";
case 21:return "Silent film advance after shooting";case 22:return "Disable shutter release without film";
case 24:return "Keep LCD illumination on during bulb exposure";case 26:return "Shorten shutter-release time lag";
case 28:return "Disable exposure compensation with Quick Control Dial";default:return nullptr;}}
void printPfnSwitch(const std::vector<open1v::CameraPacket>& p,int n,bool changed=false){const auto& dd=findPacket(p,0xdd);std::cout<<(changed?"* ":"  ")<<'['<<(pfnEnabled(dd,n)?"ON":"OFF")<<"] P.Fn-"<<n<<' '<<pfnSwitchDescription(n)<<'\n';}
void printPfnItem(const std::vector<open1v::CameraPacket>& p,int n,bool changed=false){
    const auto& dd=findPacket(p,0xdd);std::cout<<(changed?"* ":"  ")<<'['<<(pfnEnabled(dd,n)?"ON":"OFF")<<"] P.Fn-"<<n<<' ';
    const auto& c1=findPacket(p,0xc1);const auto& c3=findPacket(p,0xc3);const auto& c4=findPacket(p,0xc4);
    const auto& cb=findPacket(p,0xcb);const auto& cc=findPacket(p,0xcc);const auto& ca=findPacket(p,0xca);
    const auto& c7=findPacket(p,0xc7);const auto& c8=findPacket(p,0xc8);const auto& c0=findPacket(p,0xc0);
    const auto& cf=findPacket(p,0xcf);const auto& ce=findPacket(p,0xce);const auto& c5=findPacket(p,0xc5);const auto& c6=findPacket(p,0xc6);const auto& cd=findPacket(p,0xcd);
    static constexpr std::array<const char*,4> meter{"Evaluative","Spot","Partial","Center-weighted average"};
    auto fps=[](std::uint8_t v){return (0x14-v)/2;};
    auto modeList=[](std::uint8_t value,const auto& modes,bool allowed){std::ostringstream out;bool first=true;for(const auto& mode:modes){const bool present=(value&mode.first)!=0;if(present==allowed){if(!first)out<<", ";out<<mode.second;first=false;}}return first?std::string("None"):out.str();};
    switch(n){
    case 1:{static constexpr std::array<std::pair<std::uint8_t,const char*>,6> modes{{
        {std::uint8_t{0x04},"Manual exposure"},{std::uint8_t{0x20},"Program AE"},{std::uint8_t{0x08},"Shutter-priority AE"},
        {std::uint8_t{0x01},"Aperture-priority AE"},{std::uint8_t{0x02},"Depth-of-field AE"},{std::uint8_t{0x10},"Bulb"}}};
        const auto mask=static_cast<std::uint8_t>(c5.at(2)&0x3f);
        std::cout<<"Limit selectable shooting modes\n    |-- Allowed: "<<modeList(mask,modes,true)
                 <<"\n    |-- Excluded: "<<modeList(mask,modes,false)
                 <<"\n    `-- Wire allowed mask: 0x"<<hexByte(mask);break;}
    case 2:{static constexpr std::array<std::pair<std::uint8_t,const char*>,4> modes{{
        {std::uint8_t{0x10},"Evaluative"},{std::uint8_t{0x20},"Spot"},{std::uint8_t{0x40},"Partial"},{std::uint8_t{0x80},"Center-weighted average"}}};
        const auto mask=static_cast<std::uint8_t>(c6.at(2)&0xf0);
        std::cout<<"Limit selectable metering modes\n    |-- Allowed: "<<modeList(mask,modes,true)
                 <<"\n    |-- Excluded: "<<modeList(mask,modes,false)
                 <<"\n    `-- Wire allowed mask: 0x"<<hexByte(mask);break;}
    case 3:{int i=oneHot(c1.at(2)>>4);std::cout<<"Manual-exposure metering\n    `-- Mode: "<<(i>=0&&i<4?meter[i]:"unknown");break;}
    case 4:std::cout<<"Set shutter-speed range\n    |-- Fastest: "<<pfnChoiceName(c3.at(2),shutterChoices())<<"\n    `-- Slowest: "<<pfnChoiceName(c3.at(3),shutterChoices());break;
    case 5:std::cout<<"Set aperture range\n    |-- Smallest: "<<pfnChoiceName(c4.at(2),apertureChoices())<<"\n    `-- Largest: "<<pfnChoiceName(c4.at(3),apertureChoices());break;
    case 12:{static constexpr std::array<const char*,5> sensitivity{"Slowest","Slow","Standard","Fast","Fastest"};const auto value=cb.at(2);const auto index=value/0x20;std::cout<<"AI Servo tracking sensitivity\n    `-- Level: "<<(value<=0x80&&value%0x20==0?sensitivity[index]:("wire 0x"+hexByte(value)));break;}
    case 19:std::cout<<"Set continuous shooting speeds\n    |-- Low: "<<fps(cc.at(3))<<" fps\n    |-- High: "<<fps(cc.at(4))<<" fps\n    `-- Ultra-high: "<<fps(cc.at(5))<<" fps";break;
    case 20:std::cout<<"Limit continuous frames\n    `-- Frames: "<<unsigned(ca.at(2));break;
    case 23:std::cout<<"Set button activation times\n    |-- Timer 1: "<<fixed16(c7.at(2),c7.at(3))<<" s\n    |-- Timer 2: "<<fixed16(c8.at(2),c8.at(3))<<" s\n    `-- Post-release: "<<fixed16(c0.at(2),c0.at(3))<<" s";break;
    case 25:{auto named=[](std::uint8_t value,const auto& table){for(const auto& x:table)if(x.first==value)return std::string(x.second);return std::string("wire 0x")+hexByte(value);};
        static constexpr std::array<std::pair<std::uint8_t,const char*>,6> shooting{{{std::uint8_t{0x10},"Program AE"},{std::uint8_t{0x20},"Shutter-priority AE"},{std::uint8_t{0x40},"Aperture-priority AE"},{std::uint8_t{0x08},"Depth-of-field AE"},{std::uint8_t{0x80},"Manual exposure"},{std::uint8_t{0x04},"Bulb"}}};
        static constexpr std::array<std::pair<std::uint8_t,const char*>,4> metering{{{std::uint8_t{0x04},"Evaluative"},{std::uint8_t{0x00},"Partial"},{std::uint8_t{0x02},"Spot"},{std::uint8_t{0x06},"Center-weighted average"}}};
        static constexpr std::array<std::pair<std::uint8_t,const char*>,7> drive{{{std::uint8_t{0x20},"Single-frame"},{std::uint8_t{0x00},"Continuous (body only)"},{std::uint8_t{0x10},"Low-speed continuous"},{std::uint8_t{0x30},"High-speed continuous"},{std::uint8_t{0x40},"Ultra-high-speed continuous"},{std::uint8_t{0x60},"10 sec. self-timer"},{std::uint8_t{0x50},"2 sec. self-timer"}}};
        static constexpr std::array<std::pair<std::uint8_t,const char*>,2> af{{{std::uint8_t{0x04},"One-Shot AF"},{std::uint8_t{0x08},"AI Servo AF"}}};
        std::cout<<"Set CLEAR defaults\n    |-- Shooting mode: "<<named(cd.at(2)&0xfc,shooting)<<"\n    |-- Metering mode: "<<named(cd.at(3)&0x06,metering)<<"\n    |-- Film advance mode: "<<named(cd.at(4)&0xf0,drive)<<"\n    |-- AF mode: "<<named(cd.at(5)&0x7f,af)<<"\n    `-- Focusing point selection: "<<((cd.at(3)&0x80)?"Automatic":"Center focusing point");break;}
    case 27:{static constexpr std::array<const char*,3> dial{"Main Dial only","Quick Control Dial only","Both dials"};const auto value=dd.at(6);std::cout<<"Reverse electronic dial direction\n    `-- Dials: "<<(value<dial.size()?dial[value]:"unknown");break;}
    case 29:std::cout<<"Remaining-roll warning\n    `-- Rolls: "<<unsigned(ce.at(2));break;
    case 30:std::cout<<"Film-ID imprint density\n    `-- Density: "<<(cf.at(2)==0x02?"Dark":cf.at(2)==0x00?"Light":("wire 0x"+hexByte(cf.at(2))));break;
    default:std::cout<<"Unsupported preview";break;
    } std::cout<<'\n';
}
void printPfn(const std::vector<open1v::CameraPacket>& p,
              const std::vector<open1v::CameraPacket>* original=nullptr){
    static constexpr std::array<int,13> parameterized{1,2,3,4,5,12,19,20,23,25,27,29,30};
    auto isChanged=[&](int n){if(!original)return false;const auto& now=findPacket(p,0xdd);const auto& old=findPacket(*original,0xdd);bool changed=pfnEnabled(now,n)!=pfnEnabled(old,n)||(n==27&&now.at(6)!=old.at(6));auto block=[&](std::uint8_t command){return findPacket(p,command)!=findPacket(*original,command);};switch(n){case 1:changed|=block(0xc5);break;case 2:changed|=block(0xc6);break;case 3:changed|=block(0xc1);break;case 4:changed|=block(0xc3);break;case 5:changed|=block(0xc4);break;case 12:changed|=block(0xcb);break;case 19:changed|=block(0xcc);break;case 20:changed|=block(0xca);break;case 23:changed|=block(0xc7)||block(0xc8)||block(0xc0);break;case 25:changed|=block(0xcd);break;case 29:changed|=block(0xce);break;case 30:changed|=block(0xcf);break;}return changed;};
    auto printRange=[&](const char* heading,int first,int last){std::cout<<heading<<":\n";for(int n=first;n<=last;++n){const auto changed=isChanged(n);if(std::find(parameterized.begin(),parameterized.end(),n)!=parameterized.end())printPfnItem(p,n,changed);else printPfnSwitch(p,n,changed);}};
    printRange("Exposure Functions",1,11);
    printRange("AF Functions",12,18);
    printRange("Film Transport Functions",19,22);
    printRange("Other Functions",23,30);
}
std::vector<std::uint8_t>& mutablePacket(std::vector<open1v::CameraPacket>& packets,std::uint8_t command){for(auto& p:packets)if(!p.bytes.empty()&&p.bytes[0]==command)return p.bytes;throw std::runtime_error("camera block missing");}
bool promptChoice(const char* prompt,const std::vector<std::string>& names,std::size_t& selected){
    if(names.empty())throw std::runtime_error("selection list is empty");
    for(std::size_t i=0;i<names.size();++i)std::cout<<"  "<<i+1<<") "<<names[i]<<'\n';
    std::cout<<prompt;std::string text;if(!std::getline(std::cin,text)||text=="q"||text=="Q")return false;
    const auto value=parseUnsigned(text,1,names.size(),"selection");selected=static_cast<std::size_t>(value-1);return true;
}
void editPfnParameters(std::vector<open1v::CameraPacket>& packets,int n){
    auto u8=[](unsigned value){return static_cast<std::uint8_t>(value);};
    auto chooseCode=[&](const char* title,std::uint8_t command,std::size_t offset,const std::vector<std::pair<std::string,std::uint8_t>>& values){std::cout<<title<<":\n";std::vector<std::string> names;for(const auto& v:values)names.push_back(v.first);std::size_t selected;if(promptChoice("Setting (q=keep current): ",names,selected))mutablePacket(packets,command).at(2+offset)=values[selected].second;};
    auto chooseSub=[&](const std::vector<std::string>& names,std::size_t& selected){std::cout<<"Sub-items:\n";return promptChoice("Sub-item (q=finish): ",names,selected);};
    if(n==1||n==2){const std::uint8_t command=n==1?0xc5:0xc6;const std::vector<std::pair<std::string,std::uint8_t>> modes=n==1?
        std::vector<std::pair<std::string,std::uint8_t>>{{"Manual exposure",u8(0x04)},{"Program AE",u8(0x20)},{"Shutter-priority AE",u8(0x08)},{"Aperture-priority AE",u8(0x01)},{"Depth-of-field AE",u8(0x02)},{"Bulb",u8(0x10)}}:
        std::vector<std::pair<std::string,std::uint8_t>>{{"Evaluative",u8(0x10)},{"Spot",u8(0x20)},{"Partial",u8(0x40)},{"Center-weighted average",u8(0x80)}};
        std::vector<std::string> names;for(const auto& m:modes){const bool allowed=(mutablePacket(packets,command).at(2)&m.second)!=0;names.push_back(m.first+" ["+(allowed?"Allowed":"Excluded")+"]");}
        std::size_t item;if(!chooseSub(names,item))return;std::size_t setting;if(!promptChoice("Setting (q=keep current): ",{"Allowed","Excluded"},setting))return;auto& value=mutablePacket(packets,command).at(2);if(setting==0)value|=modes[item].second;else value&=static_cast<std::uint8_t>(~modes[item].second);return;}
    if(n==3){chooseCode("Metering mode",0xc1,0,{{"Evaluative",u8(0x10)},{"Spot",u8(0x20)},{"Partial",u8(0x40)},{"Center-weighted average",u8(0x80)}});return;}
    if(n==4){std::size_t item;if(!chooseSub({"Fastest shutter speed","Slowest shutter speed"},item))return;std::cout<<(item?"Slowest":"Fastest")<<" shutter speed (examples: 1/300, 0.3s, 2; q=keep current): ";std::string text;if(!std::getline(std::cin,text)||text=="q"||text=="Q")return;const auto& selected=nearestExposureChoice(parseShutterSeconds(text),shutterChoices(),true);auto& b=mutablePacket(packets,0xc3);const auto other=b.at(2+(item?0:1));if((!item&&selected.second<other)||(item&&b.at(2)<selected.second))throw std::runtime_error("fastest shutter speed must not be slower than the slowest shutter speed");b.at(2+item)=selected.second;std::cout<<"Nearest camera setting: "<<selected.first<<" (wire 0x"<<hexByte(selected.second)<<")\n";return;}
    if(n==5){std::size_t item;if(!chooseSub({"Smallest aperture","Largest aperture"},item))return;std::cout<<(item?"Largest":"Smallest")<<" aperture (examples: f/2.7, 5.6; q=keep current): ";std::string text;if(!std::getline(std::cin,text)||text=="q"||text=="Q")return;const auto& selected=nearestExposureChoice(parseAperture(text),apertureChoices(),false);auto& b=mutablePacket(packets,0xc4);const auto other=b.at(2+(item?0:1));if((!item&&selected.second<other)||(item&&b.at(2)<selected.second))throw std::runtime_error("smallest aperture must not be wider than the largest aperture");b.at(2+item)=selected.second;std::cout<<"Nearest camera setting: "<<selected.first<<" (wire 0x"<<hexByte(selected.second)<<")\n";return;}
    if(n==12){chooseCode("AI Servo tracking sensitivity",0xcb,0,{{"Slowest",u8(0x00)},{"Slow",u8(0x20)},{"Standard",u8(0x40)},{"Fast",u8(0x60)},{"Fastest",u8(0x80)}});return;}
    if(n==19){std::size_t item;if(!chooseSub({"Ultra-high-speed continuous","High-speed continuous","Low-speed continuous"},item))return;std::vector<std::pair<std::string,std::uint8_t>> rates;for(int fps=1;fps<=10;++fps)rates.push_back({std::to_string(fps)+" fps",static_cast<std::uint8_t>(0x14-fps*2)});const std::size_t offset=item==0?3:item==1?2:1;chooseCode("Continuous shooting speed",0xcc,offset,rates);if(item==0)mutablePacket(packets,0xcc).at(2)=mutablePacket(packets,0xcc).at(5);return;}
    if(n==27){chooseCode("Electronic dials",0xdd,4,{{"Main Dial only",u8(0)},{"Quick Control Dial only",u8(1)},{"Both dials",u8(2)}});return;}
    if(n==20||n==29){std::cout<<(n==20?"Number of continuous frames":"Remaining-roll warning threshold")<<"\nValue (q=keep current): ";std::string text;if(!std::getline(std::cin,text)||text=="q"||text=="Q")return;const auto value=parseUnsigned(text,1,n==20?36:10,"value");mutablePacket(packets,n==20?0xca:0xce).at(2)=static_cast<std::uint8_t>(value);return;}
    if(n==23){std::size_t item;if(!chooseSub({"Timer 1","Timer 2","Post-release timer"},item))return;std::cout<<"Seconds 0..3600 (q=keep current): ";std::string text;if(!std::getline(std::cin,text)||text=="q"||text=="Q")return;const auto seconds=parseUnsigned(text,0,3600,"timer");const auto command=std::array<std::uint8_t,3>{0xc7,0xc8,0xc0}[item];auto& b=mutablePacket(packets,command);const auto raw=seconds*16;b.at(2)=static_cast<std::uint8_t>(raw>>8);b.at(3)=static_cast<std::uint8_t>(raw);return;}
    if(n==25){std::size_t item;if(!chooseSub({"Shooting mode","Metering mode","Film advance mode","AF mode","Focusing point selection"},item))return;
        if(item==0)chooseCode("Shooting mode",0xcd,0,{{"Program AE",u8(0x10)},{"Shutter-priority AE",u8(0x20)},{"Aperture-priority AE",u8(0x40)},{"Depth-of-field AE",u8(0x08)},{"Manual exposure",u8(0x80)},{"Bulb",u8(0x04)}});
        else if(item==1){auto& b=mutablePacket(packets,0xcd);const auto preserved=static_cast<std::uint8_t>(b.at(3)&0xf9);chooseCode("Metering mode",0xcd,1,{{"Evaluative",u8(preserved|0x04)},{"Partial",u8(preserved|0x00)},{"Spot",u8(preserved|0x02)},{"Center-weighted average",u8(preserved|0x06)}});}
        else if(item==2)chooseCode("Film advance mode",0xcd,2,{{"Single-frame",u8(0x20)},{"Continuous (body only)",u8(0x00)},{"Low-speed continuous",u8(0x10)},{"High-speed continuous",u8(0x30)},{"Ultra-high-speed continuous",u8(0x40)},{"10 sec. self-timer",u8(0x60)},{"2 sec. self-timer",u8(0x50)}});
        else if(item==3)chooseCode("AF mode",0xcd,3,{{"One-Shot AF",u8(0x04)},{"AI Servo AF",u8(0x08)}});
        else {std::size_t selected;if(promptChoice("Setting (q=keep current): ",{"Automatic","Center focusing point"},selected)){auto& b=mutablePacket(packets,0xcd);if(selected==0)b.at(3)|=0x80;else b.at(3)&=0x7f;}}return;}
    if(n==30){chooseCode("Film-ID imprint density",0xcf,0,{{"Dark",u8(0x02)},{"Light",u8(0x00)}});return;}
}
unsigned bcd(std::uint8_t v){if((v>>4)>9||(v&15)>9)throw std::runtime_error("invalid BCD");return (v>>4)*10+(v&15);}
std::array<unsigned,6> validateClock(const std::array<std::uint8_t,6>& raw){const std::array<unsigned,6> value{bcd(raw[0]),bcd(raw[1]),bcd(raw[2]),bcd(raw[3]),bcd(raw[4]),bcd(raw[5])};if(value[1]<1||value[1]>12||value[3]>23||value[4]>59||value[5]>59)throw std::runtime_error("date/time is outside its valid range");static constexpr std::array<unsigned,12> days{31,28,31,30,31,30,31,31,30,31,30,31};auto maximum=days[value[1]-1];const auto year=2000+value[0];if(value[1]==2&&((year%4==0&&year%100!=0)||year%400==0))maximum=29;if(value[2]<1||value[2]>maximum)throw std::runtime_error("day is invalid for the selected month");return value;}
void printClock(const std::vector<open1v::CameraPacket>& p){const auto& f=findPacket(p,0xf3);const std::array<std::uint8_t,6> raw{f.at(2),f.at(3),f.at(4),f.at(5),f.at(6),f.at(7)};const auto value=validateClock(raw);
    std::ostringstream text; text<<"Camera local time = 20"<<std::setfill('0')<<std::setw(2)<<value[0]<<'-'<<std::setw(2)<<value[1]<<'-'<<std::setw(2)<<value[2]<<' '<<std::setw(2)<<value[3]<<':'<<std::setw(2)<<value[4]<<':'<<std::setw(2)<<value[5];
    std::cout<<text.str()<<'\n';}

} // namespace

int main(int argc, char** argv) {
    try {
#ifdef OPEN1V_DEBUG_CLI
        if (argc == 2 && std::string(argv[1]) == "self-test")
            return open1v::runSelfTests();
        if (argc == 3 && std::string(argv[1]) == "frame" &&
            std::string(argv[2]) == "ping") {
            printHex(open1v::encodeFrame({
                static_cast<std::uint8_t>(open1v::MessageType::ping), 1, {}}));
            return 0;
        }
#endif
#ifndef OPEN1V_DEBUG_CLI
        // The normal executable is an interactive settings tool by default.
        // Keep the synthetic argv in static storage because the rest of main
        // reads the command group/action through argv until the session ends.
        static char defaultApp[] = "open1V";
        static char defaultGroup[] = "camera";
        static char defaultAction[] = "console";
        static char* defaultConsoleArgs[] = {defaultApp, defaultGroup, defaultAction};
        if (argc == 1) {
            argc = 3;
            argv = defaultConsoleArgs;
        }
#endif
        if (argc < 3 || argc > 12) { usage(); return argc == 1 ? 0 : 2; }
        std::unique_ptr<open1v::ITransport> transport;
        bool useWinUsb = false;
#ifdef OPEN1V_DEBUG_CLI
        bool allowWriteDebug = false;
#endif
        std::string port;
        std::string debugArgument;
#ifdef OPEN1V_DEBUG_CLI
        std::string blockText, expectedText, valueText;
#endif
        std::string idText, functionText, optionText, enabledText;
        for (int index = 3; index < argc; ++index) {
            const std::string option = argv[index];
            if (option == "--winusb") useWinUsb = true;
#ifdef OPEN1V_DEBUG_CLI
            else if (option == "--allow-write-debug") allowWriteDebug = true;
#endif
            else if (option == "--clock" && index + 1 < argc)
                debugArgument = argv[++index];
            else if (option == "--id" && index + 1 < argc) idText = argv[++index];
            else if (option == "--function" && index + 1 < argc) functionText = argv[++index];
            else if (option == "--option" && index + 1 < argc) optionText = argv[++index];
            else if (option == "--enabled" && index + 1 < argc) enabledText = argv[++index];
#ifdef OPEN1V_DEBUG_CLI
            else if (option == "--block" && index + 1 < argc) blockText = argv[++index];
            else if (option == "--expect" && index + 1 < argc) expectedText = argv[++index];
            else if (option == "--value" && index + 1 < argc) valueText = argv[++index];
#endif
            else if (option == "--port" && index + 1 < argc) {
                const std::string value = argv[++index];
                port = value;
            } else { usage(); return 2; }
        }
        if (useWinUsb && !port.empty()) { usage(); return 2; }
        if (useWinUsb) transport = std::make_unique<open1v::WinUsbTransport>();
        else transport = std::make_unique<open1v::SerialTransport>(port);
        open1v::BridgeClient bridge(*transport);
        const std::string group = argv[1];
        const std::string action = argv[2];
#ifdef OPEN1V_DEBUG_CLI
        if (group == "bridge" && action == "ping") {
            std::cout << bridge.ping() << '\n';
        } else if (group == "bridge" && action == "status") {
            printHex(bridge.status().raw);
        } else if (group == "bridge" && action == "link-status") {
            const auto status = bridge.linkStatus();
            std::cout << "usb_rx_bytes=" << status.usbRxBytes << '\n'
                      << "uart_tx_bytes=" << status.uartTxBytes << '\n'
                      << "uart_rx_bytes=" << status.uartRxBytes << '\n'
                      << "queue_dropped_bytes=" << status.queueDroppedBytes << '\n'
                      << "forwarded_frames=" << status.forwardedFrames << '\n'
                      << "usb_tx_accepted_bytes=" << status.usbTxAcceptedBytes << '\n'
                      << "usb_tx_zero_writes=" << status.usbTxZeroWrites << '\n';
        } else if (group == "camera" && action == "identify") {
            open1v::CameraProtocolSession protocol(bridge);
            const auto packets=protocol.readOnce(open1v::CameraRead::identity);
            const auto& id=findPacket(packets,0xf1);
            std::cout << "type=" << static_cast<unsigned>(id.at(2))
                      << " id=" << static_cast<unsigned>(id.at(3)&0x7f)
                      << " status=0x" << std::hex << static_cast<unsigned>(id.at(4))
                      << std::dec << '\n';
        } else if (group == "camera" && action == "id") {
#else
        if (group == "camera" && action == "id") {
#endif
            open1v::CameraProtocolSession protocol(bridge);
            const auto packets=protocol.readOnce(open1v::CameraRead::identity);
            const auto& id=findPacket(packets,0xf1);
            std::cout<<"Camera model = "<<(id.at(2)==1?"EOS-1V":"unknown")<<"\nCamera ID = "<<unsigned(id.at(3)&0x7f)<<"\nStatus = 0x"<<hexByte(id.at(4))<<'\n';
        } else if (group == "camera" && (action=="cfn"||action=="pfn"||action=="clock")) {
            open1v::CameraProtocolSession protocol(bridge);
            const auto selection = action=="cfn" ? open1v::CameraRead::cfn :
                action=="pfn" ? open1v::CameraRead::pfn : open1v::CameraRead::clock;
            auto packets=protocol.readOnce(selection);
            if(action=="cfn")printCfn(packets);else if(action=="pfn")printPfn(packets);else printClock(packets);
        } else if (group == "camera" && action == "info") {
            open1v::CameraProtocolSession protocol(bridge);
            auto packets=protocol.readOnce(open1v::CameraRead::all);
            const auto& id=findPacket(packets,0xf1);
            std::cout<<"Camera model = "<<(id.at(2)==1?"EOS-1V":"unknown")<<"\nCamera ID = "<<unsigned(id.at(3)&0x7f)<<"\nStatus = 0x"<<hexByte(id.at(4))<<'\n';
            printCfn(packets); printPfn(packets); printClock(packets);
        } else if (group == "camera" && action == "console") {
            open1v::CameraProtocolSession protocol(bridge); bool opened=false;
            try {
                std::vector<open1v::CameraPacket> packets;
                try {
                    packets=protocol.beginSession(); opened=true;
                } catch(const std::exception& e) {
                    std::cerr<<"Camera: not connected or not in PC mode ("<<e.what()<<")\n";
                    return 1;
                }
                const auto& detected=findPacket(packets,0xf1);
                std::cout<<"Camera: "<<(detected.at(2)==1?"EOS-1V":"unknown model")<<"\n";
                auto showClock=[&](const std::vector<open1v::CameraPacket>& source){
                    try { printClock(source); }
                    catch(const std::exception&) {
                        std::cout<<"Camera time response was invalid; F3 raw: ";
                        try { printHex(findPacket(source,0xf3)); } catch(...) { std::cout<<"missing\n"; }
                        std::cout<<"Retrying once...\n";
                        printClock(protocol.perform(open1v::CameraRead::clock));
                    }
                };
                std::cout<<"Camera ID = "<<unsigned(detected.at(3)&0x7f)<<'\n';
                std::string line;
                auto ask=[&](const char* prompt,std::string& answer){std::cout<<prompt; if(!std::getline(std::cin,answer))return false; return answer!="q"&&answer!="Q";};
                auto number=[](const std::string& text){return parseUnsigned(text,0,255,"number");};
                while(std::cout<<"\nCommands: show | cfn | pfn | set id | set clock | exit\n"
                               && std::cout<<"open1V> " && std::getline(std::cin,line)){
                    std::istringstream input(line); std::string command,target,extra; input>>command>>target>>extra;
                    if(command=="exit"||command=="quit")break;
                    try {
                        if(command=="show") { auto p=protocol.perform(open1v::CameraRead::all); printCfn(p);printPfn(p);showClock(p); }
                        else if(command=="cfn"&&target.empty()&&extra.empty()) {
                            std::string value; std::cout<<"C.Fn banks: current, 1, 2, 3\n";
                            if(!ask("Bank (q=back): ",value))continue;
                            open1v::CfnBank bank{}; std::uint8_t bankCommand=0xd1; const char* bankName="Current";
                            if(value=="current")bank=open1v::CfnBank::current;
                            else if(value=="1"){bank=open1v::CfnBank::registered1;bankCommand=0xd5;bankName="Registered 1";}
                            else if(value=="2"){bank=open1v::CfnBank::registered2;bankCommand=0xd7;bankName="Registered 2";}
                            else if(value=="3"){bank=open1v::CfnBank::registered3;bankCommand=0xd9;bankName="Registered 3";}
                            else throw std::runtime_error("bank must be current, 1, 2, or 3");
                            auto original=protocol.perform(open1v::CameraRead::cfn), staged=original;
                            auto mutablePacket=[&](auto& packets,std::uint8_t cmd)->std::vector<std::uint8_t>&{for(auto& packet:packets)if(!packet.bytes.empty()&&packet.bytes[0]==cmd)return packet.bytes;throw std::runtime_error("camera block missing");};
                            bool done=false;while(!done){printCfnBank(staged,bankCommand,bankName,&original);std::cout<<"* marks staged changes\ncfn-edit: enter 1..19, commit, discard, or q\n";if(!ask("cfn-edit> ",value))break;if(value=="discard")break;if(value=="commit"){
                                const auto& before=findPacket(original,bankCommand);const auto& after=findPacket(staged,bankCommand);
                                for(int fn=1;fn<=19;++fn){auto decode=[&](const auto& b){auto x=b.at(2+(fn-1)/2);return oneHot(static_cast<std::uint8_t>(fn==19?x:((fn&1)?x&15:x>>4)));};const auto oldValue=decode(before),newValue=decode(after);if(oldValue!=newValue)protocol.setCfn(bank,static_cast<std::uint8_t>(fn),static_cast<std::uint8_t>(newValue));}
                                std::cout<<"C.Fn changes committed and verified\n";done=true;continue;}
                                const auto n=number(value);if(n<1||n>19)throw std::runtime_error("C.Fn number must be 1..19");std::cout<<"Available options for C.Fn-"<<n<<": 0.."<<(n==19?7:3)<<"\n";if(!ask("Option (q=cancel this item): ",value))continue;const auto option=number(value);open1v::setCfnOption(mutablePacket(staged,bankCommand),static_cast<unsigned>(n),static_cast<unsigned>(option));
                            }
                        } else if(command=="pfn"&&target.empty()&&extra.empty()) {
                            std::vector<open1v::CameraPacket> original;
                            try { original=protocol.perform(open1v::CameraRead::pfn); }
                            catch(const std::exception& first) { std::cout<<"P.Fn read failed ("<<first.what()<<"); retrying in a new logical action...\n";original=protocol.perform(open1v::CameraRead::pfn); }
                            auto staged=original;std::string value;bool done=false;
                            auto& stagedDd=[&]()->std::vector<std::uint8_t>&{for(auto& packet:staged)if(!packet.bytes.empty()&&packet.bytes[0]==0xdd)return packet.bytes;throw std::runtime_error("DD block missing");}();
                            while(!done){printPfn(staged,&original);std::cout<<"* marks staged changes\npfn-edit: enter 1..30, commit, discard, or q\n";if(!ask("pfn-edit> ",value))break;if(value=="discard")break;if(value=="commit"){
                                for(const auto& packet:staged){if(packet.bytes.empty()||packet.bytes[0]==0xd3)continue;const auto& before=findPacket(original,packet.bytes[0]);if(packet.bytes!=before)protocol.setPfnBlock(packet.bytes[0],std::span<const std::uint8_t>(packet.bytes.data()+2,packet.bytes[1]));}std::cout<<"P.Fn changes committed and verified\n";done=true;continue;}
                                const auto n=number(value);if(n<1||n>30)throw std::runtime_error("P.Fn number must be 1..30");std::size_t state;if(!promptChoice("State (q=cancel this item): ",{"ON","OFF"},state))continue;open1v::setPfnEnabled(stagedDd,static_cast<unsigned>(n),state==0);
                                static constexpr std::array<int,13> parameterized{1,2,3,4,5,12,19,20,23,25,27,29,30};if(std::find(parameterized.begin(),parameterized.end(),static_cast<int>(n))!=parameterized.end())editPfnParameters(staged,static_cast<int>(n));
                            }
                        } else if(command=="set"&&extra.empty()&&target=="id") {
                            auto p=protocol.perform(open1v::CameraRead::identity); const auto& currentId=findPacket(p,0xf1); std::cout<<"Current Camera ID = "<<unsigned(currentId.at(3)&0x7f)<<'\n'; std::string value;
                            if(!ask("New ID 0..99 (q=back): ",value))continue; const auto idValue=number(value); if(idValue>99)throw std::runtime_error("camera ID must be 0..99");
                            protocol.setCameraId(static_cast<std::uint8_t>(idValue)); std::cout<<"Verified: Camera ID = "<<idValue<<'\n';
                        } else if(command=="set"&&extra.empty()&&(target=="time"||target=="clock")) {
                            auto p=protocol.perform(open1v::CameraRead::clock); showClock(p); std::string value;
                            auto f=findPacket(p,0xf3); std::array<std::uint8_t,6> v{}; std::copy_n(f.begin()+2,6,v.begin());
                            // Validate before preserving either half; retry once if the
                            // camera returned a semantically invalid but checksummed F3.
                            try { for(const auto x:v)(void)bcd(x); }
                            catch(...) { p=protocol.perform(open1v::CameraRead::clock);f=findPacket(p,0xf3);std::copy_n(f.begin()+2,6,v.begin());for(const auto x:v)(void)bcd(x); }
                            auto applyDigits=[&](const std::string& digits,std::size_t first,std::size_t count){if(digits.size()!=count*2)throw std::runtime_error("wrong number of digits");for(std::size_t i=0;i<count;++i){const char a=digits[i*2],b=digits[i*2+1];if(!std::isdigit(static_cast<unsigned char>(a))||!std::isdigit(static_cast<unsigned char>(b)))throw std::runtime_error("value contains a non-digit");v[first+i]=static_cast<std::uint8_t>(((a-'0')<<4)|(b-'0'));}};
                            std::cout<<"Clock setting:\n1) Sync with system\n2) Set manually\n";
                            if(!ask("Selection (q=back): ",value))continue;
                            if(value=="1") {
                                const auto now=std::time(nullptr); std::tm local{};
#ifdef _WIN32
                                localtime_s(&local,&now);
#else
                                localtime_r(&now,&local);
#endif
                                const auto fullYear=local.tm_year+1900;if(fullYear<2000||fullYear>2099)throw std::runtime_error("EOS-1V clock supports years 2000..2099");
                                const std::array<unsigned,6> d{static_cast<unsigned>(fullYear-2000),static_cast<unsigned>(local.tm_mon+1),static_cast<unsigned>(local.tm_mday),static_cast<unsigned>(local.tm_hour),static_cast<unsigned>(local.tm_min),static_cast<unsigned>(local.tm_sec)};
                                for(std::size_t i=0;i<6;++i)v[i]=static_cast<std::uint8_t>(((d[i]/10)<<4)|(d[i]%10));
                            } else if(value=="2") {
                                if(!ask("Date and time YYMMDDhhmmss (q=back): ",value))continue; applyDigits(value,0,6);
                            } else throw std::runtime_error("selection must be 1 or 2");
                            (void)validateClock(v);auto result=protocol.setClock(v); printClock({result.back()}); std::cout<<"Camera time write verified\n";
                        }
                        else if(!command.empty()) std::cout<<"Unknown command\n";
                    } catch(const std::exception& e) { std::cout<<"Not changed: "<<e.what()<<'\n'; }
                }
                protocol.endSession(); opened=false;
            } catch(...) { try{if(opened)protocol.endSession();}catch(...){} throw; }
        } else if (group == "camera" && action == "set-id") {
            if(idText.empty())throw std::runtime_error("set-id requires --id");
            const auto parsedId=parseUnsigned(idText,0,99,"camera ID");open1v::CameraProtocolSession protocol(bridge); auto packets=protocol.setCameraId(static_cast<std::uint8_t>(parsedId));
            const auto& id=findPacket(packets,0xf1); std::cout<<"Camera ID = "<<unsigned(id.at(3)&0x7f)<<'\n';
        } else if (group == "camera" && action == "set-clock") {
            if(debugArgument.size()!=12)throw std::runtime_error("set-clock requires --clock YYMMDDhhmmss");
            std::array<std::uint8_t,6> value{}; for(std::size_t i=0;i<6;++i){if(debugArgument[i*2]<'0'||debugArgument[i*2]>'9'||debugArgument[i*2+1]<'0'||debugArgument[i*2+1]>'9')throw std::runtime_error("clock contains a non-digit");value[i]=static_cast<std::uint8_t>(((debugArgument[i*2]-'0')<<4)|(debugArgument[i*2+1]-'0'));}
            (void)validateClock(value);open1v::CameraProtocolSession protocol(bridge); auto packets=protocol.setClock(value); printClock(packets);
#ifdef OPEN1V_DEBUG_CLI
        } else if (group == "camera" && action == "read-settings") {
            open1v::CameraProtocolSession protocol(bridge);
            const auto packets=protocol.readOnce(open1v::CameraRead::settings);
            const auto& id=findPacket(packets,0xf1); const auto& e8=findPacket(packets,0xe8);
            const auto& fc=findPacket(packets,0xfc); const auto& e1=findPacket(packets,0xe1);
            const auto rolls=(static_cast<unsigned>(e1.at(2))<<8)|e1.at(3);
            std::cout << "type=" << static_cast<unsigned>(id.at(2))
                      << " id=" << static_cast<unsigned>(id.at(3)&0x7f)
                      << " status=0x" << std::hex << static_cast<unsigned>(id.at(4))
                      << std::dec << " rolls=" << rolls << '\n';
            std::cout << "E8 "; printHex(e8);
            std::cout << "FC "; printHex(fc);
            std::cout << "E1 "; printHex(e1);
        } else if (group == "camera" &&
                   (action == "pfn-write-debug" || action == "cfn-write-debug")) {
            if (blockText.empty() || expectedText.empty() || valueText.empty())
                throw std::runtime_error("write debug requires --block, --expect and --value");
            const auto block = parseHex(blockText);
            if (block.size() != 1) throw std::runtime_error("block must be one byte");
            const auto expected = parseHex(expectedText);
            const auto value = parseHex(valueText);
            open1v::CameraProtocolSession protocol(bridge);
            const auto packets = action == "pfn-write-debug"
                ? protocol.writePfn(block[0], expected, value, allowWriteDebug)
                : protocol.writeCfn(block[0], expected, value, allowWriteDebug);
            for (const auto& packet : packets) {
                std::cout << packet.label << ' '; printHex(packet.bytes);
            }
        } else if (group == "camera" && action.ends_with("-debug")) {
            open1v::CameraProtocolSession protocol(bridge);
            for (const auto& packet : protocol.runDiagnostic(action, allowWriteDebug, debugArgument)) {
                std::cout << packet.label << ' ';
                printHex(packet.bytes);
            }
#endif
        } else {
            usage(); return 2;
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
