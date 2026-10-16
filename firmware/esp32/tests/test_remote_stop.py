"""Host regression of actual stop handlers; no ESP32 or motor access."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1] / 'traction_control_v4_2_bounded_motion_api'

def function(source, name):
    start = source.index('void ' + name + '(')
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]

def main():
    source = (ROOT / (ROOT.name + '.ino')).read_text()
    fixture = r'''
#include <cassert>
#include <string>
#include "remote_stop_observer.h"
using String = std::string;
enum class MotionState { READY, BRAKING, FAULT };
enum class CommandState { ACCEPTED, BRAKING, FAULT, STOPPED };
enum class BoundedStopMode { ACTIVE_BRAKE };
struct CommandRecord {
 int rightFinalCount=0,leftFinalCount=0,rightCountAtBrakeStart=0,leftCountAtBrakeStart=0;
 BoundedStopMode stopMode{}; bool reobserveRequired=false;
};
RemoteStopObserver remoteStop;
uint32_t clockMs=0, motionStartTime=0, motionElapsedMs=0;
int rightEncoder=0,leftEncoder=0,rightCountAtStop=0,leftCountAtStop=0;
float rightSpeedMps=1,leftSpeedMps=1,profileSpeedMps=1;
bool startRequested=false,squareRequested=false,stopRequested=false;
bool square=false,haveCommand=true,boundedFaultLatched=false,activeBoundedFaultPending=false;
String boundedFaultReason,faultReason;
MotionState motionState=MotionState::READY;
CommandState commandState=CommandState::ACCEPTED;
CommandRecord record;
int output=0,terminalCalls=0;
struct Server {int code=0; String body; void send(int c,const char*,const char* b){code=c;body=b;}} server;
uint32_t millis(){return clockMs;}
CommandRecord* activeCommand(){return haveCommand?&record:nullptr;}
bool motionIsActive(){return motionState==MotionState::BRAKING;}
bool squareIsActive(){return square;}
void failSquareSequence(const String&){square=false;output=0;}
void brakeMotors(){output=255;}
void stopMotors(){output=0;}
void updateDistancesFromEncoders(){}
void readMotionCounts(int32_t& r,int32_t& l){r=rightEncoder;l=leftEncoder;}
void setActiveCommandState(CommandState s){commandState=s;}
void completeActiveCommand(CommandState s){commandState=s;haveCommand=false;++terminalCalls;}
'''
    fixture += '\n' + function(source, 'updateRemoteStop') + '\n' + function(source, 'handleStop')
    fixture += r'''
void reset(){remoteStop={};clockMs=0;rightEncoder=leftEncoder=0;terminalCalls=0;
 boundedFaultLatched=activeBoundedFaultPending=false;faultReason=boundedFaultReason="";
 haveCommand=true;motionState=MotionState::READY;startRequested=squareRequested=stopRequested=false;square=false;}
void tick(uint32_t t){clockMs=t;updateRemoteStop();}
int main(){
 reset();startRequested=true;squareRequested=true;square=true;handleStop();
 assert(output==255 && remoteStop.pending && terminalCalls==0 && server.code==202);
 assert(!startRequested && !squareRequested && !square);
 clockMs=200;handleStop();assert(remoteStop.started_ms==0);
 tick(400);tick(401);tick(499);assert(terminalCalls==0);
 tick(500);assert(terminalCalls==0);tick(600);
 assert(terminalCalls==1 && commandState==CommandState::STOPPED && output==0);
 assert(record.reobserveRequired && remoteStop.complete);
 handleStop();assert(server.code==200 && terminalCalls==1);
 reset();handleStop();tick(400);rightEncoder=1;tick(500);tick(600);
 assert(terminalCalls==0);tick(700);assert(terminalCalls==1 && record.rightFinalCount==1);
 reset();handleStop();for(uint32_t t=400;t<=2000;t+=100){++rightEncoder;tick(t);}
 assert(remoteStop.pending && remoteStop.timed_out && boundedFaultLatched && output==255);
 tick(2100);tick(2200);assert(commandState==CommandState::FAULT && terminalCalls==1);
 reset();boundedFaultLatched=true;faultReason="PRIOR_FAULT";handleStop();tick(400);tick(500);tick(600);
 assert(commandState==CommandState::FAULT && boundedFaultLatched);
 // uint32 rollover and continued movement reset the quiet observation.
 RemoteStopObserver o;o.begin(UINT32_MAX-200,4,7);
 assert(!o.update(199,4,7));assert(!o.update(299,4,7));assert(o.update(399,4,7));
}
'''
    with tempfile.TemporaryDirectory() as tmp:
        cpp = Path(tmp) / 'test.cpp'
        cpp.write_text(fixture)
        exe = Path(tmp) / 'test'
        subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I',str(ROOT),str(cpp),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
    print('PASS: actual stop handlers, delayed terminal, repeat, queued start/square cancellation, moving encoders, timeout/fault, rollover')

if __name__ == '__main__':
    main()
