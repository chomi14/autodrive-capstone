// Host-only test. Fake Arduino I/O; never opens hardware.
#include <cassert>
#include <algorithm>
#include <climits>
#include <string>
#include <cstdlib>
using byte = unsigned char;
using std::min;
using std::max;
constexpr int A2=16, INPUT=0, OUTPUT=1, LOW=0;
unsigned long fake_time=0;
int pwm[20]={}, adc=522;
struct FakeSerial {
  void begin(int) {}
  int available() { return 0; }
  int read() { return 0; }
  template<class T> void print(T) {}
  template<class T> void println(T) {}
} Serial;
unsigned long millis() { return fake_time; }
void delay(unsigned long n) { fake_time+=n; }
void pinMode(int,int) {}
void analogWrite(int pin,int value) { pwm[pin]=value; }
int analogRead(int) { return adc; }
template<class T> T constrain(T x,T lo,T hi) { return std::min(std::max(x,lo),hi); }
long map(long x,long a,long b,long c,long d) { return (x-a)*(d-c)/(b-a)+c; }
#undef MAX_INPUT
#ifdef TEST_LEGACY
#include "../../src/control/driving/driving.ino"
#define last_command_time lastValidCommandTime
#else
#include "../../src/control/driving_user_pins/driving_user_pins.ino"
#endif
void stopped() { for (int pin : {2,3,4,5,6,7}) assert(pwm[pin]==0); }
void line(const std::string& s) { for (char c:s) processIncomingByte(c); processIncomingByte('\n'); }
int main() {
  setup(); adc=600; loop(); stopped(); // Boot must not center by itself.
  processData("?"); loop(); stopped();
#ifndef TEST_LEGACY
  for (auto endpoints : {std::pair<int,int>{600,445}, {445,600}}) {
    int l=endpoints.first, r=endpoints.second, c=510;
    assert(potToStep(l,l,c,r)==-7);
    assert(potToStep(c,l,c,r)==0);
    assert(potToStep(r,l,c,r)==7);
    assert(potToStep((l+c)/2,l,c,r)<0);
    assert(potToStep((r+c)/2,l,c,r)>0);
  }
#endif
  fake_time+=50;
  line("s7l100r-100"); loop(); assert(command_active); assert(pwm[7]==100 && pwm[5]==100);
  fake_time+=499; loop(); assert(command_active);
  auto last=last_command_time;
  for (auto s : {"garbage", "s0l0r0junk", "s0l99999999999999r0", "s0l0", "s8l0r0"}) line(s);
  line("?"); // Queries cannot extend a drive command's lifetime.
  line(std::string("s0l0r0")+std::string(80,'0'));
  line(std::string("s0l0r0\0junk", 11));
  assert(last_command_time==last);
  fake_time+=1; loop(); stopped(); assert(!command_active);
  fake_time+=100; loop(); stopped();
  line("s-7l-80r80"); loop(); assert(command_active);
  line("X"); loop(); stopped(); assert(!command_active);
#ifndef TEST_LEGACY
  line("C"); loop(); assert(calibration_running);
  auto cal_last=last_command_time;
  fake_time+=100; line("s1l10r10");
  assert(last_command_time==cal_last); // Blocked drive frames cannot feed calibration.
  fake_time+=400; line("s0l0r0"); loop(); assert(calibration_running);
  fake_time+=500; loop(); stopped(); assert(!calibration_running);
  calibration_waiting_apply=true;
  // Current preserved 440/355/270 baseline: integer interpolation gives 533.
  line("K610,455"); assert(runtime_center==533); assert(!command_active);
  calibration_waiting_apply=true;
  line("K1junk,900"); assert(calibration_waiting_apply);
  line("X");
#endif
  fake_time+=50; line("s7l30r30"); loop(); assert(command_active);
  fake_time+=450; line("s7l30r30"); loop(); assert(command_active);
  fake_time+=499; loop(); assert(command_active); // Identical valid frames feed watchdog.
  fake_time+=1; loop(); stopped(); assert(!command_active);
  // The bridge repeats the last valid frame at 20 Hz while its independent
  // controller health lease is alive. Both firmware variants must stay active
  // during slow inference, retaining X and serial-loss watchdog stops.
  for (int tick=0; tick<200; ++tick) {
    fake_time+=50; line("s3l80r80"); loop();
    assert(command_active); assert(pwm[7]==80 && pwm[4]==80);
  }
  line("X"); loop(); stopped(); assert(!command_active);
  line("s3l80r80"); loop();
  fake_time+=500; loop(); stopped(); assert(!command_active);
  fake_time=ULONG_MAX-100; line("s1l20r20");
  fake_time=398; loop(); assert(command_active);
  fake_time=399; loop(); stopped();
}
