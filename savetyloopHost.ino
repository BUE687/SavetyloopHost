/*
  UDPSendReceive.pde:
  This sketch receives UDP message strings, prints them to the serial port
  and sends an "acknowledge" string back to the sender

  A Processing sketch is included at the end of file that can be used to send
  and received messages for testing with a computer.

  created 21 Aug 2010
  by Michael Margolis

  This code is in the public domain.

  adapted from Ethernet library examples
*/

//#define DEBUG

#define IN_RECHTS   12
#define IN_LINKS    4
#define OUT_RECHTS  15
#define OUT_LINKS   2
#define SI_SCHLEIFE 14

#include <Ticker.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#ifndef STASSID
#define STASSID "Astronautenarena"
#define STAPSK  "32985205082596571661"
#endif

unsigned int localPort = 8888;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1]; //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged\r\n";       // a string to send back

Ticker blinker;

enum BLSTATE {
  aus = 0,
  ein = 1,
  bli = 2,
} blink_state;

WiFiUDP Udp;

#define ip_cnt_max 5
IPAddress ip_addrs[ip_cnt_max];
uint16_t ip_ports[ip_cnt_max];
bool activ[ip_cnt_max];
#define rep_time 7
int count_down[ip_cnt_max];

IPAddress broadcastIP(255, 255, 255, 255);


unsigned long prevMillis = 0;
const long interval = 1000;

bool is_registered(IPAddress ip_addr, uint16_t ip_port) {
  return ip_id(ip_addr, ip_port) != -1; 
}

int ip_id(IPAddress ip_addr, uint16_t ip_port) {
  for (int ii = 0; ii < ip_cnt_max; ii++) {
    if (activ[ii]) {
      if (ip_addr == ip_addrs[ii] && ip_port == ip_ports[ii]) {
        return ii;
      }
    }
  }
  return -1;
}

int register_ip(IPAddress ip_addr, uint16_t ip_port) {
  if (is_registered(ip_addr, ip_port)) return 0;
  for (int ii = 0; ii < ip_cnt_max; ii++) {
    if (!activ[ii]) {
      ip_addrs[ii] = ip_addr;
      ip_ports[ii] = ip_port;
      count_down[ii] = -1;
      activ[ii] = true;
      return ii;
    }
  }
  return -1;
}

void clear_ip_register() {
  for (int ii = 0; ii < ip_cnt_max; ii++) {
    activ[ii] = false;
    count_down[ii] = 0;
  }
}

void send_msg(IPAddress ip_addr, uint16_t ip_port, bool control, bool time_out, bool usr_halt, bool user_notc, bool state_rechts, bool state_links, bool cmd_rechts, bool cmd_links) {
  Udp.beginPacket(ip_addr, ip_port);
  if (control) {
    Udp.write("SL: OK   ");
  } else {
    Udp.write("SL: STOP ");
  }
  Udp.write("\t");
  
  if (usr_halt) {
    Udp.write("RM: HALT     ");
  } else if (time_out) {
    Udp.write("RM: TIME OUT ");
  } else if (user_notc) {
    Udp.write("RM: NOT CON  ");
  } else {
    Udp.write("RM: OK       ");
  }
  Udp.write("\t");
  
  if (state_rechts) {
    Udp.write("MR in: ON  ");
  } else {
    Udp.write("MR in: OFF ");
  }
  if (cmd_rechts) {
    Udp.write("out: ON  ");
  } else {
    Udp.write("out: OFF ");
  }
  Udp.write("\t");
  
  if (state_links) {
    Udp.write("ML in: ON  ");
  } else {
    Udp.write("ML in: OFF ");
  }  
  if (cmd_links) {
    Udp.write("out: ON  ");
  } else {
    Udp.write("out: OFF ");
  }
  Udp.write("\t");
  
  Udp.write("Remote: ");
  for (int ii = 0; ii < ip_cnt_max; ii++) {
    Udp.write("C");
    Udp.write(String(ii).c_str());
    Udp.write(": ");
    if (activ[ii]) {
      Udp.write(String(count_down[ii]).c_str());
    } else {
      Udp.write("CLR ");
    }
    Udp.write("\t");
  }
  Udp.write('\n');
  Udp.endPacket();
}

void change_state() {
  if(blink_state == aus)
    digitalWrite(LED_BUILTIN, HIGH);
  else if(blink_state == ein)
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  else if(blink_state == bli)
    digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  digitalWrite(OUT_RECHTS, LOW);
  digitalWrite(OUT_LINKS, LOW);
  pinMode(IN_LINKS, INPUT);
  pinMode(IN_RECHTS, INPUT);
  pinMode(OUT_RECHTS, OUTPUT);
  pinMode(OUT_LINKS, OUTPUT);
  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SI_SCHLEIFE, INPUT);
//#ifdef DEBUG
//  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);
  while (WiFi.status() != WL_CONNECTED) {
    //Serial.print('.');
    delay(500);
  }
//  Serial.print("Connected! IP address: ");
//  Serial.println(WiFi.localIP());
//  Serial.printf("UDP server on port %d\n", localPort);
//#endif
  Udp.begin(localPort);
  blink_state = bli;
  blinker.attach(0.5, change_state);

  for(int ii = 0; ii < ip_cnt_max; ii++)
    activ[ii] = false; 
}

void loop() {
  bool control = !digitalRead(SI_SCHLEIFE);
  bool state_rechts = digitalRead(IN_RECHTS);
  bool state_links = digitalRead(IN_LINKS);

  unsigned long currentMillis = millis();

  bool time_out = false;
  bool remote_stop = false;
  bool user_halt = false;
  bool user_notc = true;
  for(int ii = 0; ii < ip_cnt_max; ii++) {
    if(activ[ii]) {
      user_notc = false;
      if (count_down[ii] < 0) {
        user_halt = true;
      } else if (count_down[ii] == 0) {
        time_out = true;
      }
    }
  }

  bool control_cum = control && !user_halt && !remote_stop && !time_out && !user_notc;
  bool cmd_rechts = (state_rechts && control_cum);
  bool cmd_links = (state_links && control_cum);
  digitalWrite(OUT_RECHTS, cmd_rechts);
  digitalWrite(OUT_LINKS, cmd_links);
//#ifndef DEBUG
//  digitalWrite(LED_BUILTIN, control_cum);
//#endif
  
  if(currentMillis - prevMillis >= interval) {
    prevMillis = currentMillis;
    for(int ii = 0; ii < ip_cnt_max; ii++) {
      if(activ[ii]) {
        send_msg(ip_addrs[ii], ip_ports[ii], control, time_out, user_halt, user_notc, state_rechts, state_links, cmd_rechts, cmd_links);
      }
    }
    for(int ii = 0; ii < ip_cnt_max; ii++) {
      if(activ[ii]) {
        if (count_down[ii] > 0) {
          count_down[ii]--;
        }
      }
    }
    if(user_notc) {
      Udp.beginPacket(broadcastIP, 8888);
      Udp.write("HELO");
      Udp.endPacket();
    }
  }
  
  //digitalWrite(LED_BUILTIN, !control);
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
//    Serial.printf("Received packet of size %d from %s:%d\n    (to %s:%d, free heap = %d B)\n",
//                  packetSize,
//                  Udp.remoteIP().toString().c_str(), Udp.remotePort(),
//                  Udp.destinationIP().toString().c_str(), Udp.localPort(),
//                  ESP.getFreeHeap());
//

    // read the packet into packetBufffer
    int n = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[n] = 0;
//    Serial.println("Contents:");
//    Serial.println(packetBuffer);
    int ii = ip_id(Udp.remoteIP(), Udp.remotePort());

//    // send a reply, to the IP address and port that sent us the packet we received
//    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
//    Udp.write(ReplyBuffer);
//    if (is_registered(Udp.remoteIP(), Udp.remotePort()))
//      Udp.write("Known\n");
//    Udp.endPacket();
    register_ip(Udp.remoteIP(), Udp.remotePort());
    if (packetBuffer[0] == 'H')
       count_down[ii] = -1;
    else if (packetBuffer[0] == 'R')
       clear_ip_register();
    else
       count_down[ii] = rep_time;
  }

}

/*
  test (shell/netcat):
  --------------------
	  nc -u 192.168.esp.address 8888
*/
