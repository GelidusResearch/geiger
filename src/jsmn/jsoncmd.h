#define		ACK_DATA	0
#define		GET_IP		1
#define		ACK_IP		2
#define		GET_TIME	3
#define		ACK_TIME	4
#define		GET_DATE	5
#define		ACK_DATE	6
#define		GET_SSID	7
#define		ACK_SSID	8
#define		GET_CONFIG	9
#define		ACK_CONFIG	10
#define		GET_ESP		11
#define		ACK_ESP		12


char cmdStr[56];
extern uint8_t cmdq;

  // get string label for various json cmds
  char* jcmd(uint8_t jcmd) {


	  switch (jcmd) {
		case 0:
			strcpy_P(cmdStr, PSTR("{\"error\":0}"));
			break;
		case 1:
			strcpy_P(cmdStr, PSTR("{\"get\":\"ip\"}"));
			break;
		case 2:
			strcpy_P(cmdStr, PSTR("{\"ack\":\"ip\"}"));
			break;
		case 3:
			strcpy_P(cmdStr, PSTR("{\"get\":\"time\"}"));
			break;
		case 4:
			strcpy_P(cmdStr, PSTR("{\"ack\":\"time\"}"));
			break;
		case 5:
			strcpy_P(cmdStr, PSTR("{\"get\":\"date\"}"));
			break;
		case 6:
			strcpy_P(cmdStr, PSTR("{\"ack\":\"date\"}"));
			break;
		case 7:
			strcpy_P(cmdStr, PSTR("{\"get\":\"ssid\"}"));
			break;
		case 8:
			strcpy_P(cmdStr, PSTR("{\"ack\":\"ssid\"}"));
			break;
		case 9:
			strcpy_P(cmdStr, PSTR("{\"get\":\"config\"}"));
			break;
		case 10:
			strcpy_P(cmdStr, PSTR("{\"ack\":\"config\"}"));
			break;
		case 11:
			strcpy_P(cmdStr, PSTR("{\"get\":\"esp\"}"));
			break;
		case 12:
			strcpy_P(cmdStr, PSTR("{\"ack\":\"esp\"}"));
			break;
		  default: break;
	  }
	  return cmdStr;
  }
