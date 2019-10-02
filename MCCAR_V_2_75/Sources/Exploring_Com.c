/*
 * Exploring_Com.c
 *
 *  Created on: 01.05.2019
 *      Author: cc-isn
 */
#include "CLS_BLE.h"
#include "Exploring_Com.h"
#include <stdlib.h>


//static char dataBuffer[400] = "$6;#;1;-180;#;2;6;0,0;0,0;0,0;0,0;0,0;0,0;#;3;3;1,1;1,1;1,1;#;4;-90;#;5;2;0,0;0,0;#;6;180;$255;";
 Maze_segments MazeSegmentsToBeDriven;
 bool msgFinishFlag = FALSE;


 Maze_segments getReferenceOfMazesegment(void){
	 return MazeSegmentsToBeDriven;
 }

uint8_t TEST_BLE_PrintStatus(CLS_BLE_ConstStdIOType *io)
{
	CLS_BLE_SendStatusStr((const unsigned char*)"Status", (const unsigned char*)"\r\n", io->stdOut);
	CLS_BLE_SendStatusStr((const unsigned char*)"  Build", (const unsigned char*)__DATE__, io->stdOut);
	CLS_BLE_SendStr((unsigned char*)" ", io->stdOut);
	CLS_BLE_SendStr((unsigned char*)__TIME__, io->stdOut);
	CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
  return ERR_OK;
}



uint8_t TEST_BLE_ParseCommand(const uint8_t *cmd, bool *handled, CLS_BLE_ConstStdIOType *io)
{
  if (UTIL1_strcmp((char*)cmd, CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, "TEST help")==0) {
	  CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)CLS_BLE_DASH_LINE, io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)"MCCAR V2.75", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)CLS_BLE_DASH_LINE, io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	  CLS_BLE_SendHelpStr((unsigned char*)"Maze data", (const unsigned char*)"Send me the track informations\r\n", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)CLS_BLE_DASH_LINE, io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)"Data should have the cluster of:", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)"$number of segments; ", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)";#;segment number;", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)"number of filds for this segment;", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)"wall right(bool), wall left(bool);.....;", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)"$255;", io->stdOut);
	  CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);

    *handled = TRUE;
    return ERR_OK;

  } else if ((UTIL1_strcmp((char*)cmd, CLS1_CMD_STATUS)==0)) {
    *handled = TRUE;

    return TEST_BLE_PrintStatus(io);
  } else if ((UTIL1_strFind((char*)cmd, CLS1_CMD_MSG_START)==0)) {
	    *handled = TRUE;
	    getNumberOfMazeSegments(&MazeSegmentsToBeDriven,cmd);
	    if(msgFinishFlag){

	    	CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	    	CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	    	CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	    	CLS_BLE_SendStr((unsigned char*)CLS_BLE_DASH_LINE, io->stdOut);
	    	CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	    	CLS_BLE_SendStr((unsigned char*)"Received the track info\r\n", io->stdOut);
	    	CLS_BLE_SendStr((unsigned char*)CLS_BLE_DASH_LINE, io->stdOut);
	    	WAIT1_Waitms(1000);
	    }
	    else{

	  	  CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	  	  CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	  	  CLS_BLE_SendStr((unsigned char*)"\r\n", io->stdOut);
	  	  CLS_BLE_SendStr((unsigned char*)CLS_BLE_DASH_LINE, io->stdOut);
	  	  CLS_BLE_SendStr((unsigned char*)"Wrong Data! Please resend the Track Data", io->stdOut);
	  	  CLS_BLE_SendStr((unsigned char*)CLS_BLE_DASH_LINE, io->stdOut);
	    }

	    return ERR_OK;
  }

  return ERR_OK; /* no error */
}


static const CLS_BLE_ParseCommandCallback CmdParserTable[] =
{
  TEST_BLE_ParseCommand,
  NULL /* sentinel */
};



void testUART()
{
	  unsigned char buf[600];
	  memset(buf, '\0', sizeof(buf));
	  WAIT1_Waitms(2000);
	  while(!msgFinishFlag){
		  (void)CLS_BLE_ReadAndParseWithCommandTable(buf, sizeof(buf), CLS_BLE_GetStdio(), CmdParserTable);
		  WAIT1_Waitms(50);
		  //memset(buf, '\0', sizeof(buf));
	  }
	  msgFinishFlag = FALSE;
	  memset(buf, '\0', sizeof(buf));
}


void getNumberOfMazeSegments(Maze_segments *MazeSegments,const char *buffer) {
		  char *pch0;
		  char *pch1;
		  char *string;
		  uint8_t sizeOfString = 0;
	  	  uint8_t numberOfSegments = 0;
	  	  uint8_t endCondition = 0;
	  	  char src[4];

	  	 // pch1 = buffer;
		  pch0=strchr(buffer,'$');
		  pch1=strchr(pch0,';');

		  sizeOfString = (pch1-pch0)-1;
		  memset(src, '\0', sizeof(src));
		  strncpy(src,(pch0+1),sizeOfString);
		  numberOfSegments = atoi(src);
		  MazeSegments->numberOfSegments = numberOfSegments;
		  pch0 = getMazeSegment(MazeSegments,pch1,numberOfSegments);

		  pch0=strchr(pch0,'$');
		  pch1=strchr(pch0,';');
		  memset(src, '\0', sizeof(src));
		  strncpy(src,pch0+1,pch1-pch0-1);
		  endCondition=atoi(src);
		  if (255 == endCondition){
			  msgFinishFlag = TRUE;
		  }
}


char* getMazeSegment(Maze_segments *MazeSegments,char *buffer, uint8_t numberOfSegments) {
  	  char * pch0;
  	  char * pch1;
  	  pch1 = buffer;
  	  uint8_t sizeOfString = 0;
	  for(uint8_t i =0; i<numberOfSegments;i++){
		  WAIT1_Waitms(1);
		  char * string;
		  pch0=strchr(pch1,'#');
		  pch0=strchr(pch0,';');
		  pch1=strchr(pch0+1,';');
		  char src[4];

		  sizeOfString = (pch1-pch0)-1;
		  memset(src, '\0', sizeof(src));
		  strncpy(src,(pch0+1),sizeOfString);
		  int16 segmentNumber = atoi(src);


		  pch0=strchr(pch1,';');
		  pch1=strchr(pch0+1,';');
		  sizeOfString = (pch1-pch0)-1;
		  memset(src, '\0', sizeof(src));
		  strncpy(src,(pch0+1),sizeOfString);

		  int16 numberOfFields = atoi(src);

		  if((numberOfFields == -90)||(numberOfFields == 90)||(numberOfFields == -180)||(numberOfFields == 180)){
			  MazeSegments->segments[segmentNumber-1].SingleSegment = numberOfFields;
		  }
		  else{
			  for(uint8_t j =0; j<numberOfFields;j++){
				  WAIT1_Waitms(10);
				  memset(src, '\0', sizeof(src));
				  strncpy(src,pch1+(1),1);
				  bool wallRight=atoi(src);

				  memset(src, '\0', sizeof(src));
				  strncpy(src,pch1+(3),1);
				  bool wallLeft=atoi(src);
				  pch1=strchr(pch1+3,';');

				  MazeSegments->segments[segmentNumber-1].SingleSegment = numberOfFields;
				  MazeSegments->segments[segmentNumber-1].rightWallAvailability[j] = wallRight;
				  MazeSegments->segments[segmentNumber-1].leftWallAvailability[j] = wallLeft;
			  }
		  }
		  //memset(string, 0, 4);
	  }

	  return (pch1);
}








