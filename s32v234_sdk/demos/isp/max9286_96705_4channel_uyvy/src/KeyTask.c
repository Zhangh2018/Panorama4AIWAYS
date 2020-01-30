#include <stdio.h>
#include <linux/input.h>
#include <fcntl.h>
#include <sys/time.h>
#include <unistd.h>
#include "ImageStitching.h"
#include "uart_to_mcu.h"

//void keyboard_test()
extern int parking_mode_select;
extern int parking_mode_ok;
extern int parking_mode_ok_flag;

/////////////////////////////////////wyf added//////////////////////////////////
#if 0
extern int Parking_Select_Num ; //parking place selected
extern int Car_Parking_Status ;
extern int Parking_Place_Mode_Select;
extern int Parking_Place_Mode_Select_OK;

extern int Parking_Place_Mode_Left_Vert_Select;
extern int Parking_Place_Mode_Rght_Vert_Select;
extern int Parking_Place_Mode_Left_Vert_Select_Ok;
extern int Parking_Place_Mode_Right_Vert_Select_Ok;
extern int SwitchChannelNum;
extern int Total_Parking_Num;
extern TARGETPLACEDATA ToMcuTargetData;
extern int Car_Speed_Flag;
extern int AvmVideoImageSwitchFlag;
extern int Mcu_Send_Parking_Data_Fag;
#endif 
//extern 
int SwitchChannelNum;
//extern 
float yangjiao,xuanzhuan;


//////////////////////////////////////////////////////////////////////////////
void *KeyTask(void *ptr1)
{
    int fd=open("/dev/input/event0",O_RDWR);
    if( fd <= 0 )
    {
        printf("Can not open keyboard input file\n");
    }
   printf("This is a button  KeyTask\n");
 //  struct input_event *event;
    char buf[128] = {0};
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
	cha16 = 2;
    while(1)
    {
        int ret = select(fd + 1,&rfds, NULL,NULL,NULL);
        if(ret < 0)
            continue;
        if( FD_ISSET(fd, &rfds) )
        {
            int readn = read(fd, buf, sizeof(struct input_event));
            if (readn <= 0)
            {
                printf("uart read error %d\n", readn);
                continue;
            }
            struct input_event *key_event=(struct input_event*)buf;
            if(key_event->type == EV_KEY)      //ok
            {
                if((key_event->code == 59) && !key_event->value)
                {
                	
					////////////////////////wyf added//////////////////////////////////////////
					xuanzhuan=xuanzhuan+30*3.141592654 / 180.0;
					printf("key16--mode: parking_mode_ok\n");

					///////////////////////////////////////////////////////////////////////////
				}
				else if((key_event->code == 60) && !key_event->value) //mode select
				{
					////////////////////////////////////////////////////////////////////////
					#if 1
					
					//if(Car_Parking_Status <1 || Car_Parking_Status>3)
					{
						SwitchChannelNum++;
						if(SwitchChannelNum>12)
						SwitchChannelNum = 0;
						printf("key 17::SwitchChannelNum==%d\n",SwitchChannelNum);
					}
					#endif
					if (SwitchChannelNum<8)
						xuanzhuan = 0;
					printf("key17--Mode_key\n");
					//////////////////////////////////////////////////////////////////////
					
				}
				else if((key_event->code == 61) && !key_event->value) //clean
				{
					printf("key18--down\n");
					/////////////////////////////////////////////////////////////////////////
				}
           }
        }
		usleep(50000);
    }
}

