#ifndef _KEYBOARD_DEVICE_H__
#define _KEYBOARD_DEVICE_H__
#include<map>
namespace keyboard_device{
     int initDevice();
     int startRead();
     void refreshTimes();
     int stopRead();
     int deinitDevice();
     int getKeyTimes(int key);
     void readFromDevice();
     const int MY_KEY_POWER = 'p';
     const int MY_KEY_RETURN = 'r';
     const int MY_KEY_MENU = 'm';
     const int MY_KEY_UP = 'w';
     const int MY_KEY_DOWN = 's';
     const int MY_KEY_LEFT = 'a';
     const int MY_KEY_RIGHT = 'd';


    const int MY_KEY_TEST = 't';
    const int MY_KEY_OK = 'o';
    const int MY_KEY_DELETE = 'x';

    const int MY_KEY_1 = '1';
    const int MY_KEY_2 = '2';
    const int MY_KEY_3 = '3';
    const int MY_KEY_4 = '4';
    const int MY_KEY_5 = '5';
    const int MY_KEY_6 = '6';
    const int MY_KEY_7 = '7';
    const int MY_KEY_8 = '8';
    const int MY_KEY_9 = '9';
    const int MY_KEY_0 = '0';
}

#endif // _IR_DEVICE_H__
