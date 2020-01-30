#include "keyboardInput.h"
#include <stdio.h>
#include <pthread.h>
#include <linux/input.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <memory.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <termios.h>
#include <unistd.h>



namespace keyboard_device
{
static std::map<int,int> keyTimes;
static bool g_isRun;
void* readInputThread(void* arg);
static pthread_mutex_t g_times_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t g_thread;
int set_fl(int fd,int flags);
static struct termios ori_attr, cur_attr;

static
int tty_reset(void)
{
    if (tcsetattr(STDIN_FILENO, TCSANOW, &ori_attr) != 0)
        return -1;

    return 0;
}


static
int tty_set(void)
{

    if ( tcgetattr(STDIN_FILENO, &ori_attr) )
        return -1;
    memcpy(&cur_attr, &ori_attr, sizeof(cur_attr) );
    cur_attr.c_lflag &= ~ICANON;
    cur_attr.c_lflag &= ~ECHO;
    cur_attr.c_cc[VMIN] = 1;
    cur_attr.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &cur_attr) != 0)
        return -1;

    return 0;
}

static
int kbhit(void)
{

    fd_set rfds;
    struct timeval tv;
    int retval;

    /* Watch stdin (fd 0) to see when it has input. */
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    /* Wait up to five seconds. */
    tv.tv_sec  = 0;
    tv.tv_usec = 0;

    retval = select(1, &rfds, NULL, NULL, &tv);
    /* Don't rely on the value of tv now! */

    if (retval == -1)
    {
        perror("select()");
        return 0;
    }
    else if (retval)
        return 1;
    /* FD_ISSET(0, &rfds) will be true. */
    else
        return 0;
    return 0;
}
int initDevice()
{
    startRead();
    return 0;
}
int getKeyTimes(int key)
{
    pthread_mutex_lock(&g_times_mutex);
    int times = keyTimes[key];
    keyTimes[key] = 0;
    pthread_mutex_unlock(&g_times_mutex);
    return times;
}
int set_fl(int fd,int flags)
{
    int val;
    if ((val = fcntl(fd, F_GETFL, 0)) < 0)
    {
        printf("fcntl F_GETFL error");
        return -1;
    }
    val |= flags;        /* turn on flags */
    if (fcntl(fd, F_SETFL, val) < 0)
    {
        printf("fcntl F_SETFL error");
        return -2;
    }
    return 0;
}
int deinitDevice()
{
    stopRead();
    printf("deinit ir device complete!\n");
}
void* readInputThread(void* arg)
{
    int tty_set_flag = tty_set();
    while(g_isRun)
    {
        if( kbhit() ) {

        int ch = getchar();
        printf("%c\n",ch);
        if(ch == 'q')
        {
            return NULL;
        }
        pthread_mutex_lock(&g_times_mutex);
        keyTimes[ch]++;
        pthread_mutex_unlock(&g_times_mutex);
        }
        usleep(33000);
    }
    if(tty_set_flag == 0)
        tty_reset();
    return NULL;
}
void refreshTimes()
{
    pthread_mutex_lock(&g_times_mutex);
    std::map<int,int>::iterator iter = keyTimes.begin();
    for(; iter != keyTimes.end(); iter++)
    {
        iter->second = 0;
    }
    pthread_mutex_unlock(&g_times_mutex);
}
int startRead()
{
    g_isRun = true;
    refreshTimes();
    int ret = pthread_create(&g_thread, NULL, readInputThread, NULL);
    if (ret != 0)
    {
        printf("Create pthread error!\n");
        return -1;
    }
    return 0;
}
int stopRead()
{
    g_isRun = false;
    pthread_join(g_thread, NULL);
    printf("stop read keyboard complete!\n");
}
void readFromDevice()
{
}
}

