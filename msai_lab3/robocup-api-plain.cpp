#define WIN32
#ifdef WIN32
#include <WinSock2.h>
#include <Windows.h>

inline long long gettimeus()
{
    static LARGE_INTEGER ClockPerSecond = { 0 };
    if( ClockPerSecond.QuadPart == 0 ) QueryPerformanceFrequency( &ClockPerSecond );
    LARGE_INTEGER li;
    QueryPerformanceCounter( &li );
    return li.QuadPart * 1000000LL / ClockPerSecond.QuadPart;
}

#pragma comment(lib, "ws2_32.lib")

#else // WIN32

#include <arpa/inet.h>
#include <unistd.h>
#include <sys/time.h>

inline long long gettimeus()
{
    struct timeval tv;

    gettimeofday( &tv, NULL );
    return (long long) tv.tv_sec * 1000000LL + (long long) tv.tv_usec;
}

#endif // WIN32

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <mutex>

#include "robocup.h"

using namespace std;

const int buf_len = 256;

int sock;
char buf[buf_len];

long long start_time = 0;

std::mutex api_mutex;

bool initNet()
{
    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(20037);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    return connect(sock, (sockaddr*)&addr, sizeof(addr)) == 0;
}

void initRobo()
{
    strcpy(buf, "I0=arduino\n");
    send(sock, buf, strlen(buf), 0);
    recv(sock, buf, buf_len, 0);
    //printf("I> %s\n", buf);

    strcpy(buf, "ASD=0,0\n");
    send(sock, buf, strlen(buf), 0);
    recv(sock, buf, buf_len, 0);
    //printf("ASD> %s\n", buf);

    strcpy(buf, "ASO0=0,1\n");
    send(sock, buf, strlen(buf), 0);
    recv(sock, buf, buf_len, 0);
    //printf("ASO0> %s\n", buf);

    strcpy(buf, "ASO1=0,2\n");
    send(sock, buf, strlen(buf), 0);
    recv(sock, buf, buf_len, 0);
    //printf("ASO1> %s\n", buf);

    strcpy(buf, "ASO2=0,3\n");
    send(sock, buf, strlen(buf), 0);
    recv(sock, buf, buf_len, 0);
    //printf("ASO2> %s\n", buf);

    strcpy(buf, "ASG0=0,4\n");
    send(sock, buf, strlen(buf), 0);
    recv(sock, buf, buf_len, 0);
    //printf("ASG0> %s\n", buf);

    strcpy(buf, "ASG1=0,5\n");
    send(sock, buf, strlen(buf), 0);
    recv(sock, buf, buf_len, 0);
    //printf("ASG1> %s\n", buf);

    strcpy(buf, "ASG2=0,6\n");
    send(sock, buf, strlen(buf), 0);
    recv(sock, buf, buf_len, 0);
    //printf("ASG2> %s\n", buf);

    strcpy(buf, "ASA=0,7\n");
    send(sock, buf, strlen(buf), 0);
    recv(sock, buf, buf_len, 0);
    //printf("ASA> %s\n", buf);
}

void setMotor(int i, double v)
{
	std::unique_lock<std::mutex> lk(api_mutex);
    sprintf(buf, "SRD0,%d=%lf\n", i, v);
    send(sock, buf, strlen(buf), 0);
    recv(sock, buf, buf_len, 0);
    //printf("SRD0,%d=%lf> %s\n", i, v, buf);
}

int main()
{
#ifdef WIN32
	WSADATA wsa;
	WSAStartup(MAKEWORD(1, 1), &wsa);
	STARTUPINFOA si = {};
	si.cb = sizeof(STARTUPINFOA);
	PROCESS_INFORMATION pi = {};
    CreateProcessA("robo-plain.exe", "", NULL, NULL, FALSE, NORMAL_PRIORITY_CLASS, NULL, NULL, &si, &pi);
#else
    system("./robo-plain >/dev/null &");
    //system("ffmpeg -f x11grab -r 30 -s 939x738 -i $DISPLAY+399,29 -vcodec libx264 -threads 0 output.mkv 2>/dev/null >/dev/null &");
#endif

    delay(2000);
    bool inited = false;//initNet();
    for (int i = 0; i < 5 && !(inited = initNet()); i++) {
        delay(1);
    }
    if (!inited) {
        fprintf(stderr, "simulation run failed\n");
        return 1;
    }
    initRobo();

    start_time = gettimeus();

	setup();
    while (true) {
        user_loop();
    }

    return 0;
}

void set_movement(float left, float right)
{
    if (left < -1)
        left = -1;
    if (left > 1)
        left = 1;
    if (right < -1)
        right = -1;
    if (right > 1)
        right = 1;
    setMotor(0, left);
    setMotor(1, right);
}

void set_servo_angle(float angle)
{
    if (angle < -90 || angle > 90)
        return;
    setMotor(2, angle / 100);
}


/*void set_display_text(const char *text)
{
    // do nothing here
}*/

void set_rgb_led(int red, int green, int blue)
{
    setMotor(3, red <= 55 ? -1 : (red - 155) / 100.0);
    setMotor(4, green <= 55 ? -1 : (green - 155) / 100.0);
    setMotor(5, blue <= 55 ? -1 : (blue - 155) / 100.0);
}

float get_ultrasonic()
{
	std::unique_lock<std::mutex> lk(api_mutex);
    sprintf(buf, "GSD\n");
    send(sock, buf, strlen(buf), 0);
    recv(sock, buf, buf_len, 0);
	return float(atof(buf) * 100);
}


int get_grayscale(int no)
{
	std::unique_lock<std::mutex> lk(api_mutex);
    sprintf(buf, "GSO%d\n", no);
    //printf("%s", buf);
    send(sock, buf, strlen(buf), 0);
    recv(sock, buf, buf_len, 0);
    //printf(">%s", buf);
    return 1-atoi(buf);
}


float get_accel_x()
{
	std::unique_lock<std::mutex> lk(api_mutex);
    sprintf(buf, "GSG0\n");
    send(sock, buf, strlen(buf), 0);
    recv(sock, buf, buf_len, 0);
    return float(atof(buf));
}


float get_accel_y()
{
	std::unique_lock<std::mutex> lk(api_mutex);
    sprintf(buf, "GSG1\n");
    send(sock, buf, strlen(buf), 0);
    recv(sock, buf, buf_len, 0);
    return float(atof(buf));
}


float get_accel_z()
{
	std::unique_lock<std::mutex> lk(api_mutex);
    sprintf(buf, "GSG2\n");
    send(sock, buf, strlen(buf), 0);
    recv(sock, buf, buf_len, 0);
    return float(atof(buf));
}


float get_audio_level()
{
	std::unique_lock<std::mutex> lk(api_mutex);
    sprintf(buf, "GSA\n");
    send(sock, buf, strlen(buf), 0);
    recv(sock, buf, buf_len, 0);
    return float(atof(buf));
}


float get_time()
{
    return float((gettimeus() - start_time) / 1000000.0f);
}


void delay(int msec)
{
#ifdef _WIN32
    Sleep(msec);
#else
    usleep(msec * 1000);
#endif
}

