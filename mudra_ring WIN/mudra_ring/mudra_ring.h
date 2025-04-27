#ifndef MUDRA_RING_H
#define MUDRA_RING_H

#include "../COINES_SDK/v2.9.1/coines-api/coines.h"

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
    #include <windows.h>
    #include <process.h>
    #include <conio.h>
    #include <direct.h>
    #include <io.h>
    #define usleep(x) Sleep((x)/1000)
    #define sleep(x) Sleep((x)*1000)
    #define PATH_SEPARATOR '\\'
    #define isatty _isatty
    #define access _access
    #define F_OK 0
    
    // Windows mutex and thread definitions
    typedef CRITICAL_SECTION pthread_mutex_t;
    typedef HANDLE pthread_t;
    #define pthread_mutex_init(mutex, attr) InitializeCriticalSection(mutex)
    #define pthread_mutex_destroy(mutex) DeleteCriticalSection(mutex)
    #define pthread_mutex_lock(mutex) EnterCriticalSection(mutex)
    #define pthread_mutex_unlock(mutex) LeaveCriticalSection(mutex)
    #define PTHREAD_MUTEX_INITIALIZER {0}
    
    // Compatibility functions for threading
    #define pthread_create(thread, attr, func, arg) \
        ((*(thread) = (HANDLE)_beginthreadex(NULL, 0, (unsigned int (__stdcall *)(void *))func, arg, 0, NULL)) == 0 ? -1 : 0)
    #define pthread_join(thread, retval) \
        ((WaitForSingleObject(thread, INFINITE), CloseHandle(thread), 0))
    #define pthread_detach(thread) CloseHandle(thread)
#else
    #include <dirent.h>
    #include <pthread.h>
    #include <sys/ioctl.h>
    #include <sys/select.h>
    #include <sys/stat.h>
    #include <sys/time.h>
    #include <termios.h>
    #include <unistd.h>
    #define PATH_SEPARATOR '/'
#endif

/******************************************************************************/
/*!        BMI323 Shuttle Board Definitions               */
/******************************************************************************/
/* Board connection parameters */
#define ROBERT_BOSCH_USB_VID  (0x108C)
#define BST_APP31_CDC_USB_PID (0xAB38)

/* BMI323 Chip ID */
#define BMI323_CHIP_ID UINT16_C(0x0043)

/* BMI323 SPI register addresses */
#define BMI323_REG_CHIP_ID    UINT8_C(0x00)
#define BMI323_REG_ERR_REG    UINT8_C(0x01)
#define BMI323_REG_STATUS     UINT8_C(0x02)
#define BMI323_REG_ACC_DATA_X UINT8_C(0x03)
#define BMI323_REG_ACC_DATA_Y UINT8_C(0x04)
#define BMI323_REG_ACC_DATA_Z UINT8_C(0x05)
#define BMI323_REG_GYR_DATA_X UINT8_C(0x06)
#define BMI323_REG_GYR_DATA_Y UINT8_C(0x07)
#define BMI323_REG_GYR_DATA_Z UINT8_C(0x08)
#define BMI323_REG_TEMP_DATA  UINT8_C(0x09)
#define BMI323_REG_ACC_CONF   UINT8_C(0x20)
#define BMI323_REG_GYR_CONF   UINT8_C(0x21)
#define BMI323_REG_CMD        UINT8_C(0x7E)

/*!  Accelerometer Bandwidth parameters */
#define BMI323_ACC_AVG1  UINT8_C(0x00)
#define BMI323_ACC_AVG2  UINT8_C(0x01)
#define BMI323_ACC_AVG4  UINT8_C(0x02)
#define BMI323_ACC_AVG8  UINT8_C(0x03)
#define BMI323_ACC_AVG16 UINT8_C(0x04)
#define BMI323_ACC_AVG32 UINT8_C(0x05)
#define BMI323_ACC_AVG64 UINT8_C(0x06)

/*! Accelerometer Output Data Rate */
#define BMI323_ACC_ODR_50HZ  UINT8_C(0x07)
#define BMI323_ACC_ODR_100HZ UINT8_C(0x08)
#define BMI323_ACC_ODR_200HZ UINT8_C(0x09)
#define BMI323_ACC_ODR_400HZ UINT8_C(0x0A)
#define BMI323_ACC_ODR_800HZ UINT8_C(0x0B)

/*! Accelerometer G Range */
#define BMI323_ACC_RANGE_2G  UINT8_C(0x00)
#define BMI323_ACC_RANGE_4G  UINT8_C(0x01)
#define BMI323_ACC_RANGE_8G  UINT8_C(0x02)
#define BMI323_ACC_RANGE_16G UINT8_C(0x03)

/*! Accelerometer mode */
#define BMI323_ACC_MODE_DISABLE   UINT8_C(0x00)
#define BMI323_ACC_MODE_LOW_PWR   UINT8_C(0x03)
#define BMI323_ACC_MODE_NORMAL    UINT8_C(0X04)
#define BMI323_ACC_MODE_HIGH_PERF UINT8_C(0x07)

/*! Accelerometer bandwidth */
#define BMI323_ACC_BW_ODR_HALF    UINT8_C(0)
#define BMI323_ACC_BW_ODR_QUARTER UINT8_C(1)

/*!  Gyroscope Bandwidth parameters */
#define BMI323_GYR_AVG1  UINT8_C(0x00)
#define BMI323_GYR_AVG2  UINT8_C(0x01)
#define BMI323_GYR_AVG4  UINT8_C(0x02)
#define BMI323_GYR_AVG8  UINT8_C(0x03)
#define BMI323_GYR_AVG16 UINT8_C(0x04)
#define BMI323_GYR_AVG32 UINT8_C(0x05)
#define BMI323_GYR_AVG64 UINT8_C(0x06)

/*! Gyroscope Output Data Rate */
#define BMI323_GYR_ODR_50HZ  UINT8_C(0x07)
#define BMI323_GYR_ODR_100HZ UINT8_C(0x08)
#define BMI323_GYR_ODR_200HZ UINT8_C(0x09)
#define BMI323_GYR_ODR_400HZ UINT8_C(0x0A)
#define BMI323_GYR_ODR_800HZ UINT8_C(0x0B)

/*! Gyroscope DPS Range */
#define BMI323_GYR_RANGE_125DPS  UINT8_C(0x00)
#define BMI323_GYR_RANGE_250DPS  UINT8_C(0x01)
#define BMI323_GYR_RANGE_500DPS  UINT8_C(0x02)
#define BMI323_GYR_RANGE_1000DPS UINT8_C(0x03)
#define BMI323_GYR_RANGE_2000DPS UINT8_C(0x04)

/*! Gyroscope mode */
#define BMI323_GYR_MODE_DISABLE   UINT8_C(0x00)
#define BMI323_GYR_MODE_SUSPEND   UINT8_C(0X01)
#define BMI323_GYR_MODE_LOW_PWR   UINT8_C(0x03)
#define BMI323_GYR_MODE_NORMAL    UINT8_C(0X04)
#define BMI323_GYR_MODE_HIGH_PERF UINT8_C(0x07)

/*! Gyroscope bandwidth */
#define BMI323_GYR_BW_ODR_HALF    UINT8_C(0)
#define BMI323_GYR_BW_ODR_QUARTER UINT8_C(1)

/* BMI323 command values */
#define BMI323_CMD_SOFT_RESET UINT16_C(0xDEAF)

/* BMI323 SPI settings */
#define BMI323_SPI_READ_BIT  UINT16_C(0x80)
#define BMI323_SPI_WRITE_BIT UINT16_C(0x7F)
#define BMI323_CS_PIN        COINES_SHUTTLE_PIN_7

/* Streaming channel IDs */
#define BMI323_STREAM_ACCEL_CHANNEL 1
#define BMI323_STREAM_GYRO_CHANNEL  2

/******************************************************************************/
/*!        AFE4950EVM Definitions               */
/******************************************************************************/
#define CMD_RESET_AFE     "C:RST_AFE"
#define CMD_RESET_UC      "C:RST_UC"
#define CMD_REG_WRITE     "C:REG_W"
#define CMD_REG_READ      "C:REG_R"
#define CMD_START_CAPTURE "C:CAP_START"
#define CMD_STOP_CAPTURE  "C:CAP_STOP"
#define CMD_CDC_QUERY     "C:CDC?"
#define CMD_COMM_SELECT   "C:COMM_SEL"
#define CMD_EXTRA_REG     "C:XTRA_REG"

/******************************************************************************/
/*!        General Definitions               */
/******************************************************************************/
/* Max filename length */
#define MAX_FILENAME_LEN 128

/* Max sample buffer size for streaming */
#define MAX_STREAM_SAMPLES  50
#define STREAM_BUFFER_SIZE  1024
#define CAPTURE_BUFFER_SIZE 1024

// Name of the pipe for streaming
#ifdef _WIN32
    #define BMI323_PIPE_STREAM_NAME  "\\\\.\\pipe\\bmi323_stream"
    #define AFE4950_PIPE_STREAM_NAME "\\\\.\\pipe\\afe4950_stream"
#else
    #define BMI323_PIPE_STREAM_NAME  "/tmp/bmi323_stream"
    #define AFE4950_PIPE_STREAM_NAME "/tmp/afe4950_stream"
#endif

// Voltage resolution definition
#define VOLTAGE_RES (1.2f / (float) (1 << 21))

/* Structure to hold accelerometer configuration */
struct bmi323_accel_config {
    uint8_t odr;
    uint8_t bwp;
    uint8_t acc_mode;
    uint8_t range;
    uint8_t avg_num;
};

/* Structure to hold gyroscope configuration */
struct bmi323_gyro_config {
    uint8_t odr;
    uint8_t bwp;
    uint8_t gyr_mode;
    uint8_t range;
    uint8_t avg_num;
};

/* Variable to store the start time for relative timestamps */
#ifdef _WIN32
    struct win_timeval {
        long tv_sec;
        long tv_usec;
    };
    typedef struct win_timeval win_timeval;
#else
    struct timeval log_start_time;
#endif

/* Function declarations */
void    setup_pipe_streaming(void);
void    close_pipe_streaming(void);
void    stream_to_pipe(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyr_x, int16_t gyr_y, int16_t gyr_z);
int16_t bmi323_read_reg(uint8_t reg_addr, uint8_t* data, uint16_t len);
int16_t bmi323_write_reg(uint8_t reg_addr, const uint8_t* data, uint16_t len);
int16_t bmi323_execute_command(uint16_t command);
int16_t bmi323_set_accel_config(const struct bmi323_accel_config* config);
int16_t bmi323_set_gyro_config(const struct bmi323_gyro_config* config);
void    bmi323_get_accel_sensitivity(uint8_t range);
void    bmi323_get_gyro_sensitivity(uint8_t range);
void    generate_timestamped_filename(void);
void    setup_csv_timer(void);
void    start_csv_logging(void);
void    stop_csv_logging(void);
void    log_to_csv(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyr_x, int16_t gyr_y, int16_t gyr_z);
void    display_info(const char* message);
void    handle_signal(int signal);
int16_t bmi323_init(void);
int16_t stop_all_streaming(void);
int16_t start_streaming(void);
void    read_sensor_data(void);
void    toggle_sensors(void);
void    toggle_stream_pipe(void);
void    process_command(char command);
int16_t bmi323_find_serial_port(void);
int     afe4950_bytes_available(int port);
int     afe4950_open_serial_port(const char* port_name, int* port_handle);
void    afe4950_close_serial_port(int port_handle);
void    afe4950_close_all_ports(void);
void    afe4950_flush_serial_port(int port);
int     afe4950_send_command_to_port(int port, const char* command);
int     afe4950_read_response_from_port(int port, char* buffer, int buffer_size, int timeout_ms);
int     afe4950_query_device_identity(int port, char* identity, int buffer_size);
int     afe4950_reset_afe(void);
int     afe4950_write_register(uint8_t reg_addr, uint32_t value);
int     afe4950_read_register(uint8_t reg_addr, uint32_t* value);
int     afe4950_stop_capture(void);
int     afe4950_init(void);
int     afe4950_start_capture(int numSamples, int finite);
int     afe4950_select_comm_interface(const char* interface_name);  // Changed parameter name
int     afe4950_open_and_identify_ports(void);
void    afe4950_setup_pipe_streaming(void);
void    afe4950_close_pipe_streaming(void);
void    afe4950_stream_to_pipe(float* tia_voltage);
void    afe4950_generate_timestamped_filename(void);
void    afe4950_setup_csv_timer(void);
void    afe4950_start_csv_logging(void);
void    afe4950_stop_csv_logging(void);
void    afe4950_log_to_csv(uint32_t reg_6d, uint32_t* tia_raw, float* tia_voltage);
int     afe4950_process_capture_data_continuous(void);
void    afe4950_toggle_sensors(void);
void    afe4950_toggle_stream_pipe(void);

/* Thread function prototypes */
#ifdef _WIN32
unsigned __stdcall win_command_thread(void* arg);
unsigned __stdcall win_bmi323_thread(void* arg);
unsigned __stdcall win_afe4950_thread(void* arg);
unsigned __stdcall win_csv_timer_thread(void* arg);
int gettimeofday(struct win_timeval *tv, void* tz);
#else
void* command_thread(void* arg);
void* bmi323_thread(void* arg);
void* afe4950_thread(void* arg);
void* csv_timer_thread_func(void* arg);
#endif

#endif /* MUDRA_RING_H */