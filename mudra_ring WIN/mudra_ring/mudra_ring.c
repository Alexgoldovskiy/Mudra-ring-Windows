#include "mudra_ring.h"

/* Global flags for controlling sensor readings */
volatile bool running          = true;
volatile bool read_sensors     = false;
volatile bool logging_csv      = false;
volatile bool streaming_active = false;

/* Mutex for safe access to sensor flags */
#ifdef _WIN32
pthread_mutex_t sensor_mutex;
#else
pthread_mutex_t sensor_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

/* File pointers for CSV logging */
FILE* bmi323_csv_file  = NULL;
FILE* afe4950_csv_file = NULL;

/* Global variables for pipe file */
#ifdef _WIN32
HANDLE bmi323_pipe_stream  = INVALID_HANDLE_VALUE;
HANDLE afe4950_pipe_stream = INVALID_HANDLE_VALUE;
#else
FILE* bmi323_pipe_stream  = NULL;
FILE* afe4950_pipe_stream = NULL;
#endif

/* Global CSV filenames */
char bmi323_csv_filename[MAX_FILENAME_LEN];
char afe4950_csv_filename[MAX_FILENAME_LEN];

/* Global sample counters for CSV that persist across sessions */
uint64_t bmi323_sample_counter          = 0;
uint64_t bmi323_log_timestamp_counter   = 0;
uint64_t bmi323_pipe_timestamp_counter  = 0;
uint64_t afe4950_sample_counter         = 0;
uint64_t afe4950_log_timestamp_counter  = 0;
uint64_t afe4950_pipe_timestamp_counter = 0;

/* Global variables for holding sensors sensitivity */
float acc_sense = 0.0;
float gyr_sense = 0.0;

/* Flag to indicate if CSV auto-stop thread is running */
volatile bool csv_timer_thread_running = false;
#ifdef _WIN32
HANDLE csv_timer_thread;
#else
pthread_t csv_timer_thread;
#endif

/* AFE4950 (PPG) serial ports */
#ifdef _WIN32
HANDLE reg_edit_port = INVALID_HANDLE_VALUE;
HANDLE capture_port  = INVALID_HANDLE_VALUE;
#else
int reg_edit_port = -1;
int capture_port  = -1;
#endif

/* Variable to store the start time for relative timestamps */
#ifdef _WIN32
win_timeval log_start_time;
#else
struct timeval log_start_time;
#endif

const char* registerSettingsFile = "AFE4950_Settings_mcu_hand.cfg";

#ifdef _WIN32
/* Windows implementation of gettimeofday */
int gettimeofday(struct win_timeval *tv, void* tz)
{
    FILETIME ft;
    unsigned __int64 tmpres = 0;
    if (tv)
    {
        GetSystemTimeAsFileTime(&ft);
        tmpres |= ft.dwHighDateTime;
        tmpres <<= 32;
        tmpres |= ft.dwLowDateTime;
        tmpres /= 10;  /* convert to microseconds */
        tmpres -= 11644473600000000ULL; /* convert to Unix epoch time */
        tv->tv_sec = (long)(tmpres / 1000000UL);
        tv->tv_usec = (long)(tmpres % 1000000UL);
    }
    return 0;
}
#endif

/*******************************************
 * BMI323 IMU Functions
 *******************************************/

/* Function to set up the named pipe for streaming */
void setup_pipe_streaming(void)
{
#ifdef _WIN32
    // BMI323 pipe setup
    if (bmi323_pipe_stream == INVALID_HANDLE_VALUE)
    {
        bmi323_pipe_stream = CreateNamedPipe(
            BMI323_PIPE_STREAM_NAME,
            PIPE_ACCESS_OUTBOUND,
            PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT,
            1,
            STREAM_BUFFER_SIZE,
            STREAM_BUFFER_SIZE,
            0,
            NULL);

        if (bmi323_pipe_stream == INVALID_HANDLE_VALUE)
        {
            printf("\r\033[KError creating named pipe for BMI323 streaming: %lu\n", GetLastError());
        }
        else
        {
            printf("\r\033[KBMI323 pipe streaming activated: %s\n", BMI323_PIPE_STREAM_NAME);
            bmi323_pipe_timestamp_counter = 0;
        }
    }

    // AFE4950 pipe setup
    if (afe4950_pipe_stream == INVALID_HANDLE_VALUE)
    {
        afe4950_pipe_stream = CreateNamedPipe(
            AFE4950_PIPE_STREAM_NAME,
            PIPE_ACCESS_OUTBOUND,
            PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT,
            1,
            STREAM_BUFFER_SIZE,
            STREAM_BUFFER_SIZE,
            0,
            NULL);

        if (afe4950_pipe_stream == INVALID_HANDLE_VALUE)
        {
            printf("\r\033[KError creating named pipe for AFE4950 streaming: %lu\n", GetLastError());
        }
        else
        {
            printf("\r\033[KAFE4950 pipe streaming activated: %s\n", AFE4950_PIPE_STREAM_NAME);
            afe4950_pipe_timestamp_counter = 0;
        }
    }
#else
    // BMI323 pipe setup
    if (bmi323_pipe_stream == NULL)
    {
        /* Create named pipe if it doesn't exist */
        if (access(BMI323_PIPE_STREAM_NAME, F_OK) == -1)
        {
            if (mkfifo(BMI323_PIPE_STREAM_NAME, 0666) != 0)
            {
                printf("\r\033[KError creating named pipe for BMI323 streaming\n");
            }
            else
            {
                /* Open pipe in non-blocking mode */
                bmi323_pipe_stream = fopen(BMI323_PIPE_STREAM_NAME, "w");
                if (!bmi323_pipe_stream)
                {
                    printf("\r\033[KError opening pipe for BMI323 streaming!\n");
                }
                else
                {
                    printf("\r\033[KBMI323 pipe streaming activated: %s\n", BMI323_PIPE_STREAM_NAME);
                    bmi323_pipe_timestamp_counter = 0;
                }
            }
        }
        else
        {
            /* Open pipe in non-blocking mode */
            bmi323_pipe_stream = fopen(BMI323_PIPE_STREAM_NAME, "w");
            if (!bmi323_pipe_stream)
            {
                printf("\r\033[KError opening pipe for BMI323 streaming!\n");
            }
            else
            {
                printf("\r\033[KBMI323 pipe streaming activated: %s\n", BMI323_PIPE_STREAM_NAME);
                bmi323_pipe_timestamp_counter = 0;
            }
        }
    }

    // AFE4950 pipe setup
    if (afe4950_pipe_stream == NULL)
    {
        if (access(AFE4950_PIPE_STREAM_NAME, F_OK) == -1)
        {
            if (mkfifo(AFE4950_PIPE_STREAM_NAME, 0666) != 0)
            {
                printf("\r\033[KError creating named pipe for AFE4950 streaming\n");
            }
            else
            {
                afe4950_pipe_stream = fopen(AFE4950_PIPE_STREAM_NAME, "w");
                if (!afe4950_pipe_stream)
                {
                    printf("\r\033[KError opening pipe for AFE4950 streaming!\n");
                }
                else
                {
                    printf("\r\033[KAFE4950 pipe streaming activated: %s\n", AFE4950_PIPE_STREAM_NAME);
                    afe4950_pipe_timestamp_counter = 0;
                }
            }
        }
        else
        {
            afe4950_pipe_stream = fopen(AFE4950_PIPE_STREAM_NAME, "w");
            if (!afe4950_pipe_stream)
            {
                printf("\r\033[KError opening pipe for AFE4950 streaming!\n");
            }
            else
            {
                printf("\r\033[KAFE4950 pipe streaming activated: %s\n", AFE4950_PIPE_STREAM_NAME);
                afe4950_pipe_timestamp_counter = 0;
            }
        }
    }
#endif
}

/* Function to close the pipe */
void close_pipe_streaming(void)
{
#ifdef _WIN32
    if (bmi323_pipe_stream != INVALID_HANDLE_VALUE)
    {
        FlushFileBuffers(bmi323_pipe_stream);
        DisconnectNamedPipe(bmi323_pipe_stream);
        CloseHandle(bmi323_pipe_stream);
        bmi323_pipe_stream = INVALID_HANDLE_VALUE;
        bmi323_pipe_timestamp_counter = 0;
        printf("\r\033[KBMI323 pipe streaming deactivated\n");
    }

    if (afe4950_pipe_stream != INVALID_HANDLE_VALUE)
    {
        FlushFileBuffers(afe4950_pipe_stream);
        DisconnectNamedPipe(afe4950_pipe_stream);
        CloseHandle(afe4950_pipe_stream);
        afe4950_pipe_stream = INVALID_HANDLE_VALUE;
        afe4950_pipe_timestamp_counter = 0;
        printf("\r\033[KAFE4950 pipe streaming deactivated\n");
    }
#else
    if (bmi323_pipe_stream)
    {
        fclose(bmi323_pipe_stream);
        bmi323_pipe_stream            = NULL;
        bmi323_pipe_timestamp_counter = 0;
        printf("\r\033[KBMI323 pipe streaming deactivated\n");
    }

    if (afe4950_pipe_stream)
    {
        fclose(afe4950_pipe_stream);
        afe4950_pipe_stream            = NULL;
        afe4950_pipe_timestamp_counter = 0;
        printf("\r\033[KAFE4950 pipe streaming deactivated\n");
    }
#endif
}

/* Function to stream BMI323 data to pipe */
void stream_to_pipe(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyr_x, int16_t gyr_y, int16_t gyr_z)
{
    pthread_mutex_lock(&sensor_mutex);
#ifdef _WIN32
    HANDLE local_pipe = bmi323_pipe_stream;
#else
    FILE* local_pipe = bmi323_pipe_stream;
#endif
    pthread_mutex_unlock(&sensor_mutex);

#ifdef _WIN32
    if (local_pipe != INVALID_HANDLE_VALUE)
    {
        char buffer[256];
        int length = snprintf(buffer, sizeof(buffer), "%llu,%f,%f,%f,%f,%f,%f\n",
                            (unsigned long long) bmi323_pipe_timestamp_counter * 1250,
                            (float) acc_x * acc_sense, (float) acc_y * acc_sense,
                            (float) acc_z * acc_sense, (float) gyr_x * gyr_sense,
                            (float) gyr_y * gyr_sense, (float) gyr_z * gyr_sense);
        bmi323_pipe_timestamp_counter++;
        
        DWORD bytesWritten;
        if (WriteFile(local_pipe, buffer, length, &bytesWritten, NULL))
        {
            FlushFileBuffers(local_pipe);
        }
    }
#else
    if (local_pipe)
    {
        fprintf(local_pipe, "%llu,%f,%f,%f,%f,%f,%f\n", 
                (unsigned long long) bmi323_pipe_timestamp_counter * 1250,
                (float) acc_x * acc_sense, (float) acc_y * acc_sense,
                (float) acc_z * acc_sense, (float) gyr_x * gyr_sense,
                (float) gyr_y * gyr_sense, (float) gyr_z * gyr_sense);
        bmi323_pipe_timestamp_counter++;
        fflush(local_pipe);
    }
#endif
}

/* Function to stream AFE4950 data to pipe */
void afe4950_stream_to_pipe(float* tia_voltage)
{
    pthread_mutex_lock(&sensor_mutex);
#ifdef _WIN32
    HANDLE local_pipe = afe4950_pipe_stream;
#else
    FILE* local_pipe = afe4950_pipe_stream;
#endif
    pthread_mutex_unlock(&sensor_mutex);

#ifdef _WIN32
    if (local_pipe != INVALID_HANDLE_VALUE)
    {
        char buffer[256];
        int length = snprintf(buffer, sizeof(buffer), "%f,%f,%f,%f,%f\n",
                            (unsigned long long) afe4950_pipe_timestamp_counter * 390.625,
                            tia_voltage[0], tia_voltage[1], tia_voltage[2], tia_voltage[3]);
        afe4950_pipe_timestamp_counter++;
        
        DWORD bytesWritten;
        if (WriteFile(local_pipe, buffer, length, &bytesWritten, NULL))
        {
            FlushFileBuffers(local_pipe);
        }
    }
#else
    if (local_pipe)
    {
        fprintf(local_pipe, "%f,%f,%f,%f,%f\n",
                (unsigned long long) afe4950_pipe_timestamp_counter * 390.625,
                tia_voltage[0], tia_voltage[1], tia_voltage[2], tia_voltage[3]);
        afe4950_pipe_timestamp_counter++;
        fflush(local_pipe);
    }
#endif
}

/* Function to read a register */
int16_t bmi323_read_reg(uint8_t reg_addr, uint8_t* data, uint16_t len)
{
    return coines_read_spi(COINES_SPI_BUS_0, BMI323_CS_PIN, reg_addr | BMI323_SPI_READ_BIT, data, len);
}

/* Function to write to a register */
int16_t bmi323_write_reg(uint8_t reg_addr, const uint8_t* data, uint16_t len)
{
    return coines_write_spi(COINES_SPI_BUS_0, BMI323_CS_PIN, reg_addr, (uint8_t*) data, len);
}

/* Function to execute a command */
int16_t bmi323_execute_command(uint16_t command)
{
    int16_t result;
    uint8_t cmd_bytes[2];

    cmd_bytes[0] = (uint8_t) ((command >> 8) & 0xFF);
    cmd_bytes[1] = (uint8_t) (command & 0xFF);

    result = bmi323_write_reg(BMI323_REG_CMD, cmd_bytes, 2);

    return result;
}

/* Function to set accelerometer configuration */
int16_t bmi323_set_accel_config(const struct bmi323_accel_config* config)
{
    int16_t result;
    uint8_t reg_data[2] = {0};

    reg_data[0] = (uint8_t) (config->odr | (config->range << 4) | (config->bwp << 7));
    reg_data[1] = (uint8_t) (config->avg_num | (config->acc_mode << 4));

    result = bmi323_write_reg(BMI323_REG_ACC_CONF, reg_data, 2);

    return result;
}

/* Function to set gyroscope configuration */
int16_t bmi323_set_gyro_config(const struct bmi323_gyro_config* config)
{
    int16_t result;
    uint8_t reg_data[2] = {0};

    reg_data[0] = (uint8_t) (config->odr | (config->range << 4) | (config->bwp << 7));
    reg_data[1] = (uint8_t) (config->avg_num | (config->gyr_mode << 4));

    result = bmi323_write_reg(BMI323_REG_GYR_CONF, reg_data, 2);

    return result;
}

/* Function to get accelerometer sensitivity */
void bmi323_get_accel_sensitivity(uint8_t range)
{
    switch (range)
    {
        case BMI323_ACC_RANGE_2G:
            acc_sense = 2.0 / 32768.0;
            break;
        case BMI323_ACC_RANGE_4G:
            acc_sense = 4.0 / 32768.0;
            break;
        case BMI323_ACC_RANGE_8G:
            acc_sense = 8.0 / 32768.0;
            break;
        case BMI323_ACC_RANGE_16G:
            acc_sense = 16.0 / 32768.0;
            break;
        default:
            acc_sense = 2.0 / 32768.0;
            break;
    }
}

/* Function to get gyroscope sensitivity */
void bmi323_get_gyro_sensitivity(uint8_t range)
{
    switch (range)
    {
        case BMI323_GYR_RANGE_125DPS:
            gyr_sense = 125.0 / 32768.0;
            break;
        case BMI323_GYR_RANGE_250DPS:
            gyr_sense = 250.0 / 32768.0;
            break;
        case BMI323_GYR_RANGE_500DPS:
            gyr_sense = 500.0 / 32768.0;
            break;
        case BMI323_GYR_RANGE_1000DPS:
            gyr_sense = 1000.0 / 32768.0;
            break;
        case BMI323_GYR_RANGE_2000DPS:
            gyr_sense = 2000.0 / 32768.0;
            break;
        default:
            gyr_sense = 250.0 / 32768.0;
            break;
    }
}

/* Function to generate a filename with timestamp */
void generate_timestamped_filename(void)
{
    time_t     now;
    struct tm* timeinfo;

    time(&now);
    timeinfo = localtime(&now);

    // Generate BMI323 filename
    strftime(bmi323_csv_filename, MAX_FILENAME_LEN, "data/bmi323_data_%Y-%m-%d_%H-%M-%S.csv", timeinfo);

    // Generate AFE4950 filename
    strftime(afe4950_csv_filename, MAX_FILENAME_LEN, "data/ppg_data_%Y-%m-%d_%H-%M-%S.csv", timeinfo);
}

/* Thread function for auto-stopping CSV logging after 5 seconds */
#ifdef _WIN32
unsigned __stdcall win_csv_timer_thread(void* arg)
#else
void* csv_timer_thread_func(void* arg)
#endif
{
    csv_timer_thread_running = true;

    sleep(5);

    pthread_mutex_lock(&sensor_mutex);

    if (logging_csv)
    {
        printf("\r\033[KAuto-stopping CSV logging after 5 seconds\n");
        stop_csv_logging();
    }

    pthread_mutex_unlock(&sensor_mutex);

    csv_timer_thread_running = false;
#ifdef _WIN32
    return 0;
#else
    return NULL;
#endif
}

/* Function to setup the thread for auto-stopping CSV */
void setup_csv_timer(void)
{
    if (csv_timer_thread_running)
    {
        return;
    }

#ifdef _WIN32
    csv_timer_thread = (HANDLE)_beginthreadex(NULL, 0, win_csv_timer_thread, NULL, 0, NULL);
    if (csv_timer_thread == 0)
    {
        printf("\r\033[KError creating thread for CSV auto-stop\n");
        return;
    }
#else
    if (pthread_create(&csv_timer_thread, NULL, csv_timer_thread_func, NULL) != 0)
    {
        printf("\r\033[KError creating thread for CSV auto-stop\n");
        return;
    }
    pthread_detach(csv_timer_thread);
#endif
}

/* Function to start CSV logging with 10-second auto-stop */
void start_csv_logging(void)
{
    bool already_logging;

    // Check current state with mutex locked
    pthread_mutex_lock(&sensor_mutex);
    already_logging = logging_csv;
    pthread_mutex_unlock(&sensor_mutex);

    if (already_logging)
    {
        return;
    }

    // Generate filenames
    generate_timestamped_filename();

    // Open files outside of mutex lock
    FILE* bmi_file = NULL;
    FILE* afe_file = NULL;

    // Try to open BMI323 CSV file
    bmi_file = fopen(bmi323_csv_filename, "w");
    if (!bmi_file)
    {
        printf("\r\033[KError opening BMI323 CSV file for writing!\n");
    }
    else
    {
        fprintf(bmi_file, "Timestamp,Sample,ACC_X,ACC_Y,ACC_Z,GYRO_X,GYRO_Y,GYRO_Z\n");
        printf("\r\033[KLogging BMI323 to: %s (will automatically stop after 5 seconds)\n", bmi323_csv_filename);
    }

    // Try to open AFE4950 CSV file
    afe_file = fopen(afe4950_csv_filename, "w");
    if (!afe_file)
    {
        printf("\r\033[KError opening AFE4950 CSV file for writing!\n");

        // If BMI323 file was opened but AFE4950 failed, close BMI323 file
        if (bmi_file)
        {
            fclose(bmi_file);
            bmi_file = NULL;
        }
    }
    else
    {
        // Write header to AFE4950 file
        fprintf(afe_file, "Timestamp,Sample,REG_6D,TIA1-3_RAW,TIA1-3_VALUE,TIA1-5_RAW,TIA1-5_VALUE,TIA1-8_RAW,TIA1-8_VALUE,TIA1-10_RAW,TIA1-10_VALUE\n");
        printf("\r\033[KLogging AFE4950 to: %s (will automatically stop after 5 seconds)\n", afe4950_csv_filename);
    }

    // If either file opened successfully, proceed with logging
    if (bmi_file || afe_file)
    {
#ifdef _WIN32
        win_timeval start_time;
#else
        struct timeval start_time;
#endif
        gettimeofday(&start_time, NULL);

        // Update global state with mutex locked
        pthread_mutex_lock(&sensor_mutex);
        bmi323_csv_file               = bmi_file;
        afe4950_csv_file              = afe_file;
        log_start_time                = start_time;
        logging_csv                   = true;
        bmi323_sample_counter         = 0;
        bmi323_log_timestamp_counter  = 0;
        afe4950_sample_counter        = 0;
        afe4950_log_timestamp_counter = 0;
        pthread_mutex_unlock(&sensor_mutex);

        // Start the auto-stop timer
        setup_csv_timer();
    }
}

/* Function to stop CSV logging */
void stop_csv_logging(void)
{
    if (logging_csv)
    {
        if (bmi323_csv_file)
        {
            fflush(bmi323_csv_file);
            fclose(bmi323_csv_file);
            bmi323_csv_file = NULL;
        }

        if (afe4950_csv_file)
        {
            fflush(afe4950_csv_file);
            fclose(afe4950_csv_file);
            afe4950_csv_file = NULL;
        }

        logging_csv                   = false;
        bmi323_sample_counter         = 0;
        bmi323_log_timestamp_counter  = 0;
        afe4950_sample_counter        = 0;
        afe4950_log_timestamp_counter = 0;

        printf("\r\033[KCSV logging stopped\n");
    }
}

/* Function to log BMI323 data to CSV with microsecond precision */
void log_to_csv(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyr_x, int16_t gyr_y, int16_t gyr_z)
{
    if (logging_csv && bmi323_csv_file)
    {
        // Format: timestamp, sample, ACC-X, ACC-Y, ACC-Z, GYR-X, GYR-Y, GYR-Z
#ifdef _WIN32
        // Use %I64u for MinGW instead of %llu
        fprintf(bmi323_csv_file, "%I64u,%I64u,%f,%f,%f,%f,%f,%f\n", 
#else
        fprintf(bmi323_csv_file, "%llu,%llu,%f,%f,%f,%f,%f,%f\n", 
#endif
                (unsigned long long) bmi323_log_timestamp_counter * 1250, 
                (unsigned long long) bmi323_sample_counter++,
                ((float) acc_x * acc_sense), ((float) acc_y * acc_sense), 
                ((float) acc_z * acc_sense), ((float) gyr_x * gyr_sense), 
                ((float) gyr_y * gyr_sense), ((float) gyr_z * gyr_sense));
        bmi323_log_timestamp_counter++;

        fflush(bmi323_csv_file);
    }
}

/* Function to log AFE4950 data to CSV */
void afe4950_log_to_csv(uint32_t reg_6d, uint32_t* tia_raw, float* tia_voltage)
{
    if (logging_csv && afe4950_csv_file)
    {
        // Format: timestamp, sample, REG_6D, TIA1-3_RAW, TIA1-3_VALUE, TIA1-5_RAW, TIA1-5_VALUE, TIA1-8_RAW, TIA1-8_VALUE, TIA1-10_RAW, TIA1-10_VALUE
#ifdef _WIN32
        // Use %I64u for MinGW instead of %llu
        fprintf(afe4950_csv_file, "%f,%I64u,0x%06X,%u,%f,%u,%f,%u,%f,%u,%f\n", 
#else
        fprintf(afe4950_csv_file, "%f,%llu,0x%06X,%u,%f,%u,%f,%u,%f,%u,%f\n", 
#endif
                (float)((unsigned long long) afe4950_log_timestamp_counter * 390.625), 
                (unsigned long long) afe4950_sample_counter++,
                reg_6d, 
                tia_raw[0], tia_voltage[0], 
                tia_raw[1], tia_voltage[1], 
                tia_raw[2], tia_voltage[2], 
                tia_raw[3], tia_voltage[3]);
        afe4950_log_timestamp_counter++;

        fflush(afe4950_csv_file);
    }
}

/* Function to display critical information */
void display_info(const char* message)
{
    printf("\r\033[K%s\n", message);
    fflush(stdout);
}

/* Signal handler for clean termination */
void handle_signal(int signal)
{
    running = false;
}

/*********************************************
 * BMI323 IMU Specific Functions
 *********************************************/

/* Function to initialize the BMI323 sensor */
int16_t bmi323_init(void)
{
    struct bmi323_accel_config acc_config = {0};
    struct bmi323_gyro_config  gyr_config = {0};
    int16_t result;
    uint8_t chip_id[2] = {0};

    // Try to configure shuttle board voltage
    result = coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
    if (result != COINES_SUCCESS)
    {
        display_info("Error: Failed to set board voltage");
        return result;
    }

    coines_delay_msec(100);

    // Try to configure SPI
    result = coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_6_MHZ, COINES_SPI_MODE0);
    if (result != COINES_SUCCESS)
    {
        display_info("Error: Failed to configure SPI");
        return result;
    }
    coines_delay_msec(100);

    // Try to configure CS pin
    result = coines_set_pin_config(BMI323_CS_PIN, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
    if (result != COINES_SUCCESS)
    {
        display_info("Error: Failed to configure chip select");
        return result;
    }
    coines_delay_msec(100);

    result = bmi323_execute_command(BMI323_CMD_SOFT_RESET);
    if (result != COINES_SUCCESS)
    {
        display_info("Error: Failed to reset sensor");
        return result;
    }

    coines_delay_msec(100);

    result = bmi323_read_reg(BMI323_REG_CHIP_ID, chip_id, 2);
    if (result != COINES_SUCCESS)
    {
        display_info("Error: Failed to read chip ID");
        return result;
    }

    uint16_t chip_id_be = (chip_id[0] << 8) | chip_id[1];

    if (chip_id_be != BMI323_CHIP_ID)
    {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "Error: Unexpected chip ID. Expected 0x%04X, got 0x%04X", BMI323_CHIP_ID, chip_id_be);
        display_info(buffer);
        return -1;
    }

    coines_delay_msec(100);

    acc_config.odr      = BMI323_ACC_ODR_800HZ;
    acc_config.range    = BMI323_ACC_RANGE_2G;
    acc_config.bwp      = BMI323_ACC_BW_ODR_HALF;
    acc_config.acc_mode = BMI323_ACC_MODE_HIGH_PERF;
    acc_config.avg_num  = BMI323_ACC_AVG2;

    result = bmi323_set_accel_config(&acc_config);
    if (result != COINES_SUCCESS)
    {
        display_info("Error: Failed to configure accelerometer");
        return result;
    }

    bmi323_get_accel_sensitivity(acc_config.range);

    coines_delay_msec(100);

    gyr_config.odr      = BMI323_GYR_ODR_800HZ;
    gyr_config.range    = BMI323_GYR_RANGE_250DPS;
    gyr_config.bwp      = BMI323_GYR_BW_ODR_HALF;
    gyr_config.gyr_mode = BMI323_GYR_MODE_HIGH_PERF;
    gyr_config.avg_num  = BMI323_GYR_AVG2;

    result = bmi323_set_gyro_config(&gyr_config);
    if (result != COINES_SUCCESS)
    {
        display_info("Error: Failed to configure gyroscope");
        return result;
    }

    bmi323_get_gyro_sensitivity(gyr_config.range);

    coines_delay_msec(100);

    return COINES_SUCCESS;
}

/* Function to stop all streaming */
int16_t stop_all_streaming(void)
{
    int16_t result;

    if (streaming_active)
    {
        result = coines_start_stop_streaming(COINES_STREAMING_MODE_POLLING, COINES_STREAMING_STOP);
        if (result != COINES_SUCCESS)
        {
            display_info("Error: Failed to stop streaming");
            return result;
        }

        streaming_active = false;
        coines_delay_msec(50);
    }

    return COINES_SUCCESS;
}

/* Function to configure and start streaming for accelerometer and/or gyroscope */
int16_t start_streaming(void)
{
    int16_t                        result;
    struct coines_streaming_config stream_config = {0};
    struct coines_streaming_blocks data_blocks   = {0};

    stop_all_streaming();

    stream_config.intf              = COINES_SENSOR_INTF_SPI;
    stream_config.spi_bus           = COINES_SPI_BUS_0;
    stream_config.cs_pin            = BMI323_CS_PIN;
    stream_config.sampling_time     = 1250;
    stream_config.sampling_units    = COINES_SAMPLING_TIME_IN_MICRO_SEC;
    data_blocks.no_of_blocks        = 1;
    data_blocks.reg_start_addr[0]   = BMI323_REG_ACC_DATA_X | BMI323_SPI_READ_BIT;
    data_blocks.no_of_data_bytes[0] = 13;

    result = coines_config_streaming(BMI323_STREAM_ACCEL_CHANNEL, &stream_config, &data_blocks);
    if (result != COINES_SUCCESS)
    {
        display_info("Error: Failed to configure accelerometer streaming");
        return result;
    }

    result = coines_start_stop_streaming(COINES_STREAMING_MODE_POLLING, COINES_STREAMING_START);
    if (result != COINES_SUCCESS)
    {
        display_info("Error: Failed to start streaming");
        return result;
    }

    streaming_active = true;
    return COINES_SUCCESS;
}

/* Function to read BMI323 sensor data */
void read_sensor_data(void)
{
    int16_t result;
    bool    local_read_sensors, local_logging_csv;

    pthread_mutex_lock(&sensor_mutex);
    local_read_sensors = read_sensors;
    local_logging_csv  = logging_csv;
    pthread_mutex_unlock(&sensor_mutex);

    if (!local_read_sensors)
    {
        return;
    }

    if (streaming_active)
    {
        uint8_t  buffer[MAX_STREAM_SAMPLES] = {0};
        uint32_t sensor_samples_count       = 0;

        if (local_read_sensors)
        {
            result = coines_read_stream_sensor_data(BMI323_STREAM_ACCEL_CHANNEL, 1, buffer, &sensor_samples_count);

            if (result != COINES_SUCCESS || sensor_samples_count == 0)
            {
                return;
            }
        }

        printf("\r\033[K");

        if (local_read_sensors && sensor_samples_count > 0)
        {
            int16_t acc_x = (int16_t) ((buffer[2] << 8) | buffer[1]);
            int16_t acc_y = (int16_t) ((buffer[4] << 8) | buffer[3]);
            int16_t acc_z = (int16_t) ((buffer[6] << 8) | buffer[5]);

            int16_t gyr_x = (int16_t) ((buffer[8] << 8) | buffer[7]);
            int16_t gyr_y = (int16_t) ((buffer[10] << 8) | buffer[9]);
            int16_t gyr_z = (int16_t) ((buffer[12] << 8) | buffer[11]);

#ifdef _WIN32
            if (bmi323_pipe_stream != INVALID_HANDLE_VALUE)
#else
            if (bmi323_pipe_stream)
#endif
            {
                stream_to_pipe(acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z);
            }

            if (local_logging_csv)
            {
                log_to_csv(acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z);
            }
        }
    }

    fflush(stdout);
}

/* Function to automatically find BMI323 serial port */
int16_t bmi323_find_serial_port(void)
{
#ifdef _WIN32
    char device_path[256];
    int16_t result = COINES_E_FAILURE;
    struct coines_serial_com_config scom_config = {0};
    
    // First try to directly open using COINES_COMM_INTF_USB
    result = coines_open_comm_intf(COINES_COMM_INTF_USB, NULL);
    if (result == COINES_SUCCESS)
    {
        printf("Connected to BMI323 IMU via USB\n");
        return result;
    }
    
    // If that fails, try COM ports from COM1 to COM20
    for (int i = 1; i <= 20; i++)
    {
        snprintf(device_path, sizeof(device_path), "COM%d", i);
        printf("Trying %s...\n", device_path);
        
        // Configure connection
        scom_config.baud_rate      = COINES_UART_BAUD_RATE_115200;
        scom_config.vendor_id      = ROBERT_BOSCH_USB_VID;
        scom_config.product_id     = BST_APP31_CDC_USB_PID;
        scom_config.com_port_name  = device_path;
        scom_config.rx_buffer_size = COINES_STREAM_RSP_BUF_SIZE;

        // Try to open connection
        result = coines_open_comm_intf(COINES_COMM_INTF_USB, &scom_config);
        if (result == COINES_SUCCESS)
        {
            printf("Connected to BMI323 IMU on %s\n", device_path);
            return result;
        }
    }
    
    printf("\r\033[KCould not detect BMI323 IMU on any serial port\n");
    return COINES_E_FAILURE;
#endif
}

/******************************************
 * AFE4950 PPG Specific Functions
 ******************************************/

int afe4950_bytes_available(int port)
{
#ifdef _WIN32
    COMSTAT status;
    DWORD errors;
    
    if (ClearCommError((HANDLE)port, &errors, &status))
    {
        return status.cbInQue;
    }
    return 0;
#else
    int bytes;
    if (ioctl(port, FIONREAD, &bytes) < 0)
    {
        return 0;
    }
    return bytes;
#endif
}

int afe4950_open_serial_port(const char* port_name, int* port_handle)
{
#ifdef _WIN32
    HANDLE hComm;
    DCB dcb;
    COMMTIMEOUTS timeouts;
    
    char full_port_name[256];
    // Prepend \\.\\ to support COM ports > COM9
    snprintf(full_port_name, sizeof(full_port_name), "\\\\.\\%s", port_name);
    
    hComm = CreateFile(full_port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    
    if (hComm == INVALID_HANDLE_VALUE)
    {
        printf("\r\033[KError opening serial port %s: %lu\n", port_name, GetLastError());
        return -1;
    }
    
    // Configure port settings
    if (!GetCommState(hComm, &dcb))
    {
        printf("\r\033[KError getting serial port state\n");
        CloseHandle(hComm);
        return -1;
    }
    
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    
    if (!SetCommState(hComm, &dcb))
    {
        printf("\r\033[KError setting serial port state\n");
        CloseHandle(hComm);
        return -1;
    }
    
    // Configure timeouts
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.ReadTotalTimeoutConstant = 100;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 100;
    
    if (!SetCommTimeouts(hComm, &timeouts))
    {
        printf("\r\033[KError setting serial port timeouts\n");
        CloseHandle(hComm);
        return -1;
    }
    
    *port_handle = (int)hComm;
    return 0;
#else
    int serial_port = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_port < 0)
    {
        printf("\r\033[KError opening serial port %s\n", port_name);
        return -1;
    }

    struct termios cu;
    memset(&cu, 0, sizeof(cu));

    if (tcgetattr(serial_port, &cu) != 0)
    {
        printf("\r\033[KError getting serial port attributes\n");
        close(serial_port);
        return -1;
    }

    cfsetospeed(&cu, B115200);
    cfsetispeed(&cu, B115200);

    cu.c_cflag &= ~PARENB;
    cu.c_cflag &= ~CSTOPB;
    cu.c_cflag &= ~CSIZE;
    cu.c_cflag |= CS8;
    cu.c_cflag &= ~CRTSCTS;
    cu.c_cflag |= CREAD | CLOCAL;
    cu.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    cu.c_oflag &= ~OPOST;
    cu.c_cc[VMIN]  = 0;
    cu.c_cc[VTIME] = 10;

    if (tcsetattr(serial_port, TCSANOW, &cu) != 0)
    {
        printf("\r\033[KError setting serial port attributes\n");
        close(serial_port);
        return -1;
    }

    *port_handle = serial_port;

    return 0;
#endif
}

void afe4950_close_serial_port(int port_handle)
{
#ifdef _WIN32
    if ((HANDLE)port_handle != INVALID_HANDLE_VALUE)
    {
        CloseHandle((HANDLE)port_handle);
    }
#else
    if (port_handle >= 0)
    {
        close(port_handle);
    }
#endif
}

void afe4950_close_all_ports(void)
{
    // Don't call reset if we don't have valid ports
#ifdef _WIN32
    if (reg_edit_port != INVALID_HANDLE_VALUE)
    {
        afe4950_reset_afe();
    }
#else
    if (reg_edit_port >= 0)
    {
        afe4950_reset_afe();
    }
#endif

#ifdef _WIN32
    if (reg_edit_port != INVALID_HANDLE_VALUE)
    {
        afe4950_close_serial_port((int)reg_edit_port);
        reg_edit_port = INVALID_HANDLE_VALUE;
    }
    if (capture_port != INVALID_HANDLE_VALUE)
    {
        afe4950_close_serial_port((int)capture_port);
        capture_port = INVALID_HANDLE_VALUE;
    }
#else
    if (reg_edit_port >= 0)
    {
        afe4950_close_serial_port(reg_edit_port);
        reg_edit_port = -1;
    }
    if (capture_port >= 0)
    {
        afe4950_close_serial_port(capture_port);
        capture_port = -1;
    }
#endif
}

void detect_connected_devices(void)
{
    printf("Detecting connected devices...\n");
    
#ifdef _WIN32
    HANDLE hComm;
    char device_path[256];
    
    for (int i = 1; i <= 20; i++)
    {
        snprintf(device_path, sizeof(device_path), "COM%d", i);
        
        hComm = CreateFile(device_path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
        if (hComm != INVALID_HANDLE_VALUE)
        {
            printf("Found device on %s\n", device_path);
            CloseHandle(hComm);
        }
    }
#endif
    
    printf("Device detection complete.\n");
}

void afe4950_flush_serial_port(int port)
{
#ifdef _WIN32
    PurgeComm((HANDLE)port, PURGE_RXCLEAR);
#else
    tcflush(port, TCIFLUSH);
#endif
}

int afe4950_send_command_to_port(int port, const char* command)
{
    char cmd_with_newline[256];
    snprintf(cmd_with_newline, sizeof(cmd_with_newline), "%s\n", command);

#ifdef _WIN32
    PurgeComm((HANDLE)port, PURGE_RXCLEAR | PURGE_TXCLEAR);
    DWORD bytesWritten;
    
    if (!WriteFile((HANDLE)port, cmd_with_newline, (DWORD)strlen(cmd_with_newline), &bytesWritten, NULL))
    {
        printf("\r\033[KError writing to serial port: %lu\n", GetLastError());
        return -1;
    }
#else
    tcflush(port, TCIOFLUSH);
    int bytes_written = write(port, cmd_with_newline, strlen(cmd_with_newline));
    if (bytes_written < 0)
    {
        printf("\r\033[KError writing to serial port\n");
        return -1;
    }
#endif

    usleep(5000);
    return strlen(cmd_with_newline);
}

int afe4950_read_response_from_port(int port, char* buffer, int buffer_size, int timeout_ms)
{
    int total_bytes_read = 0;
    int elapsed_ms       = 0;
    memset(buffer, 0, buffer_size);

    while (elapsed_ms < timeout_ms && total_bytes_read < buffer_size - 1)
    {
        int available = afe4950_bytes_available(port);

        if (available > 0)
        {
            int bytes_to_read = (available < buffer_size - 1 - total_bytes_read) ? available : (buffer_size - 1 - total_bytes_read);
            
#ifdef _WIN32
            DWORD bytes_read;
            if (ReadFile((HANDLE)port, buffer + total_bytes_read, bytes_to_read, &bytes_read, NULL))
            {
                if (bytes_read > 0)
                {
                    total_bytes_read += bytes_read;
                    buffer[total_bytes_read] = '\0';
                }
            }
            else
            {
                printf("\r\033[KError reading from serial port: %lu\n", GetLastError());
                return total_bytes_read;
            }
#else
            int bytes_read = read(port, buffer + total_bytes_read, bytes_to_read);
            if (bytes_read < 0)
            {
                printf("\r\033[KError reading from serial port\n");
                return total_bytes_read;
            }

            if (bytes_read > 0)
            {
                total_bytes_read += bytes_read;
                buffer[total_bytes_read] = '\0';
            }
#endif

            if (strchr(buffer, '\n') != NULL)
            {
                break;
            }
        }

        usleep(10000);
        elapsed_ms += 10;
    }

    return total_bytes_read;
}

int afe4950_query_device_identity(int port, char* identity, int buffer_size)
{
    afe4950_flush_serial_port(port);

    if (afe4950_send_command_to_port(port, CMD_CDC_QUERY) < 0)
    {
        return -1;
    }

    int bytes_read = afe4950_read_response_from_port(port, identity, buffer_size, 500);

    if (bytes_read <= 0)
    {
        return -1;
    }

    identity[strcspn(identity, "\r\n")] = 0;
    return 0;
}

int afe4950_reset_afe(void)
{
#ifdef _WIN32
    if (reg_edit_port == INVALID_HANDLE_VALUE)
    {
        return -1;
    }
    
    if (afe4950_send_command_to_port((int)reg_edit_port, CMD_RESET_AFE) < 0)
    {
        return -1;
    }
#else
    if (reg_edit_port < 0)
    {
        return -1;
    }
    
    if (afe4950_send_command_to_port(reg_edit_port, CMD_RESET_AFE) < 0)
    {
        return -1;
    }
#endif

    usleep(500000);

    char response[256];
#ifdef _WIN32
    int bytes_read = afe4950_read_response_from_port((int)reg_edit_port, response, sizeof(response), 500);
#else
    int bytes_read = afe4950_read_response_from_port(reg_edit_port, response, sizeof(response), 500);
#endif

    if (bytes_read > 0 && strstr(response, "SUCCESS") != NULL)
    {
        return 0;
    }

    printf("\r\033[KReset AFE failed: %s\n", response);
    return -1;
}


int afe4950_write_register(uint8_t reg_addr, uint32_t value)
{
    char command[64];
    sprintf(command, "%s%02X%06X%c", CMD_REG_WRITE, reg_addr, value & 0xFFFFFF, '1');

#ifdef _WIN32
    if (afe4950_send_command_to_port((int)reg_edit_port, command) < 0)
    {
        return -1;
    }
#else
    if (afe4950_send_command_to_port(reg_edit_port, command) < 0)
    {
        return -1;
    }
#endif

    char response[256];
#ifdef _WIN32
    int bytes_read = afe4950_read_response_from_port((int)reg_edit_port, response, sizeof(response), 500);
#else
    int bytes_read = afe4950_read_response_from_port(reg_edit_port, response, sizeof(response), 500);
#endif

    if (bytes_read > 0)
    {
        if (strstr(response, "SUCCESS") != NULL)
        {
            return 0;
        }
    }

    printf("\r\033[KWrite register 0x%02X failed\n", reg_addr);
    return -1;
}

int afe4950_read_register(uint8_t reg_addr, uint32_t* value)
{
    char command[32];
    sprintf(command, "%s%02X%c", CMD_REG_READ, reg_addr, '1');

#ifdef _WIN32
    if (afe4950_send_command_to_port((int)reg_edit_port, command) < 0)
    {
        return -1;
    }
#else
    if (afe4950_send_command_to_port(reg_edit_port, command) < 0)
    {
        return -1;
    }
#endif

    char response[256];
#ifdef _WIN32
    int bytes_read = afe4950_read_response_from_port((int)reg_edit_port, response, sizeof(response), 500);
#else
    int bytes_read = afe4950_read_response_from_port(reg_edit_port, response, sizeof(response), 500);
#endif

    if (bytes_read > 0)
    {
        char* success = strstr(response, "SUCCESS");
        if (success != NULL)
        {
            char* hex_value = strchr(success, '\n');
            if (hex_value != NULL)
            {
                hex_value++;
                uint32_t parsed_value;
                if (sscanf(hex_value, "%X", &parsed_value) == 1)
                {
                    *value = parsed_value;
                    return 0;
                }
            }
        }

        printf("\r\033[KFailed to parse register value from: %s\n", response);
    }

    return -1;
}

int afe4950_stop_capture(void)
{
#ifdef _WIN32
    return afe4950_send_command_to_port((int)capture_port, CMD_STOP_CAPTURE);
#else
    return afe4950_send_command_to_port(capture_port, CMD_STOP_CAPTURE);
#endif
}

int afe4950_init(void)
{
    if (afe4950_reset_afe() < 0)
    {
        return -1;
    }

    if (afe4950_select_comm_interface("SPI") < 0)
    {
        return -1;
    }

    FILE* file = fopen(registerSettingsFile, "r");
    if (!file)
    {
        printf("\r\033[KError: Could not open config file '%s'\n", registerSettingsFile);
        return -1;
    }

    char line[256];
    int  success_count = 0;

    while (fgets(line, sizeof(line), file) != NULL)
    {
        // Skip empty lines
        if (line[0] == '\n' || strlen(line) <= 1)
        {
            continue;
        }

        // Parse the {0xXX,0xXXXXXX} format
        uint16_t addr;
        uint32_t value;

        if (sscanf(line, "{0x%hX,0x%X}", &addr, &value) == 2)
        {
            if (afe4950_write_register(addr, value) == 0)
            {
                success_count++;
            }
        }
        else
        {
            printf("\r\033[KWarning: Failed to parse line: %s\n", line);
        }
    }

    fclose(file);
    return success_count > 0 ? 0 : -1;
}

int afe4950_start_capture(int numSamples, int finite)
{
    char command[64];
    sprintf(command, "%s%02X%08X", CMD_START_CAPTURE, finite ? 1 : 0, numSamples);

#ifdef _WIN32
    if (afe4950_send_command_to_port((int)capture_port, command) < 0)
    {
        return -1;
    }
#else
    if (afe4950_send_command_to_port(capture_port, command) < 0)
    {
        return -1;
    }
#endif

    usleep(10000);

    return 0;
}

int afe4950_select_comm_interface(const char* interface_name)
{
    if (afe4950_reset_afe() < 0)
    {
        return -1;
    }

    char command[32];
    sprintf(command, "%s %c", CMD_COMM_SELECT, interface_name[0]);

#ifdef _WIN32
    if (afe4950_send_command_to_port((int)reg_edit_port, command) < 0)
    {
        return -1;
    }
#else
    if (afe4950_send_command_to_port(reg_edit_port, command) < 0)
    {
        return -1;
    }
#endif

    char response[256];
#ifdef _WIN32
    int bytes_read = afe4950_read_response_from_port((int)reg_edit_port, response, sizeof(response), 500);
#else
    int bytes_read = afe4950_read_response_from_port(reg_edit_port, response, sizeof(response), 500);
#endif

    if (bytes_read > 0 && strstr(response, "SUCCESS") != NULL)
    {
        return 0;
    }

    printf("\r\033[KFailed to set communication interface: %s\n", response);
    return -1;
}
int afe4950_open_and_identify_ports(void)
{
    char device_path[256];
    char response[256];
    
#ifdef _WIN32
    // First, check if ports are already open and valid
    if (reg_edit_port != INVALID_HANDLE_VALUE && capture_port != INVALID_HANDLE_VALUE)
    {
        return 0;
    }
    
    // Try COM ports from COM1 to COM20
    for (int i = 1; i <= 20; i++)
    {
        snprintf(device_path, sizeof(device_path), "COM%d", i);
        printf("Checking %s for AFE4950...\n", device_path);
        
        int port_handle = -1;
        if (afe4950_open_serial_port(device_path, &port_handle) == 0)
        {
            afe4950_flush_serial_port(port_handle);
            
            if (afe4950_send_command_to_port(port_handle, CMD_CDC_QUERY) > 0)
            {
                memset(response, 0, sizeof(response));
                int bytes_read = afe4950_read_response_from_port(port_handle, response, sizeof(response), 500);
                
                if (bytes_read > 0)
                {
                    if (strstr(response, "REG_EDIT") != NULL)
                    {
                        reg_edit_port = (HANDLE)port_handle;
                        printf("Found REG_EDIT port on %s\n", device_path);
                    }
                    else if (strstr(response, "CAPTURE") != NULL)
                    {
                        capture_port = (HANDLE)port_handle;
                        printf("Found CAPTURE port on %s\n", device_path);
                    }
                    else
                    {
                        afe4950_close_serial_port(port_handle);
                    }
                }
                else
                {
                    afe4950_close_serial_port(port_handle);
                }
            }
            else
            {
                afe4950_close_serial_port(port_handle);
            }
            
            if (reg_edit_port != INVALID_HANDLE_VALUE && capture_port != INVALID_HANDLE_VALUE)
            {
                break;
            }
        }
    }
#else
    DIR*           dp;
    struct dirent* entry;

    dp = opendir("/dev");
    if (dp == NULL)
    {
        printf("\r\033[KError: Cannot open /dev directory: %s\n", strerror(errno));
        return -1;
    }

    while ((entry = readdir(dp)))
    {
        if (strncmp(entry->d_name, "cu.usbmodem", 11) == 0)
        {
            snprintf(device_path, sizeof(device_path), "/dev/%s", entry->d_name);

            int port_handle = -1;
            if (afe4950_open_serial_port(device_path, &port_handle) == 0)
            {
                afe4950_flush_serial_port(port_handle);

                if (afe4950_send_command_to_port(port_handle, CMD_CDC_QUERY) > 0)
                {
                    memset(response, 0, sizeof(response));
                    int bytes_read = afe4950_read_response_from_port(port_handle, response, sizeof(response), 500);

                    if (bytes_read > 0)
                    {
                        if (strstr(response, "REG_EDIT") != NULL)
                        {
                            reg_edit_port = port_handle;
                        }
                        else if (strstr(response, "CAPTURE") != NULL)
                        {
                            capture_port = port_handle;
                        }
                        else
                        {
                            afe4950_close_serial_port(port_handle);
                        }
                    }
                    else
                    {
                        afe4950_close_serial_port(port_handle);
                    }
                }
                else
                {
                    afe4950_close_serial_port(port_handle);
                }
            }

            if (reg_edit_port != -1 && capture_port != -1)
            {
                break;
            }
        }
    }

    closedir(dp);
#endif

#ifdef _WIN32
    if (reg_edit_port == INVALID_HANDLE_VALUE || capture_port == INVALID_HANDLE_VALUE)
    {
        printf("\r\033[KError: Failed to find both required AFE4950 ports\n");
        return -1;
    }
#else
    if (reg_edit_port == -1 || capture_port == -1)
    {
        printf("\r\033[KError: Failed to find both required AFE4950 ports\n");
        return -1;
    }
#endif

    return 0;
}

/* Process capture data continuously */
int afe4950_process_capture_data_continuous(void)
{
    char buffer[CAPTURE_BUFFER_SIZE];
    int  bytes_read;
    int  samples_processed = 0;
    int  available;

    // Check for available bytes
#ifdef _WIN32
    available = afe4950_bytes_available((int)capture_port);
#else
    available = afe4950_bytes_available(capture_port);
#endif
    if (available <= 0)
    {
        return 0;
    }

    // Read only what's available
#ifdef _WIN32
    bytes_read = afe4950_read_response_from_port((int)capture_port, buffer, available < CAPTURE_BUFFER_SIZE - 1 ? available : CAPTURE_BUFFER_SIZE - 1, 100);
#else
    bytes_read = afe4950_read_response_from_port(capture_port, buffer, available < CAPTURE_BUFFER_SIZE - 1 ? available : CAPTURE_BUFFER_SIZE - 1, 100);
#endif
    if (bytes_read <= 0)
    {
        return 0;
    }

    buffer[bytes_read] = '\0';

    // Split buffer by newlines
    char* line_context = NULL;
    char* line         = strtok_r(buffer, "\n", &line_context);

    while (line != NULL)
    {
        // Skip empty lines
        if (strlen(line) == 0)
        {
            line = strtok_r(NULL, "\n", &line_context);
            continue;
        }

        // Split line by tabs
        char* words[128]; // Increased to handle more data points (up to 32 samples * 4 values)
        int   word_count   = 0;
        char* word_context = NULL;

        char* word = strtok_r(line, "\t", &word_context);
        while (word != NULL && word_count < 128)
        {
            // Skip empty words
            if (strlen(word) > 0)
            {
                words[word_count++] = word;
            }
            word = strtok_r(NULL, "\t", &word_context);
        }

        if (word_count == 0)
        {
            line = strtok_r(NULL, "\n", &line_context);
            continue;
        }

        // Skip EXTRA_REG lines
        if (strcmp(words[0], "EXTRA_REG") == 0)
        {
            line = strtok_r(NULL, "\n", &line_context);
            continue;
        }

        // Parse REG_6D value
        uint32_t reg_6d_entire = 0;
        uint8_t  reg_6d_val    = 0;

        if (sscanf(words[0], "%X", &reg_6d_entire) == 1)
        {
            reg_6d_val = (uint8_t) (reg_6d_entire + 1); // Match Python behavior
        }
        else
        {
            printf("\r\033[KError parsing REG_6D value: %s\n", words[0]);
            line = strtok_r(NULL, "\n", &line_context);
            continue;
        }

        // Process all TIA values in the line
        // For each group of 4 values, we have one complete sample
        int total_values     = word_count - 1; // Subtract 1 for REG_6D value
        int complete_samples = total_values / 4;

        // Ensure we don't exceed the number of samples specified by REG_6D_VAL
        if (complete_samples > reg_6d_val)
        {
            complete_samples = reg_6d_val;
        }

        for (int sample = 0; sample < complete_samples; sample++)
        {
            uint32_t tia_values[4]   = {0};
            float    tia_voltages[4] = {0.0f};

            // Process the 4 values for this sample
            for (int i = 0; i < 4; i++)
            {
                int word_idx = sample * 4 + i + 1; // +1 to skip REG_6D value

                if (word_idx >= word_count)
                {
                    break; // Safety check
                }

                // Parse hex value
                uint32_t unsignedCodesADC = 0;
                if (sscanf(words[word_idx], "%X", &unsignedCodesADC) != 1)
                {
                    printf("\r\033[KError parsing ADC value: %s\n", words[word_idx]);
                    continue;
                }

                // Keep only 24 bits
                unsignedCodesADC &= 0xFFFFFF;
                tia_values[i] = unsignedCodesADC;

                // Convert to signed value and calculate voltage
                int32_t signedCodesADC;
                if (unsignedCodesADC >= (1 << 23))
                {
                    signedCodesADC = unsignedCodesADC - (1 << 24);
                }
                else
                {
                    signedCodesADC = unsignedCodesADC;
                }

                // Calculate voltage (1.2/2^21)
                float voltage   = (float) signedCodesADC * VOLTAGE_RES;
                tia_voltages[i] = voltage;
            }

            if (logging_csv)
            {
                if (reg_6d_val == 32)
                {
                    afe4950_log_to_csv(reg_6d_entire, tia_values, tia_voltages);
                }
            }

#ifdef _WIN32
            if (afe4950_pipe_stream != INVALID_HANDLE_VALUE)
#else
            if (afe4950_pipe_stream)
#endif
            {
                if (reg_6d_val == 32)
                {
                    afe4950_stream_to_pipe(tia_voltages);
                }
            }

            samples_processed++;
        }

        line = strtok_r(NULL, "\n", &line_context);
    }

    // Flush stdout to ensure immediate display
    fflush(stdout);

    return samples_processed;
}

/***********************************************
 * Shared Functions
 ***********************************************/

/* Function to toggle sensor reading on and off */
void toggle_sensors(void)
{
    bool    current_state;
    int16_t bmi_result = COINES_SUCCESS;
    int     afe_result = 0;

    // First check the current state with the mutex locked
    pthread_mutex_lock(&sensor_mutex);
    current_state = read_sensors;
    pthread_mutex_unlock(&sensor_mutex);

    if (!current_state)
    {
        // Start BMI323 streaming (do this outside mutex lock)
        bmi_result = start_streaming();

        // Start AFE4950 capture (do this outside mutex lock)
        if (bmi_result == COINES_SUCCESS)
        {
            afe_result = afe4950_start_capture(0, 0);
        }

        // Now update the state based on results
        pthread_mutex_lock(&sensor_mutex);
        if (bmi_result == COINES_SUCCESS && afe_result >= 0)
        {
            read_sensors = true;
            printf("\r\033[KSensors activated - Reading IMU and PPG data\n");
        }
        else
        {
            // If either sensor failed, make sure both are stopped
            if (bmi_result == COINES_SUCCESS)
            {
                stop_all_streaming();
            }
            printf("\r\033[KError starting sensors\n");
        }
        pthread_mutex_unlock(&sensor_mutex);
    }
    else
    {
        // Stop both sensors (outside mutex lock)
        stop_all_streaming();
        afe4950_stop_capture();

        // Update state
        pthread_mutex_lock(&sensor_mutex);
        read_sensors = false;
        printf("\r\033[KSensors deactivated\n");
        pthread_mutex_unlock(&sensor_mutex);
    }
}

/* Function to toggle pipe streaming */
void toggle_stream_pipe(void)
{
    bool has_bmi_stream, has_afe_stream;

    // Check current state with mutex locked
    pthread_mutex_lock(&sensor_mutex);
#ifdef _WIN32
    has_bmi_stream = (bmi323_pipe_stream != INVALID_HANDLE_VALUE);
    has_afe_stream = (afe4950_pipe_stream != INVALID_HANDLE_VALUE);
#else
    has_bmi_stream = (bmi323_pipe_stream != NULL);
    has_afe_stream = (afe4950_pipe_stream != NULL);
#endif
    pthread_mutex_unlock(&sensor_mutex);

    if (!has_bmi_stream || !has_afe_stream)
    {
        setup_pipe_streaming();
    }
    else
    {
        close_pipe_streaming();
    }
}

/* Function to process a single-character command */
void process_command(char command)
{
    // Print first without locking
    printf("\r\033[K");

    switch (command)
    {
        case 's':
            toggle_sensors();
            break;
        case 'c':
            // Check status with mutex locked
            pthread_mutex_lock(&sensor_mutex);
            bool sensors_active = read_sensors;
            bool csv_active     = logging_csv;
            pthread_mutex_unlock(&sensor_mutex);

            if (!csv_active)
            {
                if (!sensors_active)
                {
                    printf("Sensors must be active to log data. Press 's' to start sensors first.\n");
                }
                else
                {
                    printf("Starting CSV logging for 5 seconds...\n");
                    // Call CSV start function outside of mutex lock
                    start_csv_logging();
                }
            }
            else
            {
                printf("CSV logging is already active\n");
            }
            break;
        case 'p':
            toggle_stream_pipe();
            break;
        case 'q':
            printf("Exiting program...\n");
            pthread_mutex_lock(&sensor_mutex);
            running = false;
            pthread_mutex_unlock(&sensor_mutex);
            break;
        default:
            printf("Unknown command: '%c'\n", command);
            break;
    }

    fflush(stdout);
}

/* Thread function for reading user input */
#ifdef _WIN32
unsigned __stdcall win_command_thread(void* arg)
#else
void* command_thread(void* arg)
#endif
{
    int ch;

    printf("\r\033[KAvailable commands:\n");
    printf("  s - Toggle sensors (both IMU and PPG)\n");
    printf("  c - Start logging to CSV file for 5 seconds\n");
    printf("  p - Toggle pipe stream for both sensors\n");
    printf("  q - Quit the program\n");

#ifdef _WIN32
    while (running)
    {
        if (_kbhit())
        {
            ch = _getch();
            process_command((char) ch);
        }
        Sleep(10);
    }
#else
    system("stty raw -echo");

    while (running)
    {
        ch = getchar();
        if (ch != EOF)
        {
            process_command((char) ch);
        }
        coines_delay_msec(10);
        usleep(20000);
    }

    system("stty sane");
#endif

#ifdef _WIN32
    return 0;
#else
    return NULL;
#endif
}

/* Thread function for BMI323 sensor data acquisition */
#ifdef _WIN32
unsigned __stdcall win_bmi323_thread(void* arg)
#else
void* bmi323_thread(void* arg)
#endif
{
    while (running)
    {
        pthread_mutex_lock(&sensor_mutex);
        bool sensors_active = read_sensors;
        pthread_mutex_unlock(&sensor_mutex);

        if (sensors_active)
        {
            read_sensor_data();
            coines_delay_usec(950);
        }
        else
        {
            coines_delay_msec(100);
        }
    }

#ifdef _WIN32
    return 0;
#else
    return NULL;
#endif
}

/* Thread function for AFE4950 sensor data acquisition */
#ifdef _WIN32
unsigned __stdcall win_afe4950_thread(void* arg)
#else
void* afe4950_thread(void* arg)
#endif
{
    bool      local_read_sensors;
    int       processed;
    time_t    last_data_read_time = time(NULL);
    bool      timed_out           = false;
    const int timeout             = 10; // 10 seconds timeout

    // Calculate time between frames for 50Hz (20ms per frame)
    const long     target_frame_time_us = 20000; // 20ms = 50Hz
#ifndef _WIN32
    struct timeval last_display_time;
    struct timeval current_time;
    long           elapsed_us;

    gettimeofday(&last_display_time, NULL);
#endif

    while (running)
    {
        pthread_mutex_lock(&sensor_mutex);
        local_read_sensors = read_sensors;
        pthread_mutex_unlock(&sensor_mutex);

        if (local_read_sensors)
        {
            processed = afe4950_process_capture_data_continuous();

            if (processed > 0)
            {
                // Reset timeout if we got data
                last_data_read_time = time(NULL);
                timed_out           = false;

#ifndef _WIN32
                // Check if it's time to update the display (target 50Hz)
                gettimeofday(&current_time, NULL);
                elapsed_us = (current_time.tv_sec - last_display_time.tv_sec) * 1000000 + (current_time.tv_usec - last_display_time.tv_usec);

                // If we've reached or exceeded our target frame time, reset the timer
                if (elapsed_us >= target_frame_time_us)
                {
                    last_display_time = current_time;
                }
#endif

                // Keep polling rapidly for data to avoid missing samples
                usleep(1000);
            }
            else
            {
                // Check for timeout
                if ((time(NULL) - last_data_read_time) > timeout)
                {
                    if (!timed_out)
                    {
                        printf("\r\033[KAFE4950 Capture Timed Out\n");
                        timed_out = true;
                        afe4950_stop_capture();
                    }
                }
                // If no data, wait a bit longer but not too long to miss data
                usleep(10000); // 10ms
            }
        }
        else
        {
            usleep(100000); // 100ms when not reading sensors
        }
    }

#ifdef _WIN32
    return 0;
#else
    return NULL;
#endif
}

/* Main function */
int main(void)
{
    int16_t                    result;
    pthread_t                  cmd_thread, bmi_thread, ppg_thread;

    signal(SIGINT, handle_signal);

    printf("\nMUDRA Ring Data Acquisition Program\n");
    printf("----------------------------------------------------------\n");

#ifdef _WIN32
    // Initialize critical section on Windows
    InitializeCriticalSection(&sensor_mutex);
    
    // Detect connected devices first
    detect_connected_devices();
#endif

    // Initialize BMI323 IMU
    result = bmi323_find_serial_port();
    if (result != COINES_SUCCESS)
    {
        printf("Error: Failed to connect to COINES board for BMI323\n");
        goto cleanup;
    }

    coines_delay_msec(100);

    result = bmi323_init();
    if (result != COINES_SUCCESS)
    {
        printf("Error: Failed to initialize BMI323 sensor\n");
        goto cleanup;
    }

    // Initialize AFE4950 PPG
    if (afe4950_open_and_identify_ports() < 0)
    {
        printf("Failed to find and open the required serial ports for AFE4950\n");
        goto cleanup;
    }

    if (afe4950_init() < 0)
    {
        printf("Error: Failed to initialize AFE4950 sensor\n");
        goto cleanup;
    }

    // Create threads
#ifdef _WIN32
    if ((cmd_thread = (HANDLE)_beginthreadex(NULL, 0, win_command_thread, NULL, 0, NULL)) == 0)
#else
    if (pthread_create(&cmd_thread, NULL, command_thread, NULL) != 0)
#endif
    {
        printf("Error: Failed to create command thread\n");
        goto cleanup;
    }

#ifdef _WIN32
    if ((bmi_thread = (HANDLE)_beginthreadex(NULL, 0, win_bmi323_thread, NULL, 0, NULL)) == 0)
#else
    if (pthread_create(&bmi_thread, NULL, bmi323_thread, NULL) != 0)
#endif
    {
        printf("Error: Failed to create BMI323 sensor thread\n");
        running = false;
#ifdef _WIN32
        WaitForSingleObject(cmd_thread, INFINITE);
        CloseHandle(cmd_thread);
#else
        pthread_join(cmd_thread, NULL);
#endif
        goto cleanup;
    }

#ifdef _WIN32
    if ((ppg_thread = (HANDLE)_beginthreadex(NULL, 0, win_afe4950_thread, NULL, 0, NULL)) == 0)
#else
    if (pthread_create(&ppg_thread, NULL, afe4950_thread, NULL) != 0)
#endif
    {
        printf("Error: Failed to create AFE4950 sensor thread\n");
        running = false;
#ifdef _WIN32
        WaitForSingleObject(cmd_thread, INFINITE);
        CloseHandle(cmd_thread);
        WaitForSingleObject(bmi_thread, INFINITE);
        CloseHandle(bmi_thread);
#else
        pthread_join(cmd_thread, NULL);
        pthread_join(bmi_thread, NULL);
#endif
        goto cleanup;
    }

    // Wait for threads to complete
#ifdef _WIN32
    WaitForSingleObject(cmd_thread, INFINITE);
    CloseHandle(cmd_thread);
    WaitForSingleObject(bmi_thread, INFINITE);
    CloseHandle(bmi_thread);
    WaitForSingleObject(ppg_thread, INFINITE);
    CloseHandle(ppg_thread);
#else
    pthread_join(cmd_thread, NULL);
    pthread_join(bmi_thread, NULL);
    pthread_join(ppg_thread, NULL);
#endif

    // Cleanup both sensors
cleanup:
    printf("\nCleaning up...\n");

    // Clean up AFE4950
    afe4950_stop_capture();
    afe4950_close_all_ports();
    stop_all_streaming();
    stop_csv_logging();
    close_pipe_streaming();
#ifndef _WIN32
    system("stty sane");
#endif

    if (csv_timer_thread_running)
    {
        coines_delay_msec(100);
    }

    pthread_mutex_destroy(&sensor_mutex);
    coines_set_shuttleboard_vdd_vddio_config(0, 0);
    coines_delay_msec(100);
    coines_soft_reset();
    coines_delay_msec(100);
    coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);

    printf("Program completed\n");
    return 0;
}