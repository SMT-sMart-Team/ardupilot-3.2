
#ifndef __AP_HAL_BOARDS_H__
#define __AP_HAL_BOARDS_H__

/**
 * C preprocesor enumeration of the boards supported by the AP_HAL.
 * This list exists so HAL_BOARD == HAL_BOARD_xxx preprocessor blocks
 * can be used to exclude HAL boards from the build when appropriate.
 * Its not an elegant solution but we cant get too fancy if we want to
 * work with the Arduino mk and IDE builds without too much modification.
 */

#define HAL_BOARD_APM1     1
#define HAL_BOARD_APM2     2
#define HAL_BOARD_AVR_SITL 3
#define HAL_BOARD_SMACCM   4 // unused
#define HAL_BOARD_PX4      5
#define HAL_BOARD_FLYMAPLE 6
#define HAL_BOARD_LINUX    7
#define HAL_BOARD_VRBRAIN  8
#define HAL_BOARD_EMPTY    99

// default board subtype is -1
#define HAL_BOARD_SUBTYPE_NONE -1

/**
   HAL Linux sub-types, starting at 1000
 */
#define HAL_BOARD_SUBTYPE_LINUX_NONE     1000
#define HAL_BOARD_SUBTYPE_LINUX_ERLE     1001
#define HAL_BOARD_SUBTYPE_LINUX_PXF      1002
#define HAL_BOARD_SUBTYPE_LINUX_NAVIO    1003
#define HAL_BOARD_SUBTYPE_LINUX_ZYNQ     1004
#define HAL_BOARD_SUBTYPE_LINUX_BBBMINI  1005

/**
   HAL PX4 sub-types, starting at 2000
 */
#define HAL_BOARD_SUBTYPE_PX4_V1         2000
#define HAL_BOARD_SUBTYPE_PX4_V2         2001

/**
   HAL AVR sub-types, starting at 3000
 */
#define HAL_BOARD_SUBTYPE_AVR_APM1       3000
#define HAL_BOARD_SUBTYPE_AVR_APM2       3001

// InertialSensor driver types
#define HAL_INS_OILPAN  1
#define HAL_INS_MPU6000 2
#define HAL_INS_HIL     3
#define HAL_INS_PX4     4
#define HAL_INS_FLYMAPLE 5
#define HAL_INS_L3G4200D 6
#define HAL_INS_VRBRAIN  7
#define HAL_INS_MPU9250  8
#define HAL_INS_L3GD20   9   
#define HAL_INS_MPU9255  10

// barometer driver types
#define HAL_BARO_BMP085     1
#define HAL_BARO_MS5611     2
#define HAL_BARO_MS5611_SPI 3
#define HAL_BARO_PX4        4
#define HAL_BARO_HIL        5
#define HAL_BARO_VRBRAIN    6

// compass driver types
#define HAL_COMPASS_HMC5843   1
#define HAL_COMPASS_PX4       2
#define HAL_COMPASS_HIL       3
#define HAL_COMPASS_VRBRAIN   4

/**
   CPU classes, used to select if CPU intensive algorithms should be used

   Note that these are only approximate, not exact CPU speeds.
 */
#define HAL_CPU_CLASS_16   1   // 16Mhz, AVR2560 or similar
#define HAL_CPU_CLASS_75   2   // 75Mhz, Flymaple or similar
#define HAL_CPU_CLASS_150  3   // 150Mhz, PX4 or similar, assumes
                               // hardware floating point. Assumes tens
                               // of kilobytes of memory available
#define HAL_CPU_CLASS_1000 4   // GigaHz class, SITL, BeagleBone etc,
                               // assumes megabytes of memory available

/**
   operating system features:

   HAL implementations may define the following extra feature defines
   to 1 if available

  HAL_OS_POSIX_IO    :  has posix-like filesystem IO
 */


/*
  define AP_HAL_BOARD_DRIVER to the right hal type for this
  board. This prevents us having a mess of ifdefs in every example
  sketch
 */

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
#define AP_HAL_BOARD_DRIVER AP_HAL_AVR_APM1
#define HAL_BOARD_NAME "APM 1"
#define HAL_CPU_CLASS HAL_CPU_CLASS_16
#define HAL_STORAGE_SIZE            4096
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_INS_DEFAULT HAL_INS_OILPAN
#define HAL_BARO_DEFAULT HAL_BARO_BMP085
#define HAL_COMPASS_DEFAULT HAL_COMPASS_HMC5843
#ifndef CONFIG_HAL_BOARD_SUBTYPE
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_AVR_APM1
#endif

#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
#define AP_HAL_BOARD_DRIVER AP_HAL_AVR_APM2
#define HAL_BOARD_NAME "APM 2"
#define HAL_CPU_CLASS HAL_CPU_CLASS_16
#define HAL_STORAGE_SIZE            4096
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_INS_DEFAULT HAL_INS_MPU6000
#ifdef APM2_BETA_HARDWARE
#define HAL_BARO_DEFAULT HAL_BARO_BMP085
#else
#define HAL_BARO_DEFAULT HAL_BARO_MS5611_SPI
#endif
#define HAL_COMPASS_DEFAULT HAL_COMPASS_HMC5843
#ifndef CONFIG_HAL_BOARD_SUBTYPE
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_AVR_APM2
#endif

#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#define AP_HAL_BOARD_DRIVER AP_HAL_AVR_SITL
#define HAL_BOARD_NAME "SITL"
#define HAL_CPU_CLASS HAL_CPU_CLASS_1000
#define HAL_OS_POSIX_IO 1
#define HAL_STORAGE_SIZE            16384
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_BOARD_LOG_DIRECTORY "logs"
#define HAL_BOARD_TERRAIN_DIRECTORY "terrain"
#define HAL_INS_DEFAULT HAL_INS_HIL
#define HAL_BARO_DEFAULT HAL_BARO_HIL
#define HAL_COMPASS_DEFAULT HAL_COMPASS_HIL

#elif CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE
#define AP_HAL_BOARD_DRIVER AP_HAL_FLYMAPLE
#define HAL_BOARD_NAME "FLYMAPLE"
#define HAL_CPU_CLASS HAL_CPU_CLASS_75
#define HAL_STORAGE_SIZE            4096
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_INS_DEFAULT HAL_INS_FLYMAPLE
#define HAL_BARO_DEFAULT HAL_BARO_BMP085
#define HAL_COMPASS_DEFAULT HAL_COMPASS_HMC5843
#define HAL_SERIAL0_BAUD_DEFAULT 115200
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_NONE

#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
#define AP_HAL_BOARD_DRIVER AP_HAL_PX4
#define HAL_BOARD_NAME "PX4"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_OS_POSIX_IO 1
#define HAL_BOARD_LOG_DIRECTORY "/fs/microsd/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/fs/microsd/APM/TERRAIN"
#define HAL_INS_DEFAULT HAL_INS_PX4
#define HAL_BARO_DEFAULT HAL_BARO_PX4
#define HAL_COMPASS_DEFAULT HAL_COMPASS_PX4
#define HAL_SERIAL0_BAUD_DEFAULT 115200
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_PX4_V1
#define HAL_STORAGE_SIZE            8192
#else
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_PX4_V2
#define HAL_STORAGE_SIZE            16384
#endif

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#define AP_HAL_BOARD_DRIVER AP_HAL_Linux
#define HAL_BOARD_NAME "Linux"
#define HAL_CPU_CLASS HAL_CPU_CLASS_1000
#define HAL_OS_POSIX_IO 1
#define HAL_STORAGE_SIZE            16384
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NONE
#define HAL_BOARD_LOG_DIRECTORY "logs"
#define HAL_BOARD_TERRAIN_DIRECTORY "terrain"
#define HAL_INS_DEFAULT HAL_INS_HIL
#define HAL_BARO_DEFAULT HAL_BARO_HIL
#define HAL_COMPASS_DEFAULT HAL_COMPASS_HIL
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE
#define HAL_BOARD_LOG_DIRECTORY "/var/APM/logs"
#define HAL_BOARD_TERRAIN_DIRECTORY "/var/APM/terrain"

// for enable UART multiplex
// add by ZhaoYJ@2015-11-16
#define ENABLE_MUX 1

// to fix PWM CH un-sync issue, try to make sure APM output is synced with PRU-PWM
#define SET_MAGIC_SYNC
// Control Macro: merge new battery monitor of PXF from erle v1.1
// 1. new battery monitor from PXF
// 2. new features of common battery monitor
// add by ZhaoYJ @2015-11-30
#define NEW_BATT_MONITOR_PXF 1

// add by ZhaoYJ for merging APM 3.3.2 analogin_iio @2016-01-25
#define SMT_BATT_MONITOR_ANIN_IIO

// add by ZhaoYJ for monitoring PRU & HOST alive each other @2016-04-21
#define KEEP_ALIVE_WITH_PRU 
#define PRU_POWER_UP_TIME  30 // second
#define KEEP_ALIVE_TIME_OUT_HOST 20 // second 
#define KEEP_ALIVE_TIME_OUT_PRU 1 // second

// #define HAL_INS_DEFAULT HAL_INS_MPU9255
// #define HAL_BARO_DEFAULT HAL_BARO_MS5611_SPI
// #define HAL_COMPASS_DEFAULT HAL_COMPASS_HMC5843
#define HAL_INS_DEFAULT HAL_INS_MPU9250
#define HAL_BARO_DEFAULT HAL_BARO_MS5611_SPI
//#define HAL_COMPASS_DEFAULT HAL_COMPASS_AK8963
#define HAL_COMPASS_DEFAULT HAL_COMPASS_HMC5843
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
#define HAL_BOARD_LOG_DIRECTORY "/var/APM/logs"
#define HAL_INS_DEFAULT HAL_INS_MPU6000
#define HAL_BARO_DEFAULT HAL_BARO_MS5611
#define HAL_COMPASS_DEFAULT HAL_COMPASS_AK8963
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ZYNQ
#define HAL_BOARD_LOG_DIRECTORY "/var/APM/logs"
#define HAL_BOARD_TERRAIN_DIRECTORY "/var/APM/terrain"
// Stub the sensors out for now, at least we can build and run
#define HAL_INS_DEFAULT HAL_INS_HIL
#define HAL_BARO_DEFAULT HAL_BARO_HIL
#define HAL_COMPASS_DEFAULT HAL_COMPASS_HIL
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
#define HAL_BOARD_LOG_DIRECTORY "/var/APM/logs"
#define HAL_BOARD_TERRAIN_DIRECTORY "/var/APM/terrain"
#define HAL_INS_DEFAULT HAL_INS_MPU9250
#define HAL_BARO_DEFAULT HAL_BARO_MS5611_SPI
#define HAL_COMPASS_DEFAULT HAL_COMPASS_AK8963
#else
#error "no Linux board subtype set"
#endif

#elif CONFIG_HAL_BOARD == HAL_BOARD_EMPTY
#define AP_HAL_BOARD_DRIVER AP_HAL_Empty
#define HAL_BOARD_NAME "EMPTY"
#define HAL_CPU_CLASS HAL_CPU_CLASS_16
#define HAL_STORAGE_SIZE            4096
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_INS_DEFAULT HAL_INS_HIL
#define HAL_BARO_DEFAULT HAL_BARO_HIL
#define HAL_COMPASS_DEFAULT HAL_COMPASS_HIL
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_NONE

#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#define AP_HAL_BOARD_DRIVER AP_HAL_VRBRAIN
#define HAL_BOARD_NAME "VRBRAIN"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_OS_POSIX_IO 1
#define HAL_STORAGE_SIZE            8192
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_BOARD_LOG_DIRECTORY "/fs/microsd/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/fs/microsd/APM/TERRAIN"
#define HAL_INS_DEFAULT HAL_INS_VRBRAIN
#define HAL_BARO_DEFAULT HAL_BARO_VRBRAIN
#define HAL_COMPASS_DEFAULT HAL_COMPASS_VRBRAIN
#define HAL_SERIAL0_BAUD_DEFAULT 115200
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_NONE

#else
#error "Unknown CONFIG_HAL_BOARD type"
#endif

#ifndef CONFIG_HAL_BOARD_SUBTYPE
#error "No CONFIG_HAL_BOARD_SUBTYPE set"
#endif


#endif // __AP_HAL_BOARDS_H__
