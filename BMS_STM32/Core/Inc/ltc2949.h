/*
 * ltc2949.h
 *
 *  Created on: Sep 24, 2020
 *      Author: Waterloop
 */

#ifndef INC_LTC2949_H_
#define INC_LTC2949_H_



#endif /* INC_LTC2949_H_ */


/* SPI defines */
#define LTC2949_DEFAULT_SPIMODE
#define LTC2949_DEFAULT_SPIFREQU    500000
#define LTC2949_MAX_SPIFREQU        1000000

// max. core boot-up time after power-up or reset (ms)
// typical boot-up time is 73ms. 10% margin is more than enough leading to 80ms
#define LTC2949_TIMING_BOOTUP              100U
// Time after power-up or reset to start polling if core is read (boot-up done, ms)
// the max. wakeup time spec is above. But we start polling for device to be ready earlier which is ok.
#define LTC2949_TIMING_START_POLL_CORE_RDY 10U
// max. CONT mode cycle time (ms)
#define LTC2949_TIMING_CONT_CYCLE          105U
// max. time it takes to lock the memory, worst case is twice CONT cycle time plus some margin (ms)
#define LTC2949_TIMING_MLOCK_ACK           (2*LTC2949_TIMING_CONT_CYCLE+15U)
// max. IDLE cycle time (ms)
// typical IDLE cycle time is 17 ms. 10% margin is more than enough leading to 19ms
#define LTC2949_TIMING_IDLE_CYCLE          19U
// max. power-up to auto sleep time (ms)
#define LTC2949_TIMING_AUTO_SLEEP_MAX      1500U
// min. power-up to auto sleep time (ms)
#define LTC2949_TIMING_AUTO_SLEEP          1000U
// max. time from CONT enable to 1st slow channel values UPDATE done
#define LTC2949_TIMING_IDLE2CONT2UPDATE    160U
// max. time from TB4 update to STATUS/ALERTS update
#define LTC2949_TIMING_TB4_TO_STATS        20U

/*!

|MD| Dec  | ADC Conversion Model|
|--|------|---------------------|
|01| 1    | Fast            |
|10| 2    | Normal        |
|11| 3    | Filtered          |
*/
#define MD_422HZ  	0
#define MD_FAST 	1
#define MD_NORMAL 	2
#define MD_FILTERED 3

/*!
|CH | Dec  | Channels to convert |
|---|------|---------------------|
|000| 0    | All Cells       |
|001| 1    | Cell 1 and Cell 7   |
|010| 2    | Cell 2 and Cell 8   |
|011| 3    | Cell 3 and Cell 9   |
|100| 4    | Cell 4 and Cell 10  |
|101| 5    | Cell 5 and Cell 11  |
|110| 6    | Cell 6 and Cell 12  |
*/
#define CELL_CH_ALL 	0
#define CELL_CH_1and7 	1
#define CELL_CH_2and8 	2
#define CELL_CH_3and9 	3
#define CELL_CH_4and10 	4
#define CELL_CH_5and11 	5
#define CELL_CH_6and12 	6

/*!

|CHG | Dec  |Channels to convert   |
|----|------|----------------------|
|000 | 0    | All GPIOS and 2nd Ref|
|001 | 1    | GPIO 1           |
|010 | 2    | GPIO 2               |
|011 | 3    | GPIO 3           |
|100 | 4    | GPIO 4           |
|101 | 5    | GPIO 5         |
|110 | 6    | Vref2            |
*/
#define AUX_CH_ALL 0
#define AUX_CH_GPIO1 1
#define AUX_CH_GPIO2 2
#define AUX_CH_GPIO3 3
#define AUX_CH_GPIO4 4
#define AUX_CH_GPIO5 5
#define AUX_CH_VREF2 6

// WRITE command
#define LTC2949_68XX_CMDTYPE_WR   (0U<<11U)
// READ command
#define LTC2949_68XX_CMDTYPE_RD   (1U<<11)
// POLL command
#define LTC2949_68XX_CMDTYPE_PL   (2U<<11U)
// RD&POLL command (to be used with LTC2949 only to poll EOC after fast single shot)
#define LTC2949_68XX_CMDTYPE_RDPL (LTC2949_68XX_CMDTYPE_RD|LTC2949_68XX_CMDTYPE_PL)
// command type mask
#define LTC2949_68XX_CMDTYPE_MASK (3U<<11)
