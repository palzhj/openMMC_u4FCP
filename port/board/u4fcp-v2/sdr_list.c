/*
 *   openMMC -- Open Source modular IPM Controller firmware
 *
 *   Copyright (C) 2015-2016  Henrique Silva <henrique.silva@lnls.br>
 *   Copyright (C) 2021  Krzysztof Macias <krzysztof.macias@creotech.pl>
 *   Copyright (C) 2025  Jie Zhang  <zhj@ihep.ac.cn>
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *   @license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>
 */

/* Table 42 ipmi-second-gen */

/* Project Includes */
#include "port.h"
#include "sdr.h"
#include "utils.h"
#include "i2c_mapping.h"

/* Sensors includes */
#include "sensors.h"

/* SDR List */

#ifdef MODULE_HOTSWAP
/* AMC Hot-Swap sensor SDR */
const SDR_type_02h_t SDR_HOTSWAP_AMC = {

    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_02,
    .hdr.reclength = sizeof(SDR_type_02h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00,   /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00,  /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1,                   /* entity id: AMC Module */
    .entityinstance = 0x00,             /* entity instance -> SDR_Init */
    .sensorinit = 0x03,                 /* init: event generation + scanning enabled */
    .sensorcap = 0xc1,                  /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_HOT_SWAP, /* sensor type: HOT SWAP*/
    .event_reading_type = 0x6f,         /* sensor reading*/
    .assertion_event_mask = 0x0000,     /* assertion event mask */
    .deassertion_event_mask = 0x0000,   /* deassertion event mask */
    .readable_threshold_mask = 0x00,    /* LSB: readable Threshold mask: no thresholds are readable:  */
    .settable_threshold_mask = 0x00,    /* MSB: setable Threshold mask: no thresholds are setable: */
    .sensor_units_1 = 0xc0,             /* sensor units 1 : Does not return analog reading*/
    .sensor_units_2 = 0x00,             /* sensor units 2 :*/
    .sensor_units_3 = 0x00,             /* sensor units 3 :*/
    .record_sharing[0] = 0x00,
    .record_sharing[1] = 0x00,
    .pos_thr_hysteresis = 0x00,                  /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 0x00,                  /* negative going Threshold hysteresis value */
    .reserved1 = 0x00,                           /* reserved */
    .reserved2 = 0x00,                           /* reserved */
    .reserved3 = 0x00,                           /* reserved */
    .OEM = 0x00,                                 /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE("HOTSWAP AMC"), /* 8 bit ASCII, number of bytes */
    .IDstring = "HOTSWAP AMC"                    /* sensor string */
};
#endif

#ifdef MODULE_UCD90XXX

#include "payload.h"
extern TaskHandle_t vTaskUCD_Handle;

#define SDR_AMC_12V_ID "AMC +12V"
#define SDR_RTM_12V_CURR_ID "RTM +12V Curr"
#define SDR_FMC0_12V_CURR_ID "FMC0 +12V Curr"
#define SDR_FMC0_VADJ_ID "FMC0 VADJ"
#define SDR_FMC0_VADJ_CURR_ID "FMC0 VADJ Curr"
#define SDR_FMC0_3V3_CURR_ID "FMC0 +3.3V Curr"
#define SDR_FMC1_12V_CURR_ID "FMC1 +12V Curr"
#define SDR_FMC1_VADJ_ID "FMC1 VADJ"
#define SDR_FMC1_VADJ_CURR_ID "FMC1 VADJ Curr"
#define SDR_FMC1_3V3_CURR_ID "FMC1 +3.3V Curr"
#define SDR_AMC_FPGA_0V85_ID "FPGA +0.85V"
#define SDR_AMC_FPGA_0V85_CURR_ID "FPGA +0.85V Curr"
#define SDR_AMC_FPGA_1V8_ID "FPGA +1.8V"
#define SDR_AMC_FPGA_1V8_CURR_ID "FPGA +1.8V Curr"
#define SDR_AMC_FPGA_0V9A_ID "FPGA +0.9VA"
#define SDR_AMC_FPGA_0V9A_CURR_ID "FPGA +0.9VA Curr"
#define SDR_AMC_FPGA_1V2A_ID "FPGA +1.2VA"
#define SDR_AMC_FPGA_1V2A_CURR_ID "FPGA +1.2VA Curr"
#define SDR_AMC_DDR_1V2T_ID "DDR +1.2VT"
#define SDR_AMC_DDR_1V2T_CURR_ID "DDR +1.2VT Curr"
#define SDR_AMC_DDR_1V2B_ID "DDR +1.2VB"
#define SDR_AMC_DDR_1V2B_CURR_ID "DDR +1.2VB Curr"
#define SDR_AMC_IO_3V3_ID "IO +3.3V"
#define SDR_AMC_IO_3V3_CURR_ID "IO +3.3V Curr"

/* AMC 12V */
const SDR_type_01h_t SDR_AMC_12V = {

    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00,   /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00,  /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1,                             /* entity id: AMC Module */
    .entityinstance = 0x00,                       /* entity instance -> SDR_Init */
    .sensorinit = 0x7F,                           /* init: event generation + scanning enabled */
    .sensorcap = 0x56,                            /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_VOLTAGE,            /* sensor type: Voltage*/
    .event_reading_type = 0x01,                   /* sensor reading*/
    .assertion_event_mask = 0x7A95,               /* assertion event mask (All upper going-high and lower going-low events) */
    .deassertion_event_mask = 0x7A95,             /* deassertion event mask (All upper going-high and lower going-low events) */
    .readable_threshold_mask = 0x3F,              /* LSB: readable Threshold mask: all thresholds are readable:  */
    .settable_threshold_mask = 0x00,              /* MSB: setable Threshold mask: none of the thresholds are setable: */
    .sensor_units_1 = 0x00,                       /* sensor units 1 :*/
    .sensor_units_2 = 0x04,                       /* sensor units 2 :*/
    .sensor_units_3 = 0x00,                       /* sensor units 3 :*/
    .linearization = 0x00,                        /* Linearization */
    .M = 64,                                      /* M */
    .M_tol = 0x00,                                /* M - Tolerance */
    .B = 0x00,                                    /* B */
    .B_accuracy = 0x00,                           /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00,                   /* Sensor direction */
    .Rexp_Bexp = 0xD0,                            /* R-Exp = -3 , B-Exp = 0 */
    .analog_flags = 0x03,                         /* Analogue characteristics flags */
    .nominal_reading = (12000 >> 6),              /* Nominal reading [(M * x + B * 10^(B_exp)) * 10^(R_exp)] = 12.032 V */
    .normal_max = (13000 >> 6),                   /* Normal maximum = 12.544 V */
    .normal_min = (11000 >> 6),                   /* Normal minimum = 11.456 V */
    .sensor_max_reading = 0xFF,                   /* Sensor Maximum reading */
    .sensor_min_reading = 0x00,                   /* Sensor Minimum reading */
    .upper_nonrecover_thr = (16000 >> 6),         /* Upper non-recoverable Threshold = 13.056 V */
    .upper_critical_thr = (15000 >> 6),           /* Upper critical Threshold = 12.608 V */
    .upper_noncritical_thr = (14000 >> 6),        /* Upper non critical Threshold = 12.48 V */
    .lower_nonrecover_thr = (8000 >> 6),          /* Lower non-recoverable Threshold = 11.008 V */
    .lower_critical_thr = (9000 >> 6),            /* Lower critical Threshold = 11.392 V */
    .lower_noncritical_thr = (10000 >> 6),        /* Lower non-critical Threshold = 11.52 V */
    .pos_thr_hysteresis = 2,                      /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2,                      /* negative going Threshold hysteresis value */
    .reserved1 = 0x00,                            /* reserved */
    .reserved2 = 0x00,                            /* reserved */
    .OEM = UCD_AMC_12V_ID,                                     /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE(SDR_AMC_12V_ID), /* 8 bit ASCII, number of bytes */
    .IDstring = SDR_AMC_12V_ID                    /* sensor string */
};

/* FMC0 PVADJ */
const SDR_type_01h_t SDR_FMC0_VADJ = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_VOLTAGE, /* sensor type: voltage*/
  .event_reading_type = 0x01, /* sensor reading*/
  .assertion_event_mask = 0x7A95, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A95, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x00, /* sensor units 1 :*/
  .sensor_units_2 = 0x04, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .linearization = 0x00, /* Linearization */
  .M = 16, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = (1350 >> 4), /* Nominal reading */
  .normal_max = (1950 >> 4), /* Normal maximum */
  .normal_min = (850 >> 4), /* Normal minimum */
  .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
  .sensor_min_reading = 0x00, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (2100 >> 4), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (2050 >> 4), /* Upper critical Threshold */
  .upper_noncritical_thr = (2000 >> 4), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (700 >> 4), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (750 >> 4), /* Lower critical Threshold */
  .lower_noncritical_thr = (800 >> 4), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 1, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 1, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_FMC0_VADJ_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_FMC0_VADJ_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_FMC0_VADJ_ID /* sensor string */
};

/* FMC1 PVADJ */
const SDR_type_01h_t SDR_FMC1_VADJ = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_VOLTAGE, /* sensor type: voltage*/
  .event_reading_type = 0x01, /* sensor reading*/
  .assertion_event_mask = 0x7A95, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A95, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x00, /* sensor units 1 :*/
  .sensor_units_2 = 0x04, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .linearization = 0x00, /* Linearization */
  .M = 16, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = (1350 >> 4), /* Nominal reading */
  .normal_max = (1950 >> 4), /* Normal maximum */
  .normal_min = (850 >> 4), /* Normal minimum */
  .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
  .sensor_min_reading = 0x00, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (2100 >> 4), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (2050 >> 4), /* Upper critical Threshold */
  .upper_noncritical_thr = (2000 >> 4), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (700 >> 4), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (750 >> 4), /* Lower critical Threshold */
  .lower_noncritical_thr = (800 >> 4), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 1, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 1, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_FMC1_VADJ_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_FMC1_VADJ_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_FMC1_VADJ_ID /* sensor string */
};

/* AMC FPGA 0V85 */
const SDR_type_01h_t SDR_AMC_FPGA_0V85 = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_VOLTAGE, /* sensor type: VOLTAGE*/
  .event_reading_type = 0x01, /* sensor reading*/
  .assertion_event_mask = 0x7A95, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A95, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x00, /* sensor units 1 :*/
  .sensor_units_2 = 0x04, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .linearization = 0x00, /* Linearization */
  .M = 8, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = (850 >> 3), /* Nominal reading */
  .normal_max = (900 >> 3), /* Normal maximum */
  .normal_min = (800 >> 3), /* Normal minimum */
  .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
  .sensor_min_reading = 0x00, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (1150 >> 3), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (1000 >> 3), /* Upper critical Threshold */
  .upper_noncritical_thr = (950 >> 3), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (650 >> 3), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (700 >> 3), /* Lower critical Threshold */
  .lower_noncritical_thr = (750 >> 3), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_FPGA_0V85_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_AMC_FPGA_0V85_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_AMC_FPGA_0V85_ID /* sensor string */
};

/* AMC FPGA 1V8 */
const SDR_type_01h_t SDR_AMC_FPGA_1V8 = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_VOLTAGE, /* sensor type: VOLTAGE*/
  .event_reading_type = 0x01, /* sensor reading*/
  .assertion_event_mask = 0x7A95, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A95, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x00, /* sensor units 1 :*/
  .sensor_units_2 = 0x04, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .linearization = 0x00, /* Linearization */
  .M = 16, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = (1800 >> 4), /* Nominal reading */
  .normal_max = (1900 >> 4), /* Normal maximum */
  .normal_min = (1700 >> 4), /* Normal minimum */
  .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
  .sensor_min_reading = 0x00, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (2050 >> 4), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (2000 >> 4), /* Upper critical Threshold */
  .upper_noncritical_thr = (1950 >> 4), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (1550 >> 4), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (1600 >> 4), /* Lower critical Threshold */
  .lower_noncritical_thr = (1650 >> 4), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_FPGA_1V8_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_AMC_FPGA_1V8_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_AMC_FPGA_1V8_ID /* sensor string */
};

/* AMC FPGA +0.9V Analog */
const SDR_type_01h_t SDR_AMC_FPGA_0V9A = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_VOLTAGE, /* sensor type: VOLTAGE*/
  .event_reading_type = 0x01, /* sensor reading*/
  .assertion_event_mask = 0x7A95, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A95, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x00, /* sensor units 1 :*/
  .sensor_units_2 = 0x04, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .linearization = 0x00, /* Linearization */
  .M = 8, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = (900 >> 3), /* Nominal reading */
  .normal_max = (950 >> 3), /* Normal maximum */
  .normal_min = (850 >> 3), /* Normal minimum */
  .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
  .sensor_min_reading = 0x00, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (1100 >> 3), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (1050 >> 3), /* Upper critical Threshold */
  .upper_noncritical_thr = (1000 >> 3), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (700 >> 3), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (750 >> 3), /* Lower critical Threshold */
  .lower_noncritical_thr = (800 >> 3), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_FPGA_0V9A_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_AMC_FPGA_0V9A_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_AMC_FPGA_0V9A_ID /* sensor string */
};

/* AMC FPGA +1.2V Analog */
const SDR_type_01h_t SDR_AMC_FPGA_1V2A = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_VOLTAGE, /* sensor type: VOLTAGE*/
  .event_reading_type = 0x01, /* sensor reading*/
  .assertion_event_mask = 0x7A95, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A95, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x00, /* sensor units 1 :*/
  .sensor_units_2 = 0x04, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .linearization = 0x00, /* Linearization */
  .M = 8, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = (1200 >> 3), /* Nominal reading */
  .normal_max = (1250 >> 3), /* Normal maximum */
  .normal_min = (1150 >> 3), /* Normal minimum */
  .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
  .sensor_min_reading = 0x00, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (1400 >> 3), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (1350 >> 3), /* Upper critical Threshold */
  .upper_noncritical_thr = (1300 >> 3), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (1000 >> 3), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (1050 >> 3), /* Lower critical Threshold */
  .lower_noncritical_thr = (1100 >> 3), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_FPGA_1V2A_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_AMC_FPGA_1V2A_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_AMC_FPGA_1V2A_ID /* sensor string */
};

/* AMC DDR +1.2V TOP */
const SDR_type_01h_t SDR_AMC_DDR_1V2T = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_VOLTAGE, /* sensor type: VOLTAGE*/
  .event_reading_type = 0x01, /* sensor reading*/
  .assertion_event_mask = 0x7A95, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A95, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x00, /* sensor units 1 :*/
  .sensor_units_2 = 0x04, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .linearization = 0x00, /* Linearization */
  .M = 8, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = (1200 >> 3), /* Nominal reading */
  .normal_max = (1250 >> 3), /* Normal maximum */
  .normal_min = (1150 >> 3), /* Normal minimum */
  .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
  .sensor_min_reading = 0x00, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (1400 >> 3), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (1350 >> 3), /* Upper critical Threshold */
  .upper_noncritical_thr = (1300 >> 3), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (1000 >> 3), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (1050 >> 3), /* Lower critical Threshold */
  .lower_noncritical_thr = (1100 >> 3), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_DDR_1V2T_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_AMC_DDR_1V2T_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_AMC_DDR_1V2T_ID /* sensor string */
};

/* AMC DDR +1.2V BOTTOM */
const SDR_type_01h_t SDR_AMC_DDR_1V2B = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_VOLTAGE, /* sensor type: VOLTAGE*/
  .event_reading_type = 0x01, /* sensor reading*/
  .assertion_event_mask = 0x7A95, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A95, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x00, /* sensor units 1 :*/
  .sensor_units_2 = 0x04, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .linearization = 0x00, /* Linearization */
  .M = 8, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = (1200 >> 3), /* Nominal reading */
  .normal_max = (1250 >> 3), /* Normal maximum */
  .normal_min = (1150 >> 3), /* Normal minimum */
  .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
  .sensor_min_reading = 0x00, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (1400 >> 3), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (1350 >> 3), /* Upper critical Threshold */
  .upper_noncritical_thr = (1300 >> 3), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (1000 >> 3), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (1050 >> 3), /* Lower critical Threshold */
  .lower_noncritical_thr = (1100 >> 3), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_DDR_1V2B_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_AMC_DDR_1V2B_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_AMC_DDR_1V2B_ID /* sensor string */
};

/* AMC IO 3V3 */
const SDR_type_01h_t SDR_AMC_IO_3V3 = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_VOLTAGE, /* sensor type: VOLTAGE*/
  .event_reading_type = 0x01, /* sensor reading*/
  .assertion_event_mask = 0x7A95, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A95, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x00, /* sensor units 1 :*/
  .sensor_units_2 = 0x04, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .linearization = 0x00, /* Linearization */
  .M = 16, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = (3300 >> 4), /* Nominal reading */
  .normal_max = (3500 >> 4), /* Normal maximum */
  .normal_min = (3000 >> 4), /* Normal minimum */
  .sensor_max_reading = 0xFF, /* Sensor Maximum reading */
  .sensor_min_reading = 0x00, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (3700 >> 4), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (3600 >> 4), /* Upper critical Threshold */
  .upper_noncritical_thr = (3550 >> 4), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (2800 >> 4), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (2900 >> 4), /* Lower critical Threshold */
  .lower_noncritical_thr = (2950 >> 4), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_AMC_IO_3V3_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_AMC_IO_3V3_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_AMC_IO_3V3_ID /* sensor string */
};


/* RTM 12V Current */
const SDR_type_01h_t SDR_RTM_12V_CURR = {

  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm */
  .sensortype = SENSOR_TYPE_CURRENT, /* sensor type: Current */
  .event_reading_type = 0x01, /* sensor reading*/
  .assertion_event_mask = 0x0A80, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A80, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x80, /* sensor units 1 :*/
  .sensor_units_2 = 0x05, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .linearization = 0x00, /* Linearization */
  .M = 64, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp = -3 , B-Exp = 0 */
  .analog_flags = 0x03, /* Analogue characteristics flags */
  .nominal_reading = (2000 >> 6), /* Nominal reading */
  .normal_max = (10000 >> 6), /* Normal maximum */
  .normal_min = (0 >> 6), /* Normal minimum */
  .sensor_max_reading = 0x7F, /* Sensor Maximum reading */
  .sensor_min_reading = 0x80, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (11000 >> 6), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (10500 >> 6), /* Upper critical Threshold */
  .upper_noncritical_thr = (10000 >> 6), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (0 >> 6), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (0 >> 6), /* Lower critical Threshold */
  .lower_noncritical_thr = (0 >> 6), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_RTM_12V_CURR_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_RTM_12V_CURR_ID) , /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_RTM_12V_CURR_ID /* sensor string */
};

/* FMC0 12V Current */
const SDR_type_01h_t SDR_FMC0_12V_CURR = {
    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00, /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00, /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1, /* entity id: AMC Module */
    .entityinstance = 0x00, /* entity instance -> SDR_Init */
    .sensorinit = 0x7F, /* init: event generation + scanning enabled */
    .sensorcap = 0x56, /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_CURRENT, /* sensor type: Current */
    .event_reading_type = 0x01, /* sensor reading */
    .assertion_event_mask = 0x0A80, /* assertion event mask (All upper going-high and lower going-low events) */
    .deassertion_event_mask = 0x7A80, /* deassertion event mask (All upper going-high and lower going-low events) */
    .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
    .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
    .sensor_units_1 = 0x80, /* sensor units 1 :*/
    .sensor_units_2 = 0x05, /* sensor units 2 :*/
    .sensor_units_3 = 0x00, /* sensor units 3 :*/
    .linearization = 0x00, /* Linearization */
    .M = 32, /* M */
    .M_tol = 0x00, /* M - Tolerance */
    .B = 0x00, /* B */
    .B_accuracy = 0x00, /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00, /* Sensor direction */
    .Rexp_Bexp = 0xD0, /* R-Exp = -3 , B-Exp = 0 */
    .analog_flags = 0x03, /* Analogue characteristics flags */
    .nominal_reading = (1000 >> 5), /* Nominal reading */
    .normal_max = (2000 >> 5), /* Normal maximum */
    .normal_min = 0, /* Normal minimum */
    .sensor_max_reading = 0x7F, /* Sensor Maximum reading */
    .sensor_min_reading = 0x80, /* Sensor Minimum reading */
    .upper_nonrecover_thr = (3500 >> 5), /* Upper non-recoverable Threshold */
    .upper_critical_thr = (3000 >> 5), /* Upper critical Threshold */
    .upper_noncritical_thr = (2500 >> 5), /* Upper non critical Threshold */
    .lower_nonrecover_thr =  (0 >> 5), /* Lower non-recoverable Threshold */
    .lower_critical_thr = (0 >> 5), /* Lower critical Threshold */
    .lower_noncritical_thr = (0 >> 5), /* Lower non-critical Threshold */
    .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
    .reserved1 = 0x00, /* reserved */
    .reserved2 = 0x00, /* reserved */
    .OEM = UCD_FMC0_12V_CURR_ID, /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE(SDR_FMC0_12V_CURR_ID) , /* 8 bit ASCII, number of bytes */
    .IDstring = SDR_FMC0_12V_CURR_ID /* sensor string */
};

/* FMC0 PVADJ Current */
const SDR_type_01h_t SDR_FMC0_VADJ_CURR = {
    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00, /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00, /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1, /* entity id: AMC Module */
    .entityinstance = 0x00, /* entity instance -> SDR_Init */
    .sensorinit = 0x7F, /* init: event generation + scanning enabled */
    .sensorcap = 0x56, /* capabilities: auto re-arm */
    .sensortype = SENSOR_TYPE_CURRENT, /* sensor type: CURRENT */
    .event_reading_type = 0x01, /* sensor reading*/
    .assertion_event_mask = 0x0A80, /* assertion event mask (All upper going-high and lower going-low events) */
    .deassertion_event_mask = 0x7A80, /* deassertion event mask (All upper going-high and lower going-low events) */
    .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
    .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
    .sensor_units_1 = 0x80, /* sensor units 1 :*/
    .sensor_units_2 = 0x05, /* sensor units 2 :*/
    .sensor_units_3 = 0x00, /* sensor units 3 :*/
    .linearization = 0x00, /* Linearization */
    .M = 32, /* M */
    .M_tol = 0x00, /* M - Tolerance */
    .B = 0x00, /* B */
    .B_accuracy = 0x00, /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00, /* Sensor direction */
    .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
    .analog_flags = 0x00, /* Analogue characteristics flags */
    .nominal_reading = 1000 >> 5, /* Nominal reading */
    .normal_max = 1500 >> 5, /* Normal maximum */
    .normal_min = 0, /* Normal minimum - 0A */
    .sensor_max_reading = 0x7F, /* Sensor Maximum reading */
    .sensor_min_reading = 0x80, /* Sensor Minimum reading */
    .upper_nonrecover_thr = (2000 >> 5), /* Upper non-recoverable Threshold */
    .upper_critical_thr = (1700 >> 5), /* Upper critical Threshold */
    .upper_noncritical_thr = (1500 >> 5), /* Upper non critical Threshold */
    .lower_nonrecover_thr =  (0 >> 5), /* Lower non-recoverable Threshold */
    .lower_critical_thr =  (0 >> 5), /* Lower critical Threshold */
    .lower_noncritical_thr = (0 >> 5), /* Lower non-critical Threshold */
    .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
    .reserved1 = 0x00, /* reserved */
    .reserved2 = 0x00, /* reserved */
    .OEM = UCD_FMC0_VADJ_ID, /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE(SDR_FMC0_VADJ_CURR_ID), /* 8 bit ASCII, number of bytes */
    .IDstring = SDR_FMC0_VADJ_CURR_ID /* sensor string */
};

/* FMC0 P3V3 Current */
const SDR_type_01h_t SDR_FMC0_P3V3_CURR = {
    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00, /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00, /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1, /* entity id: AMC Module */
    .entityinstance = 0x00, /* entity instance -> SDR_Init */
    .sensorinit = 0x7F, /* init: event generation + scanning enabled */
    .sensorcap = 0x56, /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_CURRENT, /* sensor type: CURRENT */
    .event_reading_type = 0x01, /* sensor reading */
    .assertion_event_mask = 0x0A80, /* assertion event mask (All upper going-high and lower going-low events) */
    .deassertion_event_mask = 0x7A80, /* deassertion event mask (All upper going-high and lower going-low events) */
    .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
    .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
    .sensor_units_1 = 0x80, /* sensor units 1 :*/
    .sensor_units_2 = 0x05, /* sensor units 2 :*/
    .sensor_units_3 = 0x00, /* sensor units 3 :*/
    .M = 32, /* M */
    .M_tol = 0x00, /* M - Tolerance */
    .B = 0x00, /* B */
    .B_accuracy = 0x00, /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00, /* Sensor direction */
    .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
    .analog_flags = 0x00, /* Analogue characteristics flags */
    .nominal_reading = (1500 >> 5), /* Nominal reading */
    .normal_max = (3000 >> 5), /* Normal maximum */
    .normal_min = 0, /* Normal minimum */
    .sensor_max_reading = 0x7F, /* Sensor Maximum reading */
    .sensor_min_reading = 0x80, /* Sensor Minimum reading */
    .upper_nonrecover_thr = (4000 >> 5), /* Upper non-recoverable Threshold */
    .upper_critical_thr = (3500 >> 5), /* Upper critical Threshold */
    .upper_noncritical_thr = (3000 >> 5), /* Upper non critical Threshold */
    .lower_nonrecover_thr = (0 >> 5), /* Lower non-recoverable Threshold */
    .lower_critical_thr = (0 >> 5), /* Lower critical Threshold */
    .lower_noncritical_thr = (0 >> 5), /* Lower non-critical Threshold */
    .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
    .reserved1 = 0x00, /* reserved */
    .reserved2 = 0x00, /* reserved */
    .OEM = UCD_FMC0_3V3_CURR_ID, /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE(SDR_FMC0_3V3_CURR_ID), /* 8 bit ASCII, number of bytes */
    .IDstring = SDR_FMC0_3V3_CURR_ID /* sensor string */
};

/* FMC1 12V Current */
const SDR_type_01h_t SDR_FMC1_12V_CURR = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_CURRENT, /* sensor type: Current */
  .event_reading_type = 0x01, /* sensor reading */
  .assertion_event_mask = 0x0A80, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A80, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x80, /* sensor units 1 :*/
  .sensor_units_2 = 0x05, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .linearization = 0x00, /* Linearization */
  .M = 32, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp = -3 , B-Exp = 0 */
  .analog_flags = 0x03, /* Analogue characteristics flags */
  .nominal_reading = (1000 >> 5), /* Nominal reading */
  .normal_max = (2000 >> 5), /* Normal maximum */
  .normal_min = 0, /* Normal minimum */
  .sensor_max_reading = 0x7F, /* Sensor Maximum reading */
  .sensor_min_reading = 0x80, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (3500 >> 5), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (3000 >> 5), /* Upper critical Threshold */
  .upper_noncritical_thr = (2500 >> 5), /* Upper non critical Threshold */
  .lower_nonrecover_thr =  (0 >> 5), /* Lower non-recoverable Threshold */
  .lower_critical_thr =  (0 >> 5), /* Lower critical Threshold */
  .lower_noncritical_thr = (0 >> 5), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_FMC1_12V_CURR_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_FMC1_12V_CURR_ID) , /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_FMC1_12V_CURR_ID /* sensor string */
};

/* FMC1 PVADJ Current */
const SDR_type_01h_t SDR_FMC1_VADJ_CURR = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm */
  .sensortype = SENSOR_TYPE_CURRENT, /* sensor type: CURRENT */
  .event_reading_type = 0x01, /* sensor reading*/
  .assertion_event_mask = 0x0A80, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A80, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x80, /* sensor units 1 :*/
  .sensor_units_2 = 0x05, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .linearization = 0x00, /* Linearization */
  .M = 32, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = 1000 >> 5, /* Nominal reading */
  .normal_max = 1500 >> 5, /* Normal maximum */
  .normal_min = 0, /* Normal minimum - 0A */
  .sensor_max_reading = 0x7F, /* Sensor Maximum reading */
  .sensor_min_reading = 0x80, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (2000 >> 5), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (1700 >> 5), /* Upper critical Threshold */
  .upper_noncritical_thr = (1500 >> 5), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (0 >> 5), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (0 >> 5), /* Lower critical Threshold */
  .lower_noncritical_thr = (0 >> 5), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_FMC1_VADJ_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_FMC1_VADJ_CURR_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_FMC1_VADJ_CURR_ID /* sensor string */
};

/* FMC1 P3V3 Current */
const SDR_type_01h_t SDR_FMC1_P3V3_CURR = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_CURRENT, /* sensor type: CURRENT */
  .event_reading_type = 0x01, /* sensor reading */
  .assertion_event_mask = 0x0A80, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A80, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x80, /* sensor units 1 :*/
  .sensor_units_2 = 0x05, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .M = 32, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = (1500 >> 5), /* Nominal reading */
  .normal_max = (3000 >> 5), /* Normal maximum */
  .normal_min = 0, /* Normal minimum */
  .sensor_max_reading = 0x7F, /* Sensor Maximum reading */
  .sensor_min_reading = 0x80, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (4000 >> 5), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (3500 >> 5), /* Upper critical Threshold */
  .upper_noncritical_thr = (3000 >> 5), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (0 >> 5), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (0 >> 5), /* Lower critical Threshold */
  .lower_noncritical_thr = (0 >> 5), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_FMC1_3V3_CURR_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_FMC1_3V3_CURR_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_FMC1_3V3_CURR_ID /* sensor string */
};

/* AMC FPGA 0V85 Current */
const SDR_type_01h_t SDR_AMC_FPGA_0V85_CURR = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm */
  .sensortype = SENSOR_TYPE_CURRENT, /* sensor type: CURRENT */
  .event_reading_type = 0x01, /* sensor reading*/
  .assertion_event_mask = 0x0A80, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A80, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x80, /* sensor units 1 :*/
  .sensor_units_2 = 0x05, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .linearization = 0x00, /* Linearization */
  .M = 128, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = 10000 >> 7, /* Nominal reading */
  .normal_max = 30000 >> 7, /* Normal maximum */
  .normal_min = 0, /* Normal minimum - 0A */
  .sensor_max_reading = 0x7F, /* Sensor Maximum reading */
  .sensor_min_reading = 0x80, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (30300 >> 7), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (30200 >> 7), /* Upper critical Threshold */
  .upper_noncritical_thr = (30100 >> 7), /* Upper non critical Threshold */
  .lower_nonrecover_thr =  (0 >> 7), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (0 >> 7), /* Lower critical Threshold */
  .lower_noncritical_thr =  (0 >> 7), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_FPGA_0V85_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_AMC_FPGA_0V85_CURR_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_AMC_FPGA_0V85_CURR_ID /* sensor string */
};

/* AMC FPGA 1V8 Current */
const SDR_type_01h_t SDR_AMC_FPGA_1V8_CURR = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_CURRENT, /* sensor type: CURRENT */
  .event_reading_type = 0x01, /* sensor reading */
  .assertion_event_mask = 0x0A80, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A80, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x80, /* sensor units 1 :*/
  .sensor_units_2 = 0x05, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .M = 64, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = (1500 >> 6), /* Nominal reading */
  .normal_max = (10000 >> 6), /* Normal maximum */
  .normal_min = 0, /* Normal minimum */
  .sensor_max_reading = 0x7F, /* Sensor Maximum reading */
  .sensor_min_reading = 0x80, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (11000 >> 6), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (10500 >> 6), /* Upper critical Threshold */
  .upper_noncritical_thr = (10000 >> 6), /* Upper non critical Threshold */
  .lower_nonrecover_thr =  (0 >> 6), /* Lower non-recoverable Threshold */
  .lower_critical_thr =  (0 >> 6), /* Lower critical Threshold */
  .lower_noncritical_thr = (0 >> 6), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_FPGA_1V8_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_AMC_FPGA_1V8_CURR_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_AMC_FPGA_1V8_CURR_ID /* sensor string */
};

/* AMC FPGA +0.9V Analog Current */
const SDR_type_01h_t SDR_AMC_FPGA_0V9A_CURR = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_CURRENT, /* sensor type: CURRENT */
  .event_reading_type = 0x01, /* sensor reading */
  .assertion_event_mask = 0x0A80, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A80, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x80, /* sensor units 1 :*/
  .sensor_units_2 = 0x05, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .M = 64, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = (1500 >> 6), /* Nominal reading */
  .normal_max = (10000 >> 6), /* Normal maximum */
  .normal_min = 0, /* Normal minimum */
  .sensor_max_reading = 0x7F, /* Sensor Maximum reading */
  .sensor_min_reading = 0x80, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (11000 >> 6), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (10500 >> 6), /* Upper critical Threshold */
  .upper_noncritical_thr = (10000 >> 6), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (0 >> 6), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (0 >> 6), /* Lower critical Threshold */
  .lower_noncritical_thr = (0 >> 6), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_FPGA_0V9A_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_AMC_FPGA_0V9A_CURR_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_AMC_FPGA_0V9A_CURR_ID /* sensor string */
};

/* AMC FPGA +1.2V Analog Current */
const SDR_type_01h_t SDR_AMC_FPGA_1V2A_CURR = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_CURRENT, /* sensor type: CURRENT */
  .event_reading_type = 0x01, /* sensor reading */
  .assertion_event_mask = 0x0A80, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A80, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x80, /* sensor units 1 :*/
  .sensor_units_2 = 0x05, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .M = 64, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = (1500 >> 6), /* Nominal reading */
  .normal_max = (10000 >> 6), /* Normal maximum */
  .normal_min = 0, /* Normal minimum */
  .sensor_max_reading = 0x7F, /* Sensor Maximum reading */
  .sensor_min_reading = 0x80, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (11000 >> 6), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (10500 >> 6), /* Upper critical Threshold */
  .upper_noncritical_thr = (10000 >> 6), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (0 >> 6), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (0 >> 6), /* Lower critical Threshold */
  .lower_noncritical_thr = (0 >> 6), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_FPGA_1V2A_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_AMC_FPGA_1V2A_CURR_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_AMC_FPGA_1V2A_CURR_ID /* sensor string */
};

/* AMC DDR +1.2V TOP Current */
const SDR_type_01h_t SDR_AMC_DDR_1V2T_CURR = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_CURRENT, /* sensor type: CURRENT */
  .event_reading_type = 0x01, /* sensor reading */
  .assertion_event_mask = 0x0A80, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A80, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x80, /* sensor units 1 :*/
  .sensor_units_2 = 0x05, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .M = 32, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = (1500 >> 5), /* Nominal reading */
  .normal_max = (3000 >> 5), /* Normal maximum */
  .normal_min = 0, /* Normal minimum */
  .sensor_max_reading = 0x7F, /* Sensor Maximum reading */
  .sensor_min_reading = 0x80, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (4000 >> 5), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (3500 >> 5), /* Upper critical Threshold */
  .upper_noncritical_thr = (3000 >> 5), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (0 >> 5), /* Lower non-recoverable Threshold */
  .lower_critical_thr =  (0 >> 5), /* Lower critical Threshold */
  .lower_noncritical_thr =  (0 >> 5), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_DDR_1V2T_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_AMC_DDR_1V2T_CURR_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_AMC_DDR_1V2T_CURR_ID /* sensor string */
};

/* AMC DDR +1.2V BOTTOM Current */
const SDR_type_01h_t SDR_AMC_DDR_1V2B_CURR = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_CURRENT, /* sensor type: CURRENT */
  .event_reading_type = 0x01, /* sensor reading */
  .assertion_event_mask = 0x0A80, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A80, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x80, /* sensor units 1 :*/
  .sensor_units_2 = 0x05, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .M = 32, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = (1500 >> 5), /* Nominal reading */
  .normal_max = (3000 >> 5), /* Normal maximum */
  .normal_min = 0, /* Normal minimum */
  .sensor_max_reading = 0x7F, /* Sensor Maximum reading */
  .sensor_min_reading = 0x80, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (4000 >> 5), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (3500 >> 5), /* Upper critical Threshold */
  .upper_noncritical_thr = (3000 >> 5), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (0 >> 5), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (0 >> 5), /* Lower critical Threshold */
  .lower_noncritical_thr =  (0 >> 5), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_DDR_1V2B_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_AMC_DDR_1V2B_CURR_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_AMC_DDR_1V2B_CURR_ID /* sensor string */
};

/* AMC IO 3V3 Current */
const SDR_type_01h_t SDR_AMC_IO_3V3_CURR = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00, /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00, /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1, /* entity id: AMC Module */
  .entityinstance = 0x00, /* entity instance -> SDR_Init */
  .sensorinit = 0x7F, /* init: event generation + scanning enabled */
  .sensorcap = 0x56, /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_CURRENT, /* sensor type: CURRENT */
  .event_reading_type = 0x01, /* sensor reading */
  .assertion_event_mask = 0x0A80, /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A80, /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F, /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00, /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x80, /* sensor units 1 :*/
  .sensor_units_2 = 0x05, /* sensor units 2 :*/
  .sensor_units_3 = 0x00, /* sensor units 3 :*/
  .M = 64, /* M */
  .M_tol = 0x00, /* M - Tolerance */
  .B = 0x00, /* B */
  .B_accuracy = 0x00, /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00, /* Sensor direction */
  .Rexp_Bexp = 0xD0, /* R-Exp , B-Exp */
  .analog_flags = 0x00, /* Analogue characteristics flags */
  .nominal_reading = (1500 >> 6), /* Nominal reading */
  .normal_max = (12000 >> 6), /* Normal maximum */
  .normal_min = 0, /* Normal minimum */
  .sensor_max_reading = 0x7F, /* Sensor Maximum reading */
  .sensor_min_reading = 0x80, /* Sensor Minimum reading */
  .upper_nonrecover_thr = (13000 >> 6), /* Upper non-recoverable Threshold */
  .upper_critical_thr = (12500 >> 6), /* Upper critical Threshold */
  .upper_noncritical_thr = (12000 >> 6), /* Upper non critical Threshold */
  .lower_nonrecover_thr = (0 >> 6), /* Lower non-recoverable Threshold */
  .lower_critical_thr = (0 >> 6), /* Lower critical Threshold */
  .lower_noncritical_thr = (0 >> 6), /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2, /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2, /* negative going Threshold hysteresis value */
  .reserved1 = 0x00, /* reserved */
  .reserved2 = 0x00, /* reserved */
  .OEM = UCD_AMC_IO_3V3_ID, /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE(SDR_AMC_IO_3V3_CURR_ID), /* 8 bit ASCII, number of bytes */
  .IDstring = SDR_AMC_IO_3V3_CURR_ID /* sensor string */
};


/* UCD Temperature*/
const SDR_type_01h_t SDR_UCD_TEMP = {
  .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
  .hdr.recID_MSB = 0x00,
  .hdr.SDRversion = 0x51,
  .hdr.rectype = TYPE_01,
  .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

  .ownerID = 0x00,   /* i2c address, -> SDR_Init */
  .ownerLUN = 0x00,  /* sensor owner LUN */
  .sensornum = 0x00, /* Filled by sdr_insert_entry() */

  /* record body bytes */
  .entityID = 0xC1,                        /* entity id: AMC Module */
  .entityinstance = 0x00,                  /* entity instance -> SDR_Init */
  .sensorinit = 0x7F,                      /* init: event generation + scanning enabled */
  .sensorcap = 0x56,                       /* capabilities: auto re-arm,*/
  .sensortype = SENSOR_TYPE_TEMPERATURE,   /* sensor type */
  .event_reading_type = 0x01,              /* sensor reading*/
  .assertion_event_mask = 0x7A95,          /* assertion event mask (All upper going-high and lower going-low events) */
  .deassertion_event_mask = 0x7A95,        /* deassertion event mask (All upper going-high and lower going-low events) */
  .readable_threshold_mask = 0x3F,         /* LSB: readable Threshold mask: all thresholds are readable:  */
  .settable_threshold_mask = 0x00,         /* MSB: setable Threshold mask: none of the thresholds are setable: */
  .sensor_units_1 = 0x00,                  /* sensor units 1 :*/
  .sensor_units_2 = 0x01,                  /* sensor units 2 :*/
  .sensor_units_3 = 0x00,                  /* sensor units 3 :*/
  .linearization = 0x00,                   /* Linearization */
  .M = 5,                                  /* M */
  .M_tol = 0x00,                           /* M - Tolerance */
  .B = 0x00,                               /* B */
  .B_accuracy = 0x00,                      /* B - Accuracy */
  .acc_exp_sensor_dir = 0x00,              /* Sensor direction */
  .Rexp_Bexp = 0xF0,                       /* R-Exp , B-Exp */
  .analog_flags = 0x03,                    /* Analogue characteristics flags */
  .nominal_reading = (24 << 1),            /* Nominal reading */
  .normal_max = (80 << 1),                 /* Normal maximum */
  .normal_min = (10 << 1),                 /* Normal minimum */
  .sensor_max_reading = 0xFF,              /* Sensor Maximum reading */
  .sensor_min_reading = 0x00,              /* Sensor Minimum reading */
  .upper_nonrecover_thr = (95 << 1),       /* Upper non-recoverable Threshold */
  .upper_critical_thr = (90 << 1),         /* Upper critical Threshold */
  .upper_noncritical_thr = (85 << 1),      /* Upper non critical Threshold */
  .lower_nonrecover_thr = - (5 << 1),        /* Lower non-recoverable Threshold */
  .lower_critical_thr = (0 << 1),          /* Lower critical Threshold */
  .lower_noncritical_thr = (5 << 1),      /* Lower non-critical Threshold */
  .pos_thr_hysteresis = 2,                 /* positive going Threshold hysteresis value */
  .neg_thr_hysteresis = 2,                 /* negative going Threshold hysteresis value */
  .reserved1 = 0x00,                       /* reserved */
  .reserved2 = 0x00,                       /* reserved */
  .OEM = 0x00,                             /* OEM reserved */
  .IDtypelen = 0xc0 | STR_SIZE("TEMP UCD"),/* 8 bit ASCII, number of bytes */
  .IDstring = "TEMP UCD"                   /* sensor string */
};

#endif

#ifdef MODULE_LM75
/* LM75 SDR List */
const SDR_type_01h_t SDR_LM75_uC = {
    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00,   /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00,  /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1,                        /* entity id: AMC Module */
    .entityinstance = 0x00,                  /* entity instance -> SDR_Init */
    .sensorinit = 0x7F,                      /* init: event generation + scanning enabled */
    .sensorcap = 0x56,                       /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_TEMPERATURE,   /* sensor type */
    .event_reading_type = 0x01,              /* sensor reading*/
    .assertion_event_mask = 0x7A95,          /* assertion event mask (All upper going-high and lower going-low events) */
    .deassertion_event_mask = 0x7A95,        /* deassertion event mask (All upper going-high and lower going-low events) */
    .readable_threshold_mask = 0x3F,         /* LSB: readable Threshold mask: all thresholds are readable:  */
    .settable_threshold_mask = 0x00,         /* MSB: setable Threshold mask: none of the thresholds are setable: */
    .sensor_units_1 = 0x00,                  /* sensor units 1 :*/
    .sensor_units_2 = 0x01,                  /* sensor units 2 :*/
    .sensor_units_3 = 0x00,                  /* sensor units 3 :*/
    .linearization = 0x00,                   /* Linearization */
    .M = 5,                                  /* M */
    .M_tol = 0x00,                           /* M - Tolerance */
    .B = 0x00,                               /* B */
    .B_accuracy = 0x00,                      /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00,              /* Sensor direction */
    .Rexp_Bexp = 0xF0,                       /* R-Exp , B-Exp */
    .analog_flags = 0x03,                    /* Analogue characteristics flags */
    .nominal_reading = (20 << 1),            /* Nominal reading */
    .normal_max = (50 << 1),                 /* Normal maximum */
    .normal_min = (10 << 1),                 /* Normal minimum */
    .sensor_max_reading = 0xFF,              /* Sensor Maximum reading */
    .sensor_min_reading = 0x00,              /* Sensor Minimum reading */
    .upper_nonrecover_thr = (80 << 1),       /* Upper non-recoverable Threshold */
    .upper_critical_thr = (75 << 1),         /* Upper critical Threshold */
    .upper_noncritical_thr = (65 << 1),      /* Upper non critical Threshold */
    .lower_nonrecover_thr = (0 << 1),        /* Lower non-recoverable Threshold */
    .lower_critical_thr = (5 << 1),          /* Lower critical Threshold */
    .lower_noncritical_thr = (10 << 1),      /* Lower non-critical Threshold */
    .pos_thr_hysteresis = 2,                 /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2,                 /* negative going Threshold hysteresis value */
    .reserved1 = 0x00,                       /* reserved */
    .reserved2 = 0x00,                       /* reserved */
    .OEM = 0x00,                             /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE("TEMP UC"), /* 8 bit ASCII, number of bytes */
    .IDstring = "TEMP UC"                    /*  sensor string */
};

const SDR_type_01h_t SDR_LM75_ADN4604 = {
    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00,   /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00,  /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1,                                /* entity id: AMC Module */
    .entityinstance = 0x00,                          /* entity instance -> SDR_Init */
    .sensorinit = 0x7F,                              /* init: event generation + scanning enabled */
    .sensorcap = 0x56,                               /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_TEMPERATURE,           /* sensor type */
    .event_reading_type = 0x01,                      /* sensor reading*/
    .assertion_event_mask = 0x7A95,                  /* assertion event mask (All upper going-high and lower going-low events) */
    .deassertion_event_mask = 0x7A95,                /* deassertion event mask (All upper going-high and lower going-low events) */
    .readable_threshold_mask = 0x3F,                 /* LSB: readable Threshold mask: all thresholds are readable:  */
    .settable_threshold_mask = 0x00,                 /* MSB: setable Threshold mask: none of the thresholds are setable: */
    .sensor_units_1 = 0x00,                          /* sensor units 1 :*/
    .sensor_units_2 = 0x01,                          /* sensor units 2 :*/
    .sensor_units_3 = 0x00,                          /* sensor units 3 :*/
    .linearization = 0x00,                           /* Linearization */
    .M = 5,                                          /* M */
    .M_tol = 0x00,                                   /* M - Tolerance */
    .B = 0x00,                                       /* B */
    .B_accuracy = 0x00,                              /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00,                      /* Sensor direction */
    .Rexp_Bexp = 0xF0,                               /* R-Exp , B-Exp */
    .analog_flags = 0x03,                            /* Analogue characteristics flags */
    .nominal_reading = (20 << 1),                    /* Nominal reading */
    .normal_max = (50 << 1),                         /* Normal maximum */
    .normal_min = (10 << 1),                         /* Normal minimum */
    .sensor_max_reading = 0xFF,                      /* Sensor Maximum reading */
    .sensor_min_reading = 0x00,                      /* Sensor Minimum reading */
    .upper_nonrecover_thr = (80 << 1),               /* Upper non-recoverable Threshold */
    .upper_critical_thr = (75 << 1),                 /* Upper critical Threshold */
    .upper_noncritical_thr = (65 << 1),              /* Upper non critical Threshold */
    .lower_nonrecover_thr = (0 << 1),                /* Lower non-recoverable Threshold */
    .lower_critical_thr = (5 << 1),                  /* Lower critical Threshold */
    .lower_noncritical_thr = (10 << 1),              /* Lower non-critical Threshold */
    .pos_thr_hysteresis = 2,                         /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2,                         /* negative going Threshold hysteresis value */
    .reserved1 = 0x00,                               /* reserved */
    .reserved2 = 0x00,                               /* reserved */
    .OEM = 0x00,                                     /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE("TEMP CLK SWITCH"), /* 8 bit ASCII, number of bytes */
    .IDstring = "TEMP CLK SWITCH"                    /*  sensor string */
};

const SDR_type_01h_t SDR_LM75_DCDC = {
    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00,   /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00,  /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1,                          /* entity id: AMC Module */
    .entityinstance = 0x00,                    /* entity instance -> SDR_Init */
    .sensorinit = 0x7F,                        /* init: event generation + scanning enabled */
    .sensorcap = 0x56,                         /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_TEMPERATURE,     /* sensor type */
    .event_reading_type = 0x01,                /* sensor reading*/
    .assertion_event_mask = 0x7A95,            /* assertion event mask (All upper going-high and lower going-low events) */
    .deassertion_event_mask = 0x7A95,          /* deassertion event mask (All upper going-high and lower going-low events) */
    .readable_threshold_mask = 0x3F,           /* LSB: readable Threshold mask: all thresholds are readable:  */
    .settable_threshold_mask = 0x00,           /* MSB: setable Threshold mask: none of the thresholds are setable: */
    .sensor_units_1 = 0x00,                    /* sensor units 1 :*/
    .sensor_units_2 = 0x01,                    /* sensor units 2 :*/
    .sensor_units_3 = 0x00,                    /* sensor units 3 :*/
    .linearization = 0x00,                     /* Linearization */
    .M = 5,                                    /* M */
    .M_tol = 0x00,                             /* M - Tolerance */
    .B = 0x00,                                 /* B */
    .B_accuracy = 0x00,                        /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00,                /* Sensor direction */
    .Rexp_Bexp = 0xF0,                         /* R-Exp , B-Exp */
    .analog_flags = 0x03,                      /* Analogue characteristics flags */
    .nominal_reading = (20 << 1),              /* Nominal reading */
    .normal_max = (50 << 1),                   /* Normal maximum */
    .normal_min = (10 << 1),                   /* Normal minimum */
    .sensor_max_reading = 0xFF,                /* Sensor Maximum reading */
    .sensor_min_reading = 0x00,                /* Sensor Minimum reading */
    .upper_nonrecover_thr = (80 << 1),         /* Upper non-recoverable Threshold */
    .upper_critical_thr = (75 << 1),           /* Upper critical Threshold */
    .upper_noncritical_thr = (65 << 1),        /* Upper non critical Threshold */
    .lower_nonrecover_thr = (0 << 1),          /* Lower non-recoverable Threshold */
    .lower_critical_thr = (5 << 1),            /* Lower critical Threshold */
    .lower_noncritical_thr = (10 << 1),        /* Lower non-critical Threshold */
    .pos_thr_hysteresis = 2,                   /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2,                   /* negative going Threshold hysteresis value */
    .reserved1 = 0x00,                         /* reserved */
    .reserved2 = 0x00,                         /* reserved */
    .OEM = 0x00,                               /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE("TEMP DCDC"), /* 8 bit ASCII, number of bytes */
    .IDstring = "TEMP DCDC"                    /*  sensor string */
};

const SDR_type_01h_t SDR_LM75_RAM = {
    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00,   /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00,  /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1,                         /* entity id: AMC Module */
    .entityinstance = 0x00,                   /* entity instance -> SDR_Init */
    .sensorinit = 0x7F,                       /* init: event generation + scanning enabled */
    .sensorcap = 0x56,                        /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_TEMPERATURE,    /* sensor type */
    .event_reading_type = 0x01,               /* sensor reading*/
    .assertion_event_mask = 0x7A95,           /* assertion event mask (All upper going-high and lower going-low events) */
    .deassertion_event_mask = 0x7A95,         /* deassertion event mask (All upper going-high and lower going-low events) */
    .readable_threshold_mask = 0x3F,          /* LSB: readable Threshold mask: all thresholds are readable:  */
    .settable_threshold_mask = 0x00,          /* MSB: setable Threshold mask: none of the thresholds are setable: */
    .sensor_units_1 = 0x00,                   /* sensor units 1 :*/
    .sensor_units_2 = 0x01,                   /* sensor units 2 :*/
    .sensor_units_3 = 0x00,                   /* sensor units 3 :*/
    .linearization = 0x00,                    /* Linearization */
    .M = 5,                                   /* M */
    .M_tol = 0x00,                            /* M - Tolerance */
    .B = 0x00,                                /* B */
    .B_accuracy = 0x00,                       /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00,               /* Sensor direction */
    .Rexp_Bexp = 0xF0,                        /* R-Exp , B-Exp */
    .analog_flags = 0x03,                     /* Analogue characteristics flags */
    .nominal_reading = (20 << 1),             /* Nominal reading */
    .normal_max = (50 << 1),                  /* Normal maximum */
    .normal_min = (10 << 1),                  /* Normal minimum */
    .sensor_max_reading = 0xFF,               /* Sensor Maximum reading */
    .sensor_min_reading = 0x00,               /* Sensor Minimum reading */
    .upper_nonrecover_thr = (80 << 1),        /* Upper non-recoverable Threshold */
    .upper_critical_thr = (75 << 1),          /* Upper critical Threshold */
    .upper_noncritical_thr = (65 << 1),       /* Upper non critical Threshold */
    .lower_nonrecover_thr = (0 << 1),         /* Lower non-recoverable Threshold */
    .lower_critical_thr = (5 << 1),           /* Lower critical Threshold */
    .lower_noncritical_thr = (10 << 1),       /* Lower non-critical Threshold */
    .pos_thr_hysteresis = 2,                  /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2,                  /* negative going Threshold hysteresis value */
    .reserved1 = 0x00,                        /* reserved */
    .reserved2 = 0x00,                        /* reserved */
    .OEM = 0x00,                              /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE("TEMP RAM"), /* 8 bit ASCII, number of bytes */
    .IDstring = "TEMP RAM"                    /* sensor string */
};
#endif

#ifdef MODULE_MAX6642
const SDR_type_01h_t SDR_MAX6642_FPGA = {
    .hdr.recID_LSB = 0x00, /* Filled by sdr_insert_entry() */
    .hdr.recID_MSB = 0x00,
    .hdr.SDRversion = 0x51,
    .hdr.rectype = TYPE_01,
    .hdr.reclength = sizeof(SDR_type_01h_t) - sizeof(SDR_entry_hdr_t),

    .ownerID = 0x00,   /* i2c address, -> SDR_Init */
    .ownerLUN = 0x00,  /* sensor owner LUN */
    .sensornum = 0x00, /* Filled by sdr_insert_entry() */

    /* record body bytes */
    .entityID = 0xC1,                          /* entity id: AMC Module */
    .entityinstance = 0x00,                    /* entity instance -> SDR_Init */
    .sensorinit = 0x7F,                        /* init: event generation + scanning enabled */
    .sensorcap = 0x56,                         /* capabilities: auto re-arm,*/
    .sensortype = SENSOR_TYPE_TEMPERATURE,     /* sensor type */
    .event_reading_type = 0x01,                /* sensor reading*/
    .assertion_event_mask = 0x7A95,            /* assertion event mask (All upper going-high and lower going-low events) */
    .deassertion_event_mask = 0x7A95,          /* deassertion event mask (All upper going-high and lower going-low events) */
    .readable_threshold_mask = 0x3F,           /* LSB: readable Threshold mask: all thresholds are readable:  */
    .settable_threshold_mask = 0x0,            /* MSB: setable Threshold mask: none of the thresholds are setable: */
    .sensor_units_1 = 0x00,                    /* sensor units 1 :*/
    .sensor_units_2 = 0x01,                    /* sensor units 2 :*/
    .sensor_units_3 = 0x00,                    /* sensor units 3 :*/
    .linearization = 0x00,                     /* Linearization */
    .M = 1,                                    /* M */
    .M_tol = 0x00,                             /* M - Tolerance */
    .B = 0x00,                                 /* B */
    .B_accuracy = 0x00,                        /* B - Accuracy */
    .acc_exp_sensor_dir = 0x00,                /* Sensor direction */
    .Rexp_Bexp = 0x00,                         /* R-Exp , B-Exp */
    .analog_flags = 0x03,                      /* Analogue characteristics flags */
    .nominal_reading = (20 << 1),              /* Nominal reading */
    .normal_max = (50 << 1),                   /* Normal maximum */
    .normal_min = (10 << 1),                   /* Normal minimum */
    .sensor_max_reading = 0xFF,                /* Sensor Maximum reading */
    .sensor_min_reading = 0x00,                /* Sensor Minimum reading */
    .upper_nonrecover_thr = (80 << 1),         /* Upper non-recoverable Threshold */
    .upper_critical_thr = (75 << 1),           /* Upper critical Threshold */
    .upper_noncritical_thr = (65 << 1),        /* Upper non critical Threshold */
    .lower_nonrecover_thr = (0 << 1),          /* Lower non-recoverable Threshold */
    .lower_critical_thr = (5 << 1),            /* Lower critical Threshold */
    .lower_noncritical_thr = (10 << 1),        /* Lower non-critical Threshold */
    .pos_thr_hysteresis = 2,                   /* positive going Threshold hysteresis value */
    .neg_thr_hysteresis = 2,                   /* negative going Threshold hysteresis value */
    .reserved1 = 0x00,                         /* reserved */
    .reserved2 = 0x00,                         /* reserved */
    .OEM = 0x00,                               /* OEM reserved */
    .IDtypelen = 0xc0 | STR_SIZE("TEMP FPGA"), /* 8 bit ASCII, number of bytes */
    .IDstring = "TEMP FPGA"                    /*  sensor string */
};
#endif

void amc_sdr_init(void)
{
  /* Hotswap Sensor */
  sdr_insert_entry(TYPE_02, (void *)&SDR_HOTSWAP_AMC, &vTaskHotSwap_Handle, 0, 0);

  /* UCD90320 sensors*/
#ifdef MODULE_UCD90XXX
  /* AMC & RTM Voltage/Current/Temperature */
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_12V, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC0_VADJ, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC1_VADJ, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_FPGA_1V8, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_FPGA_0V85, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_FPGA_0V9A, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_FPGA_1V2A, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_DDR_1V2T, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_DDR_1V2B, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_IO_3V3, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);

  sdr_insert_entry(TYPE_01, (void *)&SDR_RTM_12V_CURR, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0 );
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC0_12V_CURR, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC0_VADJ_CURR, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC0_P3V3_CURR, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC1_12V_CURR, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC1_VADJ_CURR, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC1_P3V3_CURR, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_FPGA_0V85_CURR, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_FPGA_1V8_CURR, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_FPGA_0V9A_CURR, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_FPGA_1V2A_CURR, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_DDR_1V2T_CURR, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_DDR_1V2B_CURR, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_IO_3V3_CURR, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0);

  sdr_insert_entry(TYPE_01, (void *)&SDR_UCD_TEMP, &vTaskUCD_Handle, 0, CHIP_ID_PMBUS_0 );
#endif

  /* INA3221 sensors */
#ifdef MODULE_INA3221_VOLTAGE

  /* AMC RTM Voltage */
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_12V, &vTaskINA3221_Handle, 0, CHIP_ID_INA_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_RTM_12V, &vTaskINA3221_Handle, 0, CHIP_ID_INA_0);
  sdr_add_settings(CHIP_ID_INA_0, (void *)&INA3221_SETTINGS);

  /* FMC1 Voltage */
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC1_12V, &vTaskINA3221_Handle, FMC1_12V_DEVID, CHIP_ID_INA_1);
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC1_VADJ, &vTaskINA3221_Handle, FMC1_VADJ_DEVID, CHIP_ID_INA_1);
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC1_P3V3, &vTaskINA3221_Handle, FMC1_P3V3_DEVID, CHIP_ID_INA_1);
  sdr_add_settings(CHIP_ID_INA_1, (void *)&INA3221_SETTINGS);

  /* FMC2 Voltage */
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC2_12V, &vTaskINA3221_Handle, FMC2_12V_DEVID, CHIP_ID_INA_2);
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC2_VADJ, &vTaskINA3221_Handle, FMC2_VADJ_DEVID, CHIP_ID_INA_2);
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC2_P3V3, &vTaskINA3221_Handle, FMC2_P3V3_DEVID, CHIP_ID_INA_2);
  sdr_add_settings(CHIP_ID_INA_2, (void *)&INA3221_SETTINGS);

#endif

#ifdef MODULE_INA3221_CURRENT

  /* AMC RTM Current */
  sdr_insert_entry(TYPE_01, (void *)&SDR_AMC_12V_CURR, &vTaskINA3221_Handle, 0, CHIP_ID_INA_0);
  sdr_insert_entry(TYPE_01, (void *)&SDR_RTM_12V_CURR, &vTaskINA3221_Handle, 0, CHIP_ID_INA_0);
  sdr_add_settings(CHIP_ID_INA_0, (void *)&INA3221_SETTINGS);

  /* FMC1 Current */
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC1_12V_CURR, &vTaskINA3221_Handle, FMC1_12V_CURR_DEVID, CHIP_ID_INA_1);
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC1_VADJ_CURR, &vTaskINA3221_Handle, FMC1_VADJ_CURR_DEVID, CHIP_ID_INA_1);
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC1_P3V3_CURR, &vTaskINA3221_Handle, FMC1_P3V3_CURR_DEVID, CHIP_ID_INA_1);
  sdr_add_settings(CHIP_ID_INA_1, (void *)&INA3221_SETTINGS);

  /* FMC2 Current */
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC2_12V_CURR, &vTaskINA3221_Handle, FMC2_12V_CURR_DEVID, CHIP_ID_INA_2);
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC2_VADJ_CURR, &vTaskINA3221_Handle, FMC2_VADJ_CURR_DEVID, CHIP_ID_INA_2);
  sdr_insert_entry(TYPE_01, (void *)&SDR_FMC2_P3V3_CURR, &vTaskINA3221_Handle, FMC2_P3V3_CURR_DEVID, CHIP_ID_INA_2);
  sdr_add_settings(CHIP_ID_INA_2, (void *)&INA3221_SETTINGS);

#endif

}
