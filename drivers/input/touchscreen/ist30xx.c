/*
 *  Copyright (C) 2010,Imagis Technology Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <mach/gpio.h>

#include <linux/input/ist30xx.h>
#include "ist30xx_update.h"

#if IST30XX_DEBUG
#include "ist30xx_misc.h"
#endif

#if IST30XX_TRACKING_MODE
#include "ist30xx_tracking.h"
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0))
#include <linux/input/mt.h>
#endif

#define MAX_ERR_CNT             (100)
#ifdef SEC_TSP_FACTORY_TEST

#define NODE_TOTAL_NUM 240
struct tsp_cmd tsp_cmds[] = {
	{TSP_CMD("fw_update", fw_update),},
	{TSP_CMD("get_config_ver", get_config_ver),},
	{TSP_CMD("get_threshold", get_threshold),},
	{TSP_CMD("get_fw_ver_bin", get_fw_ver_bin),},
	{TSP_CMD("get_fw_ver_ic", get_fw_ver_ic),},
	{TSP_CMD("get_chip_id", get_chip_id),},	
	{TSP_CMD("get_chip_vendor", get_chip_vendor),},
	{TSP_CMD("get_chip_name", get_chip_name),},
	{TSP_CMD("get_x_num", get_x_num),},
	{TSP_CMD("get_y_num", get_y_num),},
	{TSP_CMD("get_raw_value", get_raw_value),},
	{TSP_CMD("get_diff_value", get_diff_value),},
	{TSP_CMD("get_baseline_value", get_baseline_value),},	
	{TSP_CMD("run_raw_read", run_raw_read),},
	{TSP_CMD("run_diff_read", run_diff_read),},
	{TSP_CMD("run_baseline_read", run_baseline_read),},	
	{TSP_CMD("not_support_cmd", not_support_cmd),},
};

static uint16_t node_raw_value[NODE_TOTAL_NUM];
static s16 node_diff_value[NODE_TOTAL_NUM];
static uint16_t node_baseline_value[NODE_TOTAL_NUM];
#endif
#if IST30XX_USE_KEY
int ist30xx_key_code[] = { 0, KEY_MENU, KEY_BACK };
#endif

#if IST30XX_INTERNAL_BIN
#include "ist30xx_fw.h"
#endif

DEFINE_MUTEX(ist30xx_mutex);
extern struct class *sec_class;
struct device *sec_touchkey;
struct device *sec_touchscreen;
#if IST30XX_DETECT_TA
static int ist30xx_ta_status = -1;
#endif
#if IST30XX_DETECT_TEMPER
static int ist30xx_temp_status = -1;
#endif
#if IST30XX_DETECT_CALLER
static int ist30xx_call_status = -1;
#endif

static bool ist30xx_initialized = 0;
struct ist30xx_data *ts_data;
static struct work_struct work_irq_thread;
static struct delayed_work work_reset_check;
#if IST30XX_INTERNAL_BIN && IST30XX_UPDATE_BY_WORKQUEUE
static struct delayed_work work_fw_update;
#endif

static void clear_input_data(struct ist30xx_data *data);


#if IST30XX_EVENT_MODE
bool get_event_mode = true;

static struct timer_list idle_timer;
static struct timespec t_event, t_current;      // ns
#if IST30XX_DETECT_TEMPER
static struct timespec t_temper;                // ns
#endif
#define EVENT_TIMER_INTERVAL     (HZ / 2)       // 0.5sec

# if IST30XX_NOISE_MODE
#  define IST30XX_IDLE_STATUS     (0x1D4E0000)
#  define IDLE_ALGORITHM_MODE     (1U << 15)
# endif // IST30XX_NOISE_MODE
#endif  // IST30XX_EVENT_MODE

#if IST30XX_DEBUG
extern TSP_INFO ist30xx_tsp_info;
extern TKEY_INFO ist30xx_tkey_info;
#endif
extern int ist30xx_init_factory_sysfs(void);
int ist30xx_dbg_level = IST30XX_DEBUG_LEVEL;
void tsp_printk(int level, const char *fmt, ...)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36))
	struct va_format vaf;
#endif
	va_list args;
	int r;

	if (ist30xx_dbg_level < level)
		return;

	va_start(args, fmt);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36))
	vaf.fmt = fmt;
	vaf.va = &args;

	r = printk("%s %pV", IST30XX_DEBUG_TAG, &vaf);
#else
	printk(IST30XX_DEBUG_LEVEL);
	r = vprintk(fmt, args);
#endif
	va_end(args);
}


void ist30xx_disable_irq(struct ist30xx_data *data)
{
	if (data->irq_enabled) {
		disable_irq(data->client->irq);
		data->irq_enabled = 0;
	}
}

void ist30xx_enable_irq(struct ist30xx_data *data)
{
	if (!data->irq_enabled) {
		enable_irq(data->client->irq);
		msleep(50);
		data->irq_enabled = 1;
	}
}


int ist30xx_max_error_cnt = MAX_ERR_CNT;
int ist30xx_error_cnt = 0;
static void ist30xx_request_reset(void)
{
	ist30xx_error_cnt++;
	if (ist30xx_error_cnt >= ist30xx_max_error_cnt) {
		schedule_delayed_work(&work_reset_check, 0);
		tsp_info("%s()\n", __func__);
		ist30xx_error_cnt = 0;
	}
}


void ist30xx_start(struct ist30xx_data *data)
{
#if IST30XX_DETECT_TA
	if (ist30xx_ta_status > -1) {
		ist30xx_write_cmd(data->client, CMD_SET_TA_MODE, ist30xx_ta_status);
		tsp_info("%s(), ta_mode : %d\n", __func__, ist30xx_ta_status);
	}
#endif
#if IST30XX_DETECT_CALLER
	if (ist30xx_call_status > -1) {
		ist30xx_write_cmd(data->client, CMD_SET_CALL_MODE, ist30xx_call_status);
		tsp_info("%s(), call_mode : %d\n", __func__, ist30xx_call_status);
	}
#endif
#if IST30XX_DETECT_TEMPER
	if (ist30xx_temp_status > -1) {
		ist30xx_write_cmd(data->client, CMD_SET_TEMPER_MODE, ist30xx_temp_status);
		tsp_info("%s(), temper_mode : %d\n", __func__, ist30xx_temp_status);
	}
#endif

	ist30xx_cmd_start_scan(data->client);

#if IST30XX_EVENT_MODE
	if ((data->status.update != 1) && (data->status.calib != 1))
		ktime_get_ts(&t_event);
#endif
}


int ist30xx_get_ver_info(struct ist30xx_data *data)
{
	int ret;

	data->fw.pre_ver = data->fw.ver;
	data->fw.ver = 0;

	ret = ist30xx_read_cmd(data->client, CMD_GET_CHIP_ID, &data->chip_id);
	if (ret)
		return ret;

	ret = ist30xx_read_cmd(data->client, CMD_GET_FW_VER, &data->fw.ver);
	if (ret)
		return ret;

	ret = ist30xx_read_cmd(data->client, CMD_GET_PARAM_VER, &data->param_ver);
	if (ret)
		return ret;

	ret = ist30xx_read_cmd(data->client, CMD_GET_TSP_CHNUM2, &data->xy_channel_num);
	if (ret)
		return ret;
	ret = ist30xx_read_cmd(data->client, CMD_GET_TSP_PANNEL_TYPE, &data->tsp_type);
	if (ret)
		return ret;
	tsp_info("Chip ID : %x F/W: %x Param: %x\n",
		 data->chip_id, data->fw.ver, data->param_ver);

	if ((data->chip_id != IST30XX_CHIP_ID) &&
	    (data->chip_id != IST30XXA_CHIP_ID))
		return -EPERM;

	return 0;
}


int ist30xx_init_touch_driver(struct ist30xx_data *data)
{
	int ret = 0;

	mutex_lock(&ist30xx_mutex);
	ist30xx_disable_irq(data);

	ret = ist30xx_cmd_run_device(data->client);
	if (ret)
		goto init_touch_end;

	ret = ist30xx_get_ver_info(data);
	if (ret)
		goto init_touch_end;

init_touch_end:
	ist30xx_start(data);

	ist30xx_enable_irq(data);
	mutex_unlock(&ist30xx_mutex);

	return ret;
}


#if IST30XX_DEBUG
void ist30xx_print_info(void)
{
	TSP_INFO *tsp = &ist30xx_tsp_info;
	TKEY_INFO *tkey = &ist30xx_tkey_info;

	tsp_debug("tkey enable: %d, key num: %d\n", tkey->enable, tkey->key_num);
	tsp_debug("tkey axis_ch: %d, tx: %d\n", tkey->axis_chnum, tkey->tx_line);
	tsp_debug("tkey ch_num[0]: %d, [1]: %d, [2]: %d, [3]: %d, [4]: %d\n",
		  tkey->ch_num[0], tkey->ch_num[1], tkey->ch_num[2],
		  tkey->ch_num[3], tkey->ch_num[4]);
	tsp_debug("tscn internal x,y num: %d, %d\n", tsp->intl.x, tsp->intl.y);
	tsp_debug("tscn module x,y num: %d, %d\n", tsp->mod.x, tsp->mod.y);
	tsp_debug("tscn dir.txch_y: %d, swap: %d\n", tsp->dir.txch_y, tsp->dir.swap_xy);
	tsp_debug("tscn dir.flip_x, y: %d, %d\n", tsp->dir.flip_x, tsp->dir.flip_y);
}
#endif

#define CALIB_MSG_MASK          (0xF0000FFF)
#define CALIB_MSG_VALID         (0x80000CAB)
int ist30xx_get_info(struct ist30xx_data *data)
{
	int ret;
	u32 calib_msg;

	ist30xx_tsp_info.finger_num = IST30XX_MAX_MT_FINGERS;

	mutex_lock(&ist30xx_mutex);
	ist30xx_disable_irq(data);

#if !(IST30XX_INTERNAL_BIN)
	ret = ist30xx_write_cmd(data->client, CMD_RUN_DEVICE, 0);
	if (ret)
		goto get_info_end;
	msleep(10);

	ret = ist30xx_get_ver_info(data);
	if (ret)
		goto get_info_end;
#endif  // !(IST30XX_INTERNAL_BIN)

#if IST30XX_DEBUG
# if IST30XX_INTERNAL_BIN
	ist30xx_get_tsp_info();
	ist30xx_get_tkey_info();
# else
	ret = ist30xx_tsp_update_info();
	if (ret)
		goto get_info_end;

	ret = ist30xx_tkey_update_info();
	if (ret)
		goto get_info_end;
# endif

	ist30xx_print_info();
#endif  // IST30XX_DEBUG

	ret = ist30xx_read_cmd(ts_data->client, CMD_GET_CALIB_RESULT, &calib_msg);
	if (ret == 0) {
		tsp_info("calib status: 0x%08x\n", calib_msg);
		if ((calib_msg & CALIB_MSG_MASK) != CALIB_MSG_VALID ||
		    CALIB_TO_STATUS(calib_msg) > 0) {
			ist30xx_calibrate(IST30XX_FW_UPDATE_RETRY);

			ist30xx_cmd_run_device(data->client);
		}
	}

	ist30xx_start(ts_data);

#if IST30XX_EVENT_MODE
	ktime_get_ts(&t_event);
#endif

	data->status.calib = 0;

#if !(IST30XX_INTERNAL_BIN)
get_info_end:
#endif
	if (ret == 0)
		ist30xx_enable_irq(data);
	mutex_unlock(&ist30xx_mutex);

	return ret;
}


#define PRESS_MSG_MASK          (0x01)
#define MULTI_MSG_MASK          (0x02)
#define PRESS_MSG_KEY           (0x6)

#define TOUCH_DOWN_MESSAGE      ("down")
#define TOUCH_UP_MESSAGE        ("up  ")
#define TOUCH_MOVE_MESSAGE      ("    ")
bool tsp_touched[IST30XX_MAX_MT_FINGERS] = { 0, };

void print_tsp_event(finger_info *finger)
{
#ifdef ENABLED_TSP_LOG		
	int idx = finger->bit_field.id - 1;
	int press = finger->bit_field.udmg & PRESS_MSG_MASK;

	if (press == PRESS_MSG_MASK) {
		if (tsp_touched[idx] == 0) { // touch down
			tsp_info("%s - %d (%d, %d)\n",
				 TOUCH_DOWN_MESSAGE, finger->bit_field.id,
				 finger->bit_field.x, finger->bit_field.y);
			tsp_touched[idx] = 1;
		} else {                    // touch move
			tsp_debug("%s   %d (%d, %d)\n",
				  TOUCH_MOVE_MESSAGE, finger->bit_field.id,
				  finger->bit_field.x, finger->bit_field.y);
		}
	} else {
		if (tsp_touched[idx] == 1) { // touch up
			tsp_info("%s - %d (%d, %d)\n",
				 TOUCH_UP_MESSAGE, finger->bit_field.id,
				 finger->bit_field.x, finger->bit_field.y);
			tsp_touched[idx] = 0;
		}
	}
#endif
}


static void release_finger(finger_info *finger)
{
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0))
	input_mt_slot(ts_data->input_dev, finger->bit_field.id - 1);
	input_mt_report_slot_state(ts_data->input_dev, MT_TOOL_FINGER, false);
#else
	input_report_abs(ts_data->input_dev, ABS_MT_POSITION_X, finger->bit_field.x);
	input_report_abs(ts_data->input_dev, ABS_MT_POSITION_Y, finger->bit_field.y);
	input_report_abs(ts_data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(ts_data->input_dev, ABS_MT_WIDTH_MAJOR, 0);
	input_mt_sync(ts_data->input_dev);
#endif  // (LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0))

	tsp_info("%s %d(%d, %d)\n", __func__,
		 finger->bit_field.id, finger->bit_field.x, finger->bit_field.y);

	finger->bit_field.udmg &= ~(PRESS_MSG_MASK);
	print_tsp_event(finger);

	finger->bit_field.id = 0;

	input_sync(ts_data->input_dev);
}


#define CANCEL_KEY  (0xff)
#define RELEASE_KEY (0)
static void release_key(finger_info *key, u8 key_status)
{
	int id = key->bit_field.id;

	input_report_key(ts_data->input_dev, ist30xx_key_code[id], key_status);

	tsp_debug("%s key%d, event: %d, status: %d\n", __func__,
		  id, key->bit_field.w, key_status);

	key->bit_field.id = 0;

	input_sync(ts_data->input_dev);
}

static void clear_input_data(struct ist30xx_data *data)
{
	int i;
	finger_info *fingers = (finger_info *)data->prev_fingers;
	finger_info *keys = (finger_info *)data->prev_keys;

	for (i = 0; i < data->num_fingers; i++) {
		if (fingers[i].bit_field.id == 0)
			continue;

		if (fingers[i].bit_field.udmg & PRESS_MSG_MASK)
			release_finger(&fingers[i]);
	}

	for (i = 0; i < data->num_keys; i++) {
		if (keys[i].bit_field.id == 0)
			continue;

		if (keys[i].bit_field.w == PRESS_MSG_KEY)
			release_key(&keys[i], RELEASE_KEY);
	}
}

static int check_report_data(struct ist30xx_data *data, int finger_counts, int key_counts)
{
	int i, j;
	bool valid_id;
	finger_info *fingers = (finger_info *)data->fingers;
	finger_info *prev_fingers = (finger_info *)data->prev_fingers;

	/* current finger info */
	for (i = 0; i < finger_counts; i++) {
		if ((fingers[i].bit_field.id == 0) ||
		    (fingers[i].bit_field.id > ist30xx_tsp_info.finger_num) ||
		    (fingers[i].bit_field.x > IST30XX_MAX_X) ||
		    (fingers[i].bit_field.y > IST30XX_MAX_Y)) {
			tsp_warn("Invalid touch data - %d: %d(%d, %d)\n", i,
				 fingers[i].bit_field.id,
				 fingers[i].bit_field.x,
				 fingers[i].bit_field.y);

			fingers[i].bit_field.id = 0;
			return -EPERM;
		}
	}

	/* previous finger info */
	if (data->num_fingers >= finger_counts) {
		for (i = 0; i < ist30xx_tsp_info.finger_num; i++) { // prev_fingers
			if (prev_fingers[i].bit_field.id != 0 &&
			    (prev_fingers[i].bit_field.udmg & PRESS_MSG_MASK)) {
				valid_id = false;
				for (j = 0; j < ist30xx_tsp_info.finger_num; j++) { // fingers
					if ((prev_fingers[i].bit_field.id) ==
					    (fingers[j].bit_field.id)) {
						valid_id = true;
						break;
					}
				}
				if (valid_id == false)
					release_finger(&prev_fingers[i]);
			}
		}
	}

	return 0;
}

bool finger_on_screen(void)
{
	int i;

	for (i = 0; i < IST30XX_MAX_MT_FINGERS; i++)
		if (tsp_touched[i]) return true;

	return false;
}

int key_press = 0;
int key_id = 0;
u32 ist30xx_key_sensitivity = 0; // for SEC scenario
bool get_zvalue_mode = false;

static void report_input_data(struct ist30xx_data *data, int finger_counts, int key_counts)
{
	int i, press, count;
	finger_info *fingers = (finger_info *)data->fingers;

	memset(data->prev_fingers, 0, sizeof(data->prev_fingers));

	for (i = 0, count = 0; i < finger_counts; i++) {
		press = fingers[i].bit_field.udmg & PRESS_MSG_MASK;

		print_tsp_event(&fingers[i]);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0))
		input_mt_slot(data->input_dev, fingers[i].bit_field.id - 1);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,
					   (press ? true : false));
		if (press) {
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 fingers[i].bit_field.x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 fingers[i].bit_field.y);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 fingers[i].bit_field.w);
		}
#else
		input_report_abs(data->input_dev, ABS_MT_POSITION_X,
				 fingers[i].bit_field.x);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
				 fingers[i].bit_field.y);
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
				 press);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR,
				 fingers[i].bit_field.w);
		input_mt_sync(data->input_dev);
#endif          // (LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0)

		data->prev_fingers[i] = fingers[i];
		count++;
	}

#if IST30XX_USE_KEY
	for (i = finger_counts; i < finger_counts + key_counts; i++) {
		key_id = fingers[i].bit_field.id;
		key_press = (fingers[i].bit_field.w == PRESS_MSG_KEY) ? 1 : 0;
		ist30xx_key_sensitivity = fingers[i].bit_field.y;
#ifdef ENABLED_TSK_LOG	
		tsp_debug("key(%08x) id: %d, press: %d, sensitivity: %d\n",
			  fingers[i].full_field, key_id, key_press, fingers[i].bit_field.y);
#endif
		input_report_key(data->input_dev, ist30xx_key_code[key_id], key_press);

		data->prev_keys[key_id - 1] = fingers[i];

		count++;
	}
#endif  // IST30XX_USE_KEY

	if (count > 0)
		input_sync(data->input_dev);

	data->num_fingers = finger_counts;
	ist30xx_error_cnt = 0;
}

/*
 * CMD : CMD_GET_COORD
 *               [31:30]  [29:26]  [25:16]  [15:10]  [9:0]
 *   Multi(1st)  UDMG     Rsvd.    NumOfKey Rsvd.    NumOfFinger
 *    Single &   UDMG     ID       X        Area     Y
 *   Multi(2nd)
 *
 *   UDMG [31] 0/1 : single/multi
 *   UDMG [30] 0/1 : unpress/press
 */
static void ist30xx_irq_thread(struct work_struct *work)
{
	int i, ret;
	int key_cnt, finger_cnt, read_cnt;
	struct ist30xx_data *data = ts_data;
	u32 msg[IST30XX_MAX_MT_FINGERS];
	bool unknown_idle = false;
	static struct timespec t_intr; // ms

#if IST30XX_TRACKING_MODE
	u32 ms;
#endif

	if (!data->irq_enabled)
		goto irq_end;

	memset(msg, 0, sizeof(msg));

	ret = ist30xx_get_position(data->client, msg, 1);
	if (ret)
		goto irq_err;

	tsp_verb("intr msg: 0x%08x\n", *msg);

	if (msg[0] == 0 || msg[0] == 0xFFFFFFFF)
		goto irq_err;

#if IST30XX_EVENT_MODE
	if ((data->status.update != 1) && (data->status.calib != 1))
		ktime_get_ts(&t_intr);
#endif

#if IST30XX_TRACKING_MODE
	ms = t_intr.tv_sec * 1000 + t_intr.tv_nsec / 1000000;
	ist30xx_put_track(ms, msg[0]);
#endif

#if IST30XX_NOISE_MODE
	if (get_event_mode) {
		if ((msg[0] & 0xFFFF0000) == IST30XX_IDLE_STATUS) {
			t_event = t_intr;

			if (msg[0] & IDLE_ALGORITHM_MODE)
				goto irq_end;

			for (i = 0; i < IST30XX_MAX_MT_FINGERS; i++) {
				if (data->prev_fingers[i].bit_field.id == 0)
					continue;

				if (data->prev_fingers[i].bit_field.udmg & PRESS_MSG_MASK) {
					tsp_warn("prev_fingers: %08x\n",
						 data->prev_fingers[i].full_field);
					release_finger(&data->prev_fingers[i]);
					unknown_idle = true;
				}
			}

			for (i = 0; i < IST30XX_MAX_MT_FINGERS; i++) {
				if (data->prev_keys[i].bit_field.id == 0)
					continue;

				if (data->prev_keys[i].bit_field.w == PRESS_MSG_KEY) {
					tsp_warn("prev_keys: %08x\n",
						 data->prev_keys[i].full_field);
					release_key(&data->prev_keys[i], RELEASE_KEY);
					unknown_idle = true;
				}
			}

			if (unknown_idle) {
				schedule_delayed_work(&work_reset_check, 0);
				tsp_warn("Find unknown pressure\n");
			}

			goto irq_end;
		}
	}
#endif  // IST30XX_NOISE_MODE

	if ((msg[0] & CALIB_MSG_MASK) == CALIB_MSG_VALID) {
		data->status.calib_msg = msg[0];
		tsp_info("calib status: 0x%08x\n", data->status.calib_msg);
		t_event = t_intr;
		goto irq_end;
	}

	for (i = 0; i < IST30XX_MAX_MT_FINGERS; i++)
		data->fingers[i].full_field = 0;

	key_cnt = 0;
	finger_cnt = 1;
	read_cnt = 1;
	data->fingers[0].full_field = msg[0];

	if (data->fingers[0].bit_field.udmg & MULTI_MSG_MASK) {
		key_cnt = data->fingers[0].bit_field.x;
		finger_cnt = data->fingers[0].bit_field.y;
		read_cnt = finger_cnt + key_cnt;

		if (finger_cnt > ist30xx_tsp_info.finger_num ||
		    key_cnt > ist30xx_tkey_info.key_num) {
			tsp_warn("Invalid touch count - finger: %d(%d), key: %d(%d)\n",
				 finger_cnt, ist30xx_tsp_info.finger_num,
				 key_cnt, ist30xx_tkey_info.key_num);
			goto irq_err;
		}

#if I2C_BURST_MODE
		ret = ist30xx_get_position(data->client, msg, read_cnt);
		if (ret)
			goto irq_err;

		for (i = 0; i < read_cnt; i++)
			data->fingers[i].full_field = msg[i];
#else
		for (i = 0; i < read_cnt; i++) {
			ret = ist30xx_get_position(data->client, &msg[i], 1);
			if (ret)
				goto irq_err;

			data->fingers[i].full_field = msg[i];
		}
#endif          // I2C_BURST_MODE

#if IST30XX_TRACKING_MODE
		for (i = 0; i < read_cnt; i++)
			ist30xx_put_track(ms, msg[i]);
#endif
	}

	if (check_report_data(data, finger_cnt, key_cnt))
		goto irq_end;

	if (read_cnt > 0)
		report_input_data(data, finger_cnt, key_cnt);

	t_event = t_intr;

	goto irq_end;

irq_err:
	tsp_err("intr msg[0]: 0x%08x, ret: %d\n", msg[0], ret);
	ist30xx_request_reset();
irq_end:
	enable_irq(data->client->irq);
	return;
}

static irqreturn_t irq_thread_func(int irq, void *ptr)
{
	ts_data = ptr;

	disable_irq_nosync(ts_data->client->irq);
	schedule_work(&work_irq_thread);

	return IRQ_HANDLED;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
#define ist30xx_suspend NULL
#define ist30xx_resume  NULL
static void ist30xx_early_suspend(struct early_suspend *h)
{
	struct ist30xx_data *data = container_of(h, struct ist30xx_data,
						 early_suspend);

	mutex_lock(&ist30xx_mutex);
	ist30xx_disable_irq(data);
	ist30xx_internal_suspend(data);
	clear_input_data(data);
	mutex_unlock(&ist30xx_mutex);
}
static void ist30xx_late_resume(struct early_suspend *h)
{
	struct ist30xx_data *data = container_of(h, struct ist30xx_data,
						 early_suspend);

	mutex_lock(&ist30xx_mutex);
	ist30xx_internal_resume(data);
	ist30xx_start(data);
	ist30xx_enable_irq(data);
	mutex_unlock(&ist30xx_mutex);
}
#else
static int ist30xx_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ist30xx_data *data = i2c_get_clientdata(client);

	return ist30xx_internal_suspend(data);
}
static int ist30xx_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ist30xx_data *data = i2c_get_clientdata(client);

	return ist30xx_internal_resume(data);
}
#endif // CONFIG_HAS_EARLYSUSPEND


void ist30xx_set_ta_mode(bool charging)
{
#if IST30XX_DETECT_TA
	if ((ist30xx_ta_status == -1) || (charging == ist30xx_ta_status))
		return;

	ist30xx_ta_status = charging ? 1 : 0;

	tsp_info("%s(), charging = %d\n", __func__, ist30xx_ta_status);

	schedule_delayed_work(&work_reset_check, 0);
#endif
}
EXPORT_SYMBOL(ist30xx_set_ta_mode);

void ist30xx_set_temp_mode(int mode)
{
	/*
	 * #define NORMAL_TEMPERATURE      (0)
	 * #define LOW_TEMPERATURE         (1)
	 * #define HIGH_TEMPERATURE        (2)
	 */
#if IST30XX_DETECT_TEMPER
	if ((ist30xx_temp_status == -1) || (mode == ist30xx_temp_status))
		return;

	ist30xx_temp_status = mode;

	tsp_info("%s(), temperature = %d\n", __func__, ist30xx_temp_status);

	schedule_delayed_work(&work_reset_check, 0);
#endif
}
EXPORT_SYMBOL(ist30xx_set_temp_mode);

void ist30xx_set_call_mode(int mode)
{
#if IST30XX_DETECT_CALLER
	if ((ist30xx_call_status == -1) || (mode == ist30xx_call_status))
		return;

	ist30xx_call_status = mode;

	tsp_info("%s(), call = %d\n", __func__, ist30xx_call_status);

	schedule_delayed_work(&work_reset_check, 0);
#endif
}
EXPORT_SYMBOL(ist30xx_set_call_mode);

static void reset_work_func(struct work_struct *work)
{
	if ((ts_data == NULL) || (ts_data->client == NULL))
		return;

	tsp_info("Request reset function\n");

	if ((ts_data->status.power == 1) &&
	    (ts_data->status.update != 1) && (ts_data->status.calib != 1)) {
		mutex_lock(&ist30xx_mutex);
		ist30xx_disable_irq(ts_data);

		clear_input_data(ts_data);

		ist30xx_cmd_run_device(ts_data->client);

		ist30xx_start(ts_data);

		ist30xx_enable_irq(ts_data);
		mutex_unlock(&ist30xx_mutex);
	}
}

#if IST30XX_INTERNAL_BIN && IST30XX_UPDATE_BY_WORKQUEUE
static void fw_update_func(struct work_struct *work)
{
	if ((ts_data == NULL) || (ts_data->client == NULL))
		return;

	tsp_info("FW update function\n");

	if (ist30xx_auto_bin_update(ts_data))
		ist30xx_disable_irq(ts_data);
}
#endif // IST30XX_INTERNAL_BIN && IST30XX_UPDATE_BY_WORKQUEUE


#if IST30XX_EVENT_MODE
void timer_handler(unsigned long data)
{
	int event_ms;
	int curr_ms;

#if IST30XX_DETECT_TEMPER
	int temper = 0;
#endif

	if (get_event_mode) {
		if ((ts_data->status.power == 1) && (ts_data->status.update != 1)) {
			ktime_get_ts(&t_current);

			curr_ms = t_current.tv_sec * 1000 + t_current.tv_nsec / 1000000;
			event_ms = t_event.tv_sec * 1000 + t_event.tv_nsec / 1000000;

			tsp_verb("event_ms %d, current: %d\n", event_ms, curr_ms);

			if (ts_data->status.calib == 1) {
				if (curr_ms - event_ms >= 2000) {   // 2second
					ts_data->status.calib = 0;
					tsp_debug("calibration timeout over 3sec\n");
					schedule_delayed_work(&work_reset_check, 0);
					ktime_get_ts(&t_event);
				}
			}
#if IST30XX_NOISE_MODE
			else if (curr_ms - event_ms >= 5000) {  // 5second
				tsp_warn("idle timeout over 5sec\n");
				schedule_delayed_work(&work_reset_check, 0);
			}
#endif                  // IST30XX_NOISE_MODE

#if IST30XX_DETECT_TEMPER
			else if (t_current.tv_sec - t_temper.tv_sec >= 30) {    // 30second
				tsp_verb("Temperature check timer\n");

				temper = 0; // temperature read

				if (ist30xx_temp_status == NORMAL_TEMPERATURE) {
					if (temper < -5)        // -5 celcius
						ist30xx_set_temp_mode(LOW_TEMPERATURE);
					else if (temper > 50)   // 50 celcius
						ist30xx_set_temp_mode(HIGH_TEMPERATURE);
				} else if (ist30xx_temp_status == LOW_TEMPERATURE) {
					if (temper >= 0)        //  0 celcius
						ist30xx_set_temp_mode(NORMAL_TEMPERATURE);
				} else if (ist30xx_temp_status == HIGH_TEMPERATURE) {
					if (temper <= 40)       // 40 celcius
						ist30xx_set_temp_mode(NORMAL_TEMPERATURE);
				}

				t_temper = t_current;
			}
#endif                  // IST30XX_DETECT_TEMPER
		}
	}

	mod_timer(&idle_timer, get_jiffies_64() + EVENT_TIMER_INTERVAL);
}
#endif // IST30XX_EVENT_MODE

#ifdef SEC_TSP_FACTORY_TEST
void set_cmd_result(struct ist30xx_data *info, char *buff, int len)
{
	strncat(info->cmd_result, buff, len);
}

ssize_t show_close_tsp_test(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return snprintf(buf, TSP_BUF_SIZE, "%u\n", 0);
}

void set_default_result(struct ist30xx_data *info)
{
	char delim = ':';

	memset(info->cmd_result, 0x00, ARRAY_SIZE(info->cmd_result));
	memcpy(info->cmd_result, info->cmd, strlen(info->cmd));
	strncat(info->cmd_result, &delim, 1);
}

void not_support_cmd(void *device_data)
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;
	char buff[16] = {0};

	set_default_result(info);
	sprintf(buff, "%s", "NA");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 4;
	dev_info(&info->client->dev, "%s: \"%s(%d)\"\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
	return;
}

void get_fw_ver_bin(void *device_data)
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;

	char buff[16] = {0};

	tsp_info("%s, %d\n", __func__, __LINE__ );

	set_default_result(info);
#if IST30XX_INTERNAL_BIN
	if(info->tsp_type== 1)
		snprintf(buff, sizeof(buff), "IM01%04x", ist30xx_parse_param_ver(PARSE_FLAG_FW, ist30xx_fw));
	else
		snprintf(buff, sizeof(buff), "IM00%04x", ist30xx_parse_param_ver(PARSE_FLAG_FW, ist30xx_fw));
#endif
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

void get_fw_ver_ic(void *device_data)
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;

	char buff[16] = {0};
	
	tsp_info("%s, %d\n", __func__, __LINE__ );

	set_default_result(info);

	ist30xx_init_touch_driver(info);

	if(info->tsp_type== 1)
		snprintf(buff, sizeof(buff), "IM01%04x", info->param_ver);
	else
	snprintf(buff, sizeof(buff), "IM00%04x", info->param_ver);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

void get_config_ver(void *device_data)
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;

	char buff[16] = {0};

	tsp_info("%s, %d\n", __func__, __LINE__ );

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%#02x", info->param_ver);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

void get_threshold(void *device_data)
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;
	char buff[16] = {0};
	int v_threshold=175; //touch_key = 300


	tsp_info("%s, %d\n", __func__, __LINE__ );

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%d", v_threshold);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}


void get_chip_id(void *device_data)
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;

	char buff[16] = {0};

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%#02x", info->chip_id);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

void get_chip_vendor(void *device_data)
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;

	char buff[16] = {0};

	tsp_info("%s, %d\n", __func__, __LINE__ );

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%s", "IMAGIS");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

void get_chip_name(void *device_data)
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;

	char buff[16] = {0};

	tsp_info("%s, %d\n", __func__, __LINE__ );

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%s", "IST3032");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

int check_rx_tx_num(void *device_data)
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;

	char buff[TSP_CMD_STR_LEN] = {0};
	int node;
	u32 x_channel;
	u32 y_channel;	

	x_channel = (info->xy_channel_num >> 16) & 0xFFFF;
	y_channel =	info->xy_channel_num & 0xFFFF;

	if (info->cmd_param[0] < 0 ||
			info->cmd_param[0] >= x_channel  ||
			info->cmd_param[1] < 0 ||
			info->cmd_param[1] >= y_channel) {
		snprintf(buff, sizeof(buff) , "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = 3;

		dev_info(&info->client->dev, "%s: parameter error: %u,%u\n",
				__func__, info->cmd_param[0],
				info->cmd_param[1]);
		node = -1;
		return node;
             }
	node = info->cmd_param[0] * y_channel + info->cmd_param[1];
	dev_info(&info->client->dev, "%s: node = %d\n", __func__,
			node);
	return node;
 }


void get_raw_value(void *device_data)
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;

	char buff[16] = {0};
	unsigned int val;
	int node;

	set_default_result(info);
	node = check_rx_tx_num(info);

	if (node < 0)
		return;
    
	val = node_raw_value[node];
	snprintf(buff, sizeof(buff), "%d", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
			strnlen(buff, sizeof(buff)));
}

void get_diff_value(void *device_data) // item2
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;

	char buff[16] = {0};
	unsigned int val;
	int node;

	set_default_result(info);
	node = check_rx_tx_num(info);

	if (node < 0)
		return;
    
	val = node_diff_value[node];
	snprintf(buff, sizeof(buff), "%d", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
			strnlen(buff, sizeof(buff)));
}

void get_baseline_value(void *device_data)
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;

	char buff[16] = {0};
	unsigned int val;
	int node;

	tsp_info("%s\n", __func__);


	set_default_result(info);
	node = check_rx_tx_num(info);

	if (node < 0)
		return;
    
	val = node_baseline_value[node];
	snprintf(buff, sizeof(buff), "%d", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
			strnlen(buff, sizeof(buff)));
}

void get_x_num(void *device_data)
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;

	char buff[16] = {0};
	u32 x_channel;
	
	set_default_result(info);
    
	x_channel = (info->xy_channel_num >> 16) & 0xFFFF;
	
	tsp_info("%s, x channel=%d\n", __func__ , x_channel);
    
	snprintf(buff, sizeof(buff), "%d", x_channel);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

void get_y_num(void *device_data)
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;

	char buff[16] = {0};
	u32 y_channel;
	
	set_default_result(info);
    
	y_channel = info->xy_channel_num & 0xFFFF;

	tsp_info("%s, y channel=%d\n", __func__ , y_channel);
    
	snprintf(buff, sizeof(buff), "%d", y_channel);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

void run_raw_read(void *device_data)
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;
	char buff[16] = {0};
	int i;
	uint16_t max_value = 0, min_value = 0;	
	u32 x_channel =0, y_channel = 0;
	int ret;
	set_default_result(info);

	x_channel = (info->xy_channel_num >> 16) & 0xFFFF;
	y_channel = info->xy_channel_num & 0xFFFF;

	tsp_info("%s, x_channel=%d, y_channel=%d", __func__, x_channel, y_channel);

	y_channel = y_channel - 1;


    ret = ist30xx_read_tsp_node(node_raw_value, node_baseline_value); // by IMAGIS


	for (i = 0; i < (x_channel * y_channel); i++) {
		if (i == 0) {
   			min_value=max_value=node_raw_value[i];
		} else {
   			max_value = max(max_value, node_raw_value[i]);
   			min_value = min(min_value, node_raw_value[i]);
   		}
   
   		printk(" [%d]%3d", i, node_raw_value[i]);
   	}
   	printk("\n");
    

	snprintf(buff, sizeof(buff), "%d,%d", min_value, max_value);
    
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: \"%s(%d)\"\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
	return;
}

void run_diff_read(void *device_data) //item2
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;
	char buff[16] = {0};

	set_default_result(info);
	sprintf(buff, "%s", "NA");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 4;
	dev_info(&info->client->dev, "%s: \"%s(%d)\"\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
	return;
}

void run_baseline_read(void *device_data) //item2
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;
	char buff[16] = {0};
	u32 x_channel;
	u32 y_channel;
	int i, ret;
	
	set_default_result(info);

	x_channel = (info->xy_channel_num >> 16) & 0xFFFF;
	y_channel = info->xy_channel_num & 0xFFFF;

	y_channel = y_channel - 1;


	ret = ist30xx_read_tsp_node(node_raw_value, node_baseline_value); // by IMAGIS

   	for(i = 0 ; i < (x_channel * y_channel) ; i++)
   	{
   		printk(" [%d]%3d", i, node_baseline_value[i]);
   	}
   	printk("\n");


	snprintf(buff, sizeof(buff), "%d", ret);
    
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: \"%s(%d)\"\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
	return;
}



void fw_update(void *device_data)
{
	struct ist30xx_data *info = (struct ist30xx_data *)device_data;
	char buff[16] = {0};
	
	set_default_result(info);

   	tsp_info("%s \n", __func__);

	ist30xx_reset();
#if IST30XX_INTERNAL_BIN
	if((ist30xx_auto_bin_update(info))!= 0)
		tsp_info("FW update problem!!");
#endif
	info->cmd_state = 2;
	snprintf(buff, sizeof(buff) , "%s", "OK");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

}

ssize_t store_cmd(struct device *dev, struct device_attribute
				  *devattr, const char *buf, size_t count)
{
	struct ist30xx_data *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;

	char *cur, *start, *end;
	char buff[TSP_CMD_STR_LEN] = {0};
	int len, i;
	struct tsp_cmd *tsp_cmd_ptr = NULL;
	char delim = ',';
	bool cmd_found = false;
	int param_cnt = 0;
	int ret;

	if (info->cmd_is_running == true) {
		dev_err(&info->client->dev, "tsp_cmd: other cmd is running.\n");
		goto err_out;
	}


	/* check lock  */
	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = true;
	mutex_unlock(&info->cmd_lock);

	info->cmd_state = 1;

	for (i = 0; i < ARRAY_SIZE(info->cmd_param); i++)
		info->cmd_param[i] = 0;

	len = (int)count;
	if (*(buf + len - 1) == '\n')
		len--;
	memset(info->cmd, 0x00, ARRAY_SIZE(info->cmd));
	memcpy(info->cmd, buf, len);

	cur = strchr(buf, (int)delim);
	if (cur)
		memcpy(buff, buf, cur - buf);
	else
		memcpy(buff, buf, len);

	/* find command */
	list_for_each_entry(tsp_cmd_ptr, &info->cmd_list_head, list) {
		if (!strcmp(buff, tsp_cmd_ptr->cmd_name)) {
			cmd_found = true;
			break;
		}
	}

	/* set not_support_cmd */
	if (!cmd_found) {
		list_for_each_entry(tsp_cmd_ptr, &info->cmd_list_head, list) {
			if (!strcmp("not_support_cmd", tsp_cmd_ptr->cmd_name))
				break;
		}
	}

	/* parsing parameters */
	if (cur && cmd_found) {
		cur++;
		start = cur;
		memset(buff, 0x00, ARRAY_SIZE(buff));
		do {
			if (*cur == delim || cur - buf == len) {
				end = cur;
				memcpy(buff, start, end - start);
				*(buff + strlen(buff)) = '\0';
				ret = kstrtoint(buff, 10,\
						info->cmd_param + param_cnt);
				start = cur + 1;
				memset(buff, 0x00, ARRAY_SIZE(buff));
				param_cnt++;
			}
			cur++;
		} while (cur - buf <= len);
	}

	dev_info(&client->dev, "cmd = %s\n", tsp_cmd_ptr->cmd_name);
	for (i = 0; i < param_cnt; i++)
		dev_info(&client->dev, "cmd param %d= %d\n", i,
							info->cmd_param[i]);

	tsp_cmd_ptr->cmd_func(info);


err_out:
	return count;
}

ssize_t show_cmd_status(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct ist30xx_data *info = dev_get_drvdata(dev);
	char buff[16] = {0};

	dev_info(&info->client->dev, "tsp cmd: status:%d\n",
			info->cmd_state);

	if (info->cmd_state == 0)
		snprintf(buff, sizeof(buff), "WAITING");

	else if (info->cmd_state == 1)
		snprintf(buff, sizeof(buff), "RUNNING");

	else if (info->cmd_state == 2)
		snprintf(buff, sizeof(buff), "OK");

	else if (info->cmd_state == 3)
		snprintf(buff, sizeof(buff), "FAIL");

	else if (info->cmd_state == 4)
		snprintf(buff, sizeof(buff), "NOT_APPLICABLE");

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", buff);
}

ssize_t show_cmd_result(struct device *dev, struct device_attribute
				    *devattr, char *buf)
{
	struct ist30xx_data *info = dev_get_drvdata(dev);

	dev_info(&info->client->dev, "tsp cmd: result: %s\n", info->cmd_result);

	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = false;
	mutex_unlock(&info->cmd_lock);

	info->cmd_state = 0;

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", info->cmd_result);
}


static DEVICE_ATTR(close_tsp_test, S_IRUGO, show_close_tsp_test, NULL);
static DEVICE_ATTR(cmd, S_IWUSR | S_IWGRP, NULL, store_cmd);
static DEVICE_ATTR(cmd_status, S_IRUGO, show_cmd_status, NULL);
static DEVICE_ATTR(cmd_result, S_IRUGO, show_cmd_result, NULL);

static struct attribute *sec_touch_facotry_attributes[] = {
		&dev_attr_close_tsp_test.attr,
		&dev_attr_cmd.attr,
		&dev_attr_cmd_status.attr,
		&dev_attr_cmd_result.attr,
	NULL,
};

static struct attribute_group sec_touch_factory_attr_group = {
	.attrs = sec_touch_facotry_attributes,
};
#endif /* SEC_TSP_FACTORY_TEST */
#if defined(SEC_FAC_TK)
static ssize_t fac_read_key_store(struct device *dev,
				  struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	int cmd = 0;
	u32 val = 0;

	sscanf(buf, "%d", &cmd);

	tsp_info("tkey zvalue cmd(%d)\n", cmd);

	val = (cmd > 0 ? 0x00C80001 : 0);

	if (cmd) {
		ist30xx_disable_irq(ts_data);
		ist30xx_cmd_run_device(ts_data->client);
		ist30xx_enable_irq(ts_data);
	}

	ret = ist30xx_write_cmd(ts_data->client, CMD_ZVALUE_MODE, val);
	if (ret) {
		pr_err("[ TSP ] fail, zvalue mode enter(%d)\n", ret);
		return size;
	}

	get_zvalue_mode = (cmd > 0 ? true : false);


	return size;
}

static ssize_t fac_read_key_show(struct device *		dev,
				 struct device_attribute *	attr,
				 char *				buf)
{
	struct ist30xx_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int ret;

	ret = sprintf(buf, "%d\n", ist30xx_key_sensitivity);
	dev_info(&client->dev, "%s: %d\n", __func__, ist30xx_key_sensitivity);

	ist30xx_key_sensitivity = 0;

	return ret;
}

static ssize_t fac_read_menu_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct ist30xx_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int ret;
	u32 val = 0x00C80001;

	/* Z value mode enter ++*/
	if (get_zvalue_mode == false) {
		ist30xx_disable_irq(ts_data);
		ist30xx_cmd_run_device(ts_data->client);
		ist30xx_enable_irq(ts_data);

		ret = ist30xx_write_cmd(ts_data->client, CMD_ZVALUE_MODE, val);
		if (ret) {
			pr_err("[ TSP ] fail, zvalue mode enter(%d)\n", ret);
			return ret;
		}
		get_zvalue_mode=true;		
	}
	/* Z value mode enter --*/
	
	if(key_press == 1 &&  ist30xx_key_code[key_id] == KEY_MENU )
	{
		tsp_info("menu sensitivity = %d\n", ist30xx_key_sensitivity);
		ret = sprintf(buf, "%d\n", ist30xx_key_sensitivity);
		dev_info(&client->dev, "%s: %d\n", __func__, ist30xx_key_sensitivity);
	}
	
	return ret;
}

static ssize_t fac_read_back_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct ist30xx_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int ret;
	u32 val = 0x00C80001;

	/* Z value mode enter ++*/
	if (get_zvalue_mode == false) {
		ist30xx_disable_irq(ts_data);
		ist30xx_cmd_run_device(ts_data->client);
		ist30xx_enable_irq(ts_data);

		ret = ist30xx_write_cmd(ts_data->client, CMD_ZVALUE_MODE, val);
		if (ret) {
			pr_err("[ TSP ] fail, zvalue mode enter(%d)\n", ret);
			return ret;
		}
			get_zvalue_mode=true;		
	}
	/* Z value mode enter --*/

	if(key_press == 1 &&  ist30xx_key_code[key_id] == KEY_BACK )
	{
		tsp_info("back sensitivity = %d\n", ist30xx_key_sensitivity);
		ret = sprintf(buf, "%d\n", ist30xx_key_sensitivity);
		dev_info(&client->dev, "%s: %d\n", __func__, ist30xx_key_sensitivity);		
	}

	return ret;
}


static ssize_t fac_read_firm_version_phone(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	
#if IST30XX_INTERNAL_BIN
	int ret;
	struct ist30xx_data *data = dev_get_drvdata(dev);
	u32 tsp_fw_version;

	tsp_fw_version= ist30xx_parse_param_ver(PARSE_FLAG_FW, ist30xx_fw);

	if(data->tsp_type== 1)
		ret = sprintf(buf, "IM01%04x",tsp_fw_version );
	else
		ret = sprintf(buf, "IM00%04x",tsp_fw_version );
	return ret;
#else
	return -1;
#endif

}

static ssize_t fac_read_firm_version_panel(struct device *dev,
				      struct device_attribute *attr, char *buf)

{
	struct ist30xx_data *data =  dev_get_drvdata(dev);
	int ret;

	ist30xx_init_touch_driver(data);

	if(data->tsp_type== 1)
		ret = sprintf(buf, "IM01%04x", data->param_ver);
	else
		ret = sprintf(buf, "IM00%04x", data->param_ver);

	return ret;
}

#ifdef PREVENT_GHOST_IN_CALL
static ssize_t change_tsp_mode_in_call(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
//	ist30xx_set_ta_mode(1);
	ist30xx_set_call_mode(1);
	
	tsp_info("Set to TA mode in call\n");		

	return 1;
}

static ssize_t restore_tsp_mode_in_call(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
//	ist30xx_set_ta_mode(0);
	ist30xx_set_call_mode(0);
	tsp_info("Restore from TA mode in call\n");		

	return 1;
}

static DEVICE_ATTR(call_connected, S_IRUGO, change_tsp_mode_in_call, NULL);
static DEVICE_ATTR(call_disconnected, S_IRUGO, restore_tsp_mode_in_call, NULL);
#endif

static DEVICE_ATTR(touchkey, S_IRUGO, fac_read_key_show, fac_read_key_store); // by IMAGIS
static DEVICE_ATTR(touchkey_menu, S_IRUGO, fac_read_menu_show, NULL);
static DEVICE_ATTR(touchkey_back, S_IRUGO, fac_read_back_show, NULL);
static DEVICE_ATTR(tsp_firm_version_phone, S_IRUGO, fac_read_firm_version_phone, NULL);
static DEVICE_ATTR(tsp_firm_version_panel, S_IRUGO, fac_read_firm_version_panel, NULL);
		   

static struct attribute *fac_touchkey_attributes[] = {
	&dev_attr_touchkey.attr,          // by IMAGIS
	&dev_attr_touchkey_menu.attr,
	&dev_attr_touchkey_back.attr,
	NULL,
};

static struct attribute_group fac_touchkey_attr_group = {
	.attrs = fac_touchkey_attributes,
};


static struct attribute *fac_tsp_attributes[] = {
	&dev_attr_tsp_firm_version_phone.attr,
	&dev_attr_tsp_firm_version_panel.attr,
#ifdef PREVENT_GHOST_IN_CALL	
	&dev_attr_call_connected.attr,
	&dev_attr_call_disconnected.attr,	
#endif	
	NULL,
};

static struct attribute_group fac_tsp_attr_group = {
	.attrs = fac_tsp_attributes,
};

#endif
static int __devinit ist30xx_probe(struct i2c_client *		client,
				   const struct i2c_device_id * id)
{
	int ret;
	struct ist30xx_data *data;
	struct input_dev *input_dev;
	int retry = 3;
#ifdef SEC_TSP_FACTORY_TEST
	struct device *fac_dev_ts;
	int i;
#endif
	struct tsp_dev_info *pdata = client->dev.platform_data;
	tsp_info("\n%s(), the i2c addr=0x%x", __func__, client->addr);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		tsp_err("%s(), input_allocate_device failed (%d)\n", __func__, ret);
		goto err_alloc_dev;
	}

	data->num_fingers = IST30XX_MAX_MT_FINGERS;
	data->num_keys = IST30XX_MAX_MT_FINGERS;
	data->irq_enabled = 1;
	data->client = client;
	data->input_dev = input_dev;
	i2c_set_clientdata(client, data);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0))
	input_mt_init_slots(input_dev, IST30XX_MAX_MT_FINGERS);
#endif

	input_dev->name = "sec_touchscreen";//"ist30xx_ts_input";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	set_bit(EV_ABS, input_dev->evbit);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0))
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#endif

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, IST30XX_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, IST30XX_MAX_Y, 0, 0);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0))
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, IST30XX_MAX_W, 0, 0);
#else
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, IST30XX_MAX_Z, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, IST30XX_MAX_W, 0, 0);
#endif

#if IST30XX_USE_KEY
	{
		int i;
		set_bit(EV_KEY, input_dev->evbit);
		set_bit(EV_SYN, input_dev->evbit);
		for (i = 1; i < ARRAY_SIZE(ist30xx_key_code); i++)
			set_bit(ist30xx_key_code[i], input_dev->keybit);
	}
#endif

	input_set_drvdata(input_dev, data);
	ret = input_register_device(input_dev);
	if (ret) {
		input_free_device(input_dev);
		goto err_reg_dev;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = ist30xx_early_suspend;
	data->early_suspend.resume = ist30xx_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	ts_data = data;

	ret = ist30xx_init_system();
	if (ret) {
		dev_err(&client->dev, "chip initialization failed\n");
		goto err_init_drv;
	}

	ret = ist30xx_init_update_sysfs();
	if (ret)
		goto err_init_drv;

#if IST30XX_DEBUG
	ret = ist30xx_init_misc_sysfs();
	if (ret)
		goto err_init_drv;
#endif

#if IST30XX_FACTORY_TEST
	ret = ist30xx_init_factory_sysfs();
	if (ret)
		goto err_init_drv;
#endif

#if IST30XX_TRACKING_MODE
	ret = ist30xx_init_tracking_sysfs();
	if (ret)
		goto err_init_drv;
#endif
	/* configure touchscreen interrupt gpio */
	ret = gpio_request(pdata->gpio, "ist30xx_irq_gpio");
	if (ret) {
		tsp_err("unable to request gpio.(%s)\r\n",input_dev->name);
		goto err_init_drv;
	}

	client->irq = gpio_to_irq(pdata->gpio);
	ret = request_threaded_irq(client->irq, NULL, irq_thread_func,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ist30xx_ts", data);
	if (ret)
		goto err_irq;

	INIT_WORK(&work_irq_thread, ist30xx_irq_thread);

	ist30xx_disable_irq(data);

#if IST30XX_INTERNAL_BIN
# if IST30XX_UPDATE_BY_WORKQUEUE
	INIT_DELAYED_WORK(&work_fw_update, fw_update_func);
	schedule_delayed_work(&work_fw_update, IST30XX_UPDATE_DELAY);
# else
	ret = ist30xx_auto_bin_update(data);
	if(ret!= 0)
	{
		tsp_info("retry == 0, No TSP connected!!");
		goto err_irq;
        }
# endif
#endif  // IST30XX_INTERNAL_BIN

	while (retry--) {
		ret = ist30xx_get_info(data);
		tsp_info("Get info: %s\n", (ret == 0 ? "success" : "fail"));
	    if (!ret)
    	   break;
	}
	tsp_info("retry = %d", retry);	
	if(retry <= 0)
	{
		tsp_info("retry == 0, No TSP connected!!");
		goto err_irq;
	}
	INIT_DELAYED_WORK(&work_reset_check, reset_work_func);

#if IST30XX_DETECT_TA
	ist30xx_ta_status = 0;
#endif
#if IST30XX_DETECT_TEMPER
	ist30xx_temp_status = NORMAL_TEMPERATURE;
#endif
#if IST30XX_DETECT_CALLER
	ist30xx_call_status = 0;
#endif

#if IST30XX_EVENT_MODE
	init_timer(&idle_timer);
	idle_timer.function = timer_handler;
	idle_timer.expires = jiffies_64 + (EVENT_TIMER_INTERVAL);

	mod_timer(&idle_timer, get_jiffies_64() + EVENT_TIMER_INTERVAL);

	ktime_get_ts(&t_event);
#if IST30XX_DETECT_TEMPER
	ktime_get_ts(&t_temper);
#endif
#endif

	ist30xx_initialized = 1;
#ifdef SEC_TSP_FACTORY_TEST
		INIT_LIST_HEAD(&data->cmd_list_head);
		for (i = 0; i < ARRAY_SIZE(tsp_cmds); i++)
			list_add_tail(&tsp_cmds[i].list, &data->cmd_list_head);

		mutex_init(&data->cmd_lock);
		data->cmd_is_running = false;

	fac_dev_ts = device_create(sec_class, NULL, 0, data, "tsp");
	if (IS_ERR(fac_dev_ts))
		dev_err(&client->dev, "Failed to create device for the sysfs\n");

	ret = sysfs_create_group(&fac_dev_ts->kobj,	&sec_touch_factory_attr_group);
	if (ret)
		dev_err(&client->dev, "Failed to create sysfs group\n");
#endif

#if defined(SEC_FAC_TK)
	sec_touchkey = device_create(sec_class, NULL, 0, data, "sec_touchkey");
	if (IS_ERR(sec_touchkey)) {
		dev_err(&client->dev, "Failed to create fac tsp temp dev\n");
	}

	ret = sysfs_create_group(&sec_touchkey->kobj, &fac_touchkey_attr_group);
	if (ret)
		dev_err(&client->dev, "%s: failed to create fac_touchkey_attr_group (%d)\n", __func__, ret);

	sec_touchscreen	= device_create(sec_class, NULL, 0, data, "sec_touchscreen");
	if (IS_ERR(sec_touchscreen)) {
		dev_err(&client->dev, "Failed to create fac tsp temp dev\n");
	}

	ret = sysfs_create_group(&sec_touchscreen->kobj, &fac_tsp_attr_group);
	if (ret)
		dev_err(&client->dev, "%s: failed to create fac_tsp_attr_group (%d)\n", __func__, ret);

#endif

	return 0;

err_irq:
	ist30xx_disable_irq(data);
	free_irq(client->irq, data);
err_init_drv:
#if IST30XX_EVENT_MODE
	get_event_mode = false;
#endif
	tsp_err("Error, ist30xx init driver\n");
	ist30xx_power_off();
	input_unregister_device(input_dev);
	return 0;

err_reg_dev:
err_alloc_dev:
	tsp_err("Error, ist30xx mem free\n");
	kfree(data);
	return 0;
}


static int __devexit ist30xx_remove(struct i2c_client *client)
{
	struct ist30xx_data *data = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif

	free_irq(client->irq, data);
	ist30xx_power_off();

	input_unregister_device(data->input_dev);
	kfree(data);

	return 0;
}


static struct i2c_device_id ist30xx_idtable[] = {
	{ IST30XX_DEV_NAME, 0 },
	{},
};


MODULE_DEVICE_TABLE(i2c, ist30xx_idtable);

#ifdef CONFIG_HAS_EARLYSUSPEND
static const struct dev_pm_ops ist30xx_pm_ops = {
	.suspend	= ist30xx_suspend,
	.resume		= ist30xx_resume,
};
#endif


static struct i2c_driver ist30xx_i2c_driver = {
	.id_table	= ist30xx_idtable,
	.probe		= ist30xx_probe,
	.remove		= __devexit_p(ist30xx_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= IST30XX_DEV_NAME,
#ifdef CONFIG_HAS_EARLYSUSPEND
		.pm	= &ist30xx_pm_ops,
#endif
	},
};


static int __init ist30xx_init(void)
{
	return i2c_add_driver(&ist30xx_i2c_driver);
}


static void __exit ist30xx_exit(void)
{
	i2c_del_driver(&ist30xx_i2c_driver);
}

module_init(ist30xx_init);
module_exit(ist30xx_exit);

MODULE_DESCRIPTION("Imagis IST30XX touch driver");
MODULE_LICENSE("GPL");
