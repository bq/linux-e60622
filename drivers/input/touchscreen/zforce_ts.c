/*
 * Copyright (C) 2012 MundoReader S.L.
 * Author: Heiko Stuebner <heiko@sntech.de>
 * 
 * based on
 * 
 * Copyright (C) 2010 Barnes & Noble, Inc.
 * Author: Pieter Truter<ptruter@intrinsyc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define DEBUG 1
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/input/zforce_ts.h>

#define WAIT_TIMEOUT		msecs_to_jiffies(1000)

#define FRAME_START		0xee

/* Offsets of the different parts of the payload the controller sends */
#define PAYLOAD_HEADER		0
#define PAYLOAD_LENGTH		1
#define PAYLOAD_BODY		2

/* Response offsets */
#define RESPONSE_ID		0
#define RESPONSE_DATA		1

/* Commands */
#define COMMAND_DEACTIVATE	0x00
#define COMMAND_INITIALIZE	0x01
#define COMMAND_RESOLUTION	0x02
#define COMMAND_SETCONFIG	0x03
#define COMMAND_DATAREQUEST	0x04
#define COMMAND_SCANFREQ	0x08
#define COMMAND_PULSESTRENG	0x0F
#define COMMAND_LEVEL		0x1C
#define COMMAND_FORCECAL	0X1a
#define COMMAND_STATUS		0X1e

/* Responses the controller sends as a result of
 * command requests
 */
#define RESPONSE_DEACTIVATE	0x00
#define RESPONSE_ACTIVATE	0x01
#define RESPONSE_RESOLUTION	0x02
#define RESPONSE_SETCONFIG	0x03
#define RESPONSE_SCANFREQ	0x08
#define RESPONSE_PULSESTRENG	0x0f
#define RESPONSE_LED_LEVEL	0x1c
#define RESPONSE_ACTIVE_LEDS	0x1d
#define RESPONSE_STATUS		0X1e


/* Notifications are send by the touch controller without
 * beeing requested by the driver and include for example
 * touch indications
 */
#define NOTIFICATION_TOUCH		0x04
#define NOTIFICATION_BOOTCOMPLETE	0x07
#define NOTIFICATION_OVERRUN		0x25
#define NOTIFICATION_PROXIMITY		0x26
#define NOTIFICATION_INVALID_COMMAND	0xfe



struct touch_info_data_t {
	u16	x;
	u16	y;
	u8	id;
	u8	state;
	u8 prblty;
	u8 valid;
	u8 rsvrd;
	u16 z;
};

#define ZFORCE_REPORT_POINTS 2

// Platform specific
#define ZF_COORDATA_SIZE 7

struct zforce_ts {
	struct i2c_client	*client;
	struct input_dev	*input;
	char			phys[32];

	bool			stopped;
	bool			boot_complete;

	/* Firmware version information */
	u16			version_major;
	u16			version_minor;
	u16			version_build;
	u16			version_rev;

	struct completion	command_done;
	bool			command_active;
	int			command_waiting;
	int			command_result;
	int			irq;
	u8	zf_status_info[64];
	int	err_cnt;
};

static u8  tframe_size =0;
static u8 reported_finger_count = 0;

static int zforce_command(struct zforce_ts *ts, u8 cmd)
{
	struct i2c_client *client = ts->client;
	char buf[3];
	int ret;

	dev_dbg(&client->dev, "%s: 0x%x\n", __FUNCTION__, cmd);

	buf[0] = FRAME_START;
	buf[1] = 1; /* data size, command only */
	buf[2] = cmd;

	ret = i2c_master_send(client, &buf[0], ARRAY_SIZE(buf));
	if (ret < 0) {
		dev_err(&client->dev, "i2c send data request error: %d\n", ret);
		return ret;
	}

	return 0;
}

static int zforce_send_wait(struct zforce_ts *ts, const char *buf, const int len)
{
	struct i2c_client *client = ts->client;
	int ret;

	if (ts->command_active) {
		dev_err(&client->dev, "already waiting for a command\n");
		return -EBUSY;
	}

	ts->command_active = 1;

//FIXME: sanity checks

	dev_dbg(&client->dev, "sending %d bytes for command 0x%x\n", buf[1], buf[2]);

	ts->command_waiting = buf[2];

	ret = i2c_master_send(client, buf, len);
	if (ret < 0) {
		dev_err(&client->dev, "i2c send data request error: %d\n", ret);
		goto unlock;
	}

	dev_dbg(&client->dev, "waiting for result for command 0x%x\n", buf[2]);

	if (wait_for_completion_timeout(&ts->command_done, WAIT_TIMEOUT) == 0) {
		ret = -ETIME;
		goto unlock;
	}

	ret = ts->command_result;

unlock:
	ts->command_active = 0;
	return ret;
}

static int zforce_command_wait(struct zforce_ts *ts, u8 cmd)
{
	struct i2c_client *client = ts->client;
	char buf[3];
	int ret;

	dev_dbg(&client->dev, "%s: 0x%x\n", __FUNCTION__, cmd);

	buf[0] = FRAME_START;
	buf[1] = 1; /* data size, command only */
	buf[2] = cmd;

	ret = zforce_send_wait(ts, &buf[0], ARRAY_SIZE(buf));
	if (ret < 0) {
		dev_err(&client->dev, "i2c send data request error: %d\n", ret);
		return ret;
	}

	return 0;
}

static int zforce_resolution(struct zforce_ts *ts, u16 x, u16 y)
{
	struct i2c_client *client = ts->client;
	char buf[7] = { FRAME_START, 5, COMMAND_RESOLUTION,
			(x & 0xff), ((x >> 8) & 0xff),
			(y & 0xff), ((y >> 8) & 0xff) };

	dev_dbg(&client->dev, "set resolution to (%d,%d)\n", x, y);

	return zforce_send_wait(ts, &buf[0], ARRAY_SIZE(buf));
}

static int zforce_scan_frequency(struct zforce_ts *ts, u16 idle, u16 finger, u16 stylus)
{
	struct i2c_client *client = ts->client;
	char buf[9] = { FRAME_START, 7, COMMAND_SCANFREQ,
			(idle & 0xff), ((idle >> 8) & 0xff),
			(finger & 0xff), ((finger >> 8) & 0xff),
			(stylus & 0xff), ((stylus >> 8) & 0xff) };

	dev_dbg(&client->dev, "set scan frequency to (idle: %d, finger: %d, stylus: %d)\n",
		idle, finger, stylus);

	return zforce_send_wait(ts, &buf[0], ARRAY_SIZE(buf));
}

//////////////////////////////// todo ////////////////////////////////

// SETCONFIGURATION Request
// [1:cmd] [2:width] [2:height]
// ############################
static int send_setconfig(struct zforce_ts *ts, u32 setconfig)
{
	struct i2c_msg msg[2];
	u8 request[16];
	int ret;

	dev_info(&ts->client->dev, "%s(%d)\n", __FUNCTION__, setconfig);

	request[0] = COMMAND_SETCONFIG;
	memcpy(&request[1], &setconfig, sizeof(u32));

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 5;
	msg[0].buf = request;

	ret = i2c_transfer(ts->client->adapter, msg, 1);
	if (ret < 0)
	{
		dev_err(&ts->client->dev, "i2c send setconfig error: %d\n", ret);
		return ret;
	}

	if (wait_for_completion_timeout(&ts->command_done, WAIT_TIMEOUT) == 0)
		return -1;

	// I2C opperations was successful
	// Return the results from the controler. (0 == success)
	return ts->command_result;
}

// Fixed Pulse and Strength Request
// ############################
static int send_pulsestreng(struct zforce_ts *ts, u8 strength, u8 time)
{
	struct i2c_msg msg[2];
	u8 request[16];
	int ret;

	dev_info(&ts->client->dev, "%s(%d,%d)\n", __FUNCTION__, strength, time);

	request[0] = COMMAND_PULSESTRENG;
	request[1] = (strength&0x0F) | ( time<<4 ) ;

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = request;

	ret = i2c_transfer(ts->client->adapter, msg, 1);
	if (ret < 0)
	{
		dev_err(&ts->client->dev, "i2c send pulsestreng error: %d\n", ret);
		return ret;
	}

	if (wait_for_completion_timeout(&ts->command_done, WAIT_TIMEOUT) == 0)
		return -1;

	// I2C opperations was successful
	// Return the results from the controler. (0 == success)
	return ts->command_result;
}

// Force Calibration Request
// [1:cmd]
// #######
static int send_forcecal_request(struct zforce_ts *ts)
{
	int ret;

	dev_dbg(&ts->client->dev, "%s()\n", __FUNCTION__);

	ret = i2c_smbus_write_byte(ts->client, COMMAND_FORCECAL);
	if (ret < 0)
	{
		dev_err(&ts->client->dev, "i2c send version request error: %d\n", ret);
		return ret;
	}

	return ts->command_result;
}

// LED LEVEL Request
// [1:cmd]
// #######
static int send_level_request(struct zforce_ts *ts)
{
	int ret;

	dev_dbg(&ts->client->dev, "%s()\n", __FUNCTION__);

	ret = i2c_smbus_write_byte(ts->client, COMMAND_LEVEL);
	if (ret < 0)
	{
		dev_err(&ts->client->dev, "i2c send level request error: %d\n", ret);
		return ret;
	}

	if (wait_for_completion_timeout(&ts->command_done, WAIT_TIMEOUT) == 0)
		return -1;

	return ts->command_result;
}
#define ZF_NUMX 11
#define ZF_NUMY 15
#define ZF_LEDDATA_LEN (2+(ZF_NUMX + ZF_NUMY)*3)
static u8 ledlevel[ZF_LEDDATA_LEN];

// LED Level Payload Results
// [1:x] [1:y] [3*x:xdata] [3*y:ydata]
// #####################################
static int process_level_response(struct zforce_ts *ts, u8* payload)
{
	int i = 0;

	dev_dbg(&ts->client->dev, "%s()\n", __FUNCTION__);

	// save data
	for (i = 0; i < ZF_LEDDATA_LEN; i++ )
	{
		ledlevel[i] = payload[i] ;
	}
	return ZF_LEDDATA_LEN;
}


#define STATE_DOWN 0
#define STATE_MOVE 1
#define STATE_UP   2

struct touch_info_data_t tinfo[ZFORCE_REPORT_POINTS];
//
// Clear touch info control block
//
void touchdata_clear(void)
{
	u8 i;
	for( i=0; i< ZFORCE_REPORT_POINTS; i++ )
	{
		tinfo[i].x		= 0;
		tinfo[i].y		= 0;
		tinfo[i].id		= 0;
		tinfo[i].state	= 0;
		tinfo[i].valid	= 0;
		tinfo[i].rsvrd	= 0;
		tinfo[i].prblty	= 0;
		tinfo[i].z		= 0;
	}
}

//
// return the number of touches
// return error otherwise
static u32 framecounter = 0;
int touchdata_collect( u8* payload )
{
	u8 i;
	reported_finger_count = payload[0];
	framecounter++;
	if( reported_finger_count > ZFORCE_REPORT_POINTS )
	{
		pr_err("Detected (%d) more fingers the max(%d) number supported\n",
				reported_finger_count, ZFORCE_REPORT_POINTS );
		return -EINVAL ;
	}

	for( i=0; i< reported_finger_count; i++ )
	{
		tinfo[i].x	= (u16)((payload[2+i*ZF_COORDATA_SIZE]<<8)|
					payload[1+i*ZF_COORDATA_SIZE]);
		tinfo[i].y	= (u16)((payload[4+i*ZF_COORDATA_SIZE]<<8)|
					payload[3+i*ZF_COORDATA_SIZE]);
		tinfo[i].id	= (u8)((payload[5+i*ZF_COORDATA_SIZE]&0x3C)>>2);
		tinfo[i].state	= (u8)((payload[5+i*ZF_COORDATA_SIZE]&0xC0)>>6);
		tinfo[i].rsvrd	= (u8)( payload[6+i*ZF_COORDATA_SIZE] );
		tinfo[i].prblty	= (u8)( payload[7+i*ZF_COORDATA_SIZE] );
		tinfo[i].valid	= 1;
		tinfo[i].z = reported_finger_count == 0 ? 0 : 20;
	}
	return reported_finger_count;
}

//
// display touch data buffer
//
#ifdef ZF_USE_DEBUG
void touchdata_show(struct zforce_ts *ts)
{
	u8 i;

	if (ts->version_major > 1)
	{
		int i;
		printk("NumFingers=%02d\n", reported_finger_count);
		for(i=0; i<reported_finger_count; i++)
		{
			printk("[%05d](%03d, %03d, %02d) DMU%d ID%d R%02X P%02X V%02d\n",
				framecounter,
				tinfo[i].x,
				tinfo[i].y,
				tinfo[i].z,
				tinfo[i].state,
				tinfo[i].id,
				tinfo[i].rsvrd,
				tinfo[i].prblty,
				tinfo[i].valid );
		}
		printk("\n");
	}
}
#endif

// Fix Pulse Strength  Payload Results
// [1:x] [1:y] [3*x:xdata] [3*y:ydata]
// #####################################
#define ZF_FIXSP_BUFF_SIZE (ZF_NUMX*2+ZF_NUMY*2+2)
static u8 fixps_data[ZF_FIXSP_BUFF_SIZE];
static int process_pulsestreng_response(struct zforce_ts *ts, u8* payload)
{
	int i = 0;
	int numx = -1, numy = -1;
	int datasize;

	dev_dbg(&ts->client->dev, "%s()\n", __FUNCTION__);
	numx = payload[0];
	numy = payload[1];
	datasize = (numx+numy+2);
	if( datasize != ZF_FIXSP_BUFF_SIZE )
	{
		dev_err(&ts->client->dev, "fixps buffer mismatch.(E%d, G%d)(TC%d).\n", ZF_FIXSP_BUFF_SIZE, datasize, ++(ts->err_cnt) );
	}
	if( datasize > ZF_FIXSP_BUFF_SIZE )
	{
		dev_err(&ts->client->dev, "fixps buff overflow:(E%d, G%d)(TC%d).\n", ZF_FIXSP_BUFF_SIZE, datasize, ++(ts->err_cnt) );
		// Clamp datasize to prevent buffer overflow
		datasize = ZF_FIXSP_BUFF_SIZE;
		
	}
	// Save fix pulse strength data
	for(i=0; i<datasize; i++)
	{
		fixps_data[i]=payload[i];
	}
	return ZF_FIXSP_BUFF_SIZE;
}

// Touch Payload Results
// [1:count] [2:x] [2:y] [1:state]
// ###############################
static int process_touch_event(struct zforce_ts *ts, u8* payload)
{
	u16 x,y;
	u8  status;
	int count;
	u8 id = 0;
	u8 state = 0;
	int size = 0;

	// Request the next event ASAP.
	if (ts->version_major == 1)
	{
		size = 5;
	}
	else
	{
		size = 7;
	}

#ifdef ZF_USE_DEBUG
	// =-=-=-=-=-=-=-=-=-=
	//  Get touch data
	// =-=-=-=-=-=-=-=-=-=
	retval = touchdata_collect( payload ) ;
	if( retval < 0 )
		return retval;

	touchdata_show();
#endif

	count = payload[0];
	#define ZF_COORDATA_SIZE 7
	#ifdef ZF_USE_DEBUG
	if (ts->version_major > 1)
	{
		int i;
		printk("NumFingers=%02d\n", count);
		for(i=0; i<count; i++)
			printk("(%03d, %03d) %d %d %02X %02X ",
				(int)((payload[2+i*ZF_COORDATA_SIZE]<<8)|payload[1+i*ZF_COORDATA_SIZE]),
				(int)((payload[4+i*ZF_COORDATA_SIZE]<<8)|payload[3+i*ZF_COORDATA_SIZE]),
				(int)((payload[5+i*ZF_COORDATA_SIZE]&0xC0)>>6),
				(int)((payload[5+i*ZF_COORDATA_SIZE]&0x3C)>>2),
				payload[6+i*ZF_COORDATA_SIZE],
				payload[7+i*ZF_COORDATA_SIZE] );
		printk("\n");
	}
	#endif
	if (count != 1)
	{
		dev_dbg(&ts->client->dev, "Invalid number of coordinates: %d\n", count);
	}
	memcpy(&x, &payload[1], sizeof(u16));
	memcpy(&y, &payload[3], sizeof(u16));
	status = payload[5];

	if (ts->version_major == 1)
	{
		state = status & 0x03;
		id = 1;
	}
	else
	{
		state = (status & 0xC0) >> 6;
		id =    (status & 0x3C) >> 2;
	}

	//x = 600 - x;
	if (ts->version_major == 1)
		y = 800 - y;

	switch (state) {
	/* fall-through, input subsystem handles an unchanged BTN_TOUCH
	 * property itself
	 */
	case STATE_MOVE:
		dev_dbg(&ts->client->dev, "%d move(%d,%d)\n", id, x, y);
	case STATE_DOWN:
		dev_dbg(&ts->client->dev, "%d down(%d,%d)\n", id, x, y);
		input_report_abs(ts->input, ABS_X, x);
		input_report_abs(ts->input, ABS_Y, y);
		input_report_abs(ts->input, ABS_PRESSURE, 1024); /* FIXME: not for upstream, but for old tslib versions */
		input_report_key(ts->input, BTN_TOUCH, 1);
		break;
	case STATE_UP:
		dev_dbg(&ts->client->dev, "%d up(%d,%d)\n", id, x, y);
		input_report_abs(ts->input, ABS_X, x);
		input_report_abs(ts->input, ABS_Y, y);
		input_report_abs(ts->input, ABS_PRESSURE, 0); /* FIXME: not for upstream, but for old tslib versions */
		input_report_key(ts->input, BTN_TOUCH, 0);
		break;
	default:
		dev_err(&ts->client->dev, "Invalid state: %d\n", state);
		return (count * size) + 1;
	}
	input_sync(ts->input);
	return (count * size) + 1;
}

static int zforce_read_packet(struct zforce_ts *ts, u8 *buffer)
{
	struct i2c_client *client = ts->client;
	int ret;

	/* read 2 byte header */
	ret = i2c_master_recv(client, buffer, 2);
	if (ret < 0) {
		dev_err(&client->dev, "error reading header: %d\n", ret);
		return ret;
	}

	if (buffer[PAYLOAD_HEADER] != FRAME_START) {
		dev_err(&client->dev, "invalid frame start: %d\n", buffer[0]);
		return -EINVAL;
	}

	if (buffer[PAYLOAD_LENGTH] <= 0 || buffer[PAYLOAD_LENGTH] > 255) {
		dev_err(&client->dev, "invalid payload length: %d\n", buffer[PAYLOAD_LENGTH]);
		return -EINVAL;
	}

	/* read payload */
	ret = i2c_master_recv(client, &buffer[PAYLOAD_BODY], buffer[PAYLOAD_LENGTH]);
	if (ret < 0) {
		dev_err(&client->dev, "error reading payload: %d\n", ret);
		return ret;
	}

	dev_dbg(&client->dev, "read %d bytes with response 0x%x\n", buffer[PAYLOAD_LENGTH], buffer[PAYLOAD_BODY]);

	return 0;
}

static void zforce_complete(struct zforce_ts *ts, int cmd, int result)
{
	struct i2c_client *client = ts->client;

	if (ts->command_waiting == cmd) {
		dev_dbg(&client->dev, "completing command %d\n", cmd);
		ts->command_result = result;
		complete(&ts->command_done);
	} else {
		dev_dbg(&client->dev, "not for us command %d\n", cmd);
	}
}

static irqreturn_t zforce_interrupt(int irq, void *dev_id)
{
	struct zforce_ts *ts = dev_id;
	struct i2c_client *client = ts->client;
	const struct zforce_ts_platdata *pdata = client->dev.platform_data;
	int ret, i;
	u8 payload_buffer[512];
	u8 *payload;

dev_err(&client->dev, "intstart, gpio: %d, value: %d\n", pdata->gpio_int, gpio_get_value(pdata->gpio_int));

	while(!gpio_get_value(pdata->gpio_int)) {
dev_err(&client->dev, "inthandler, gpio: %d, value: %d\n", pdata->gpio_int, gpio_get_value(pdata->gpio_int));

		ret = zforce_read_packet(ts, payload_buffer);
		if (ret < 0) {
			dev_err(&client->dev, "could not read packet, ret: %d\n", ret); 
			break;
		}

		payload =  &payload_buffer[PAYLOAD_BODY];

		dev_err(&ts->client->dev, " response frame:  " );
		for (i = 0; i < (payload_buffer[PAYLOAD_LENGTH] + 2); i++ )
			printk( " 0x%02X ", payload_buffer[i] );
		printk( "\n" );

		switch (payload[RESPONSE_ID]) {
		case NOTIFICATION_TOUCH:
			process_touch_event(ts, &payload[RESPONSE_DATA]);
			break;
		case NOTIFICATION_BOOTCOMPLETE:
			ts->boot_complete = payload[RESPONSE_DATA];
			break;
		case RESPONSE_ACTIVATE:
		case RESPONSE_DEACTIVATE:
		case RESPONSE_SETCONFIG:
		case RESPONSE_RESOLUTION:
		case RESPONSE_SCANFREQ:
			zforce_complete(ts, payload[RESPONSE_ID], payload[RESPONSE_DATA]);
			break;
		case RESPONSE_LED_LEVEL:
			process_level_response(ts, &payload[RESPONSE_DATA]);
			zforce_complete(ts, payload[RESPONSE_ID], 0);
/*			ts->command_result = 0;
			complete(&ts->command_done);*/
			break;
		case RESPONSE_PULSESTRENG:
			process_pulsestreng_response(ts, &payload[RESPONSE_DATA]);
			zforce_complete(ts, payload[RESPONSE_ID], 0);
/*			ts->command_result = 0;
			complete(&ts->command_done);*/
			break;
		case RESPONSE_STATUS:
			/* Version Payload Results
			 * [2:major] [2:minor] [2:build] [2:rev]
			 */
			memcpy(&ts->version_major, &payload[RESPONSE_DATA+0], sizeof(u16));
			memcpy(&ts->version_minor, &payload[RESPONSE_DATA+2], sizeof(u16));
			memcpy(&ts->version_build, &payload[RESPONSE_DATA+4], sizeof(u16));
			memcpy(&ts->version_rev,   &payload[RESPONSE_DATA+6], sizeof(u16));
	// Request the next event ASAP.
	if (ts->version_major == 1)
		tframe_size = 5;
	else
		tframe_size = 7;
	dev_info(&ts->client->dev, "Firmware Version %04x:%04x %04x:%04x\n", ts->version_major, ts->version_minor, ts->version_build, ts->version_rev);

			u8 *pyld =  NULL;
	
			pyld = (u8 *)ts->zf_status_info;
			#define ZF_STATUS_SIZE 64
			memcpy(pyld, &payload[RESPONSE_DATA], ZF_STATUS_SIZE);
	
			zforce_complete(ts, payload[RESPONSE_ID], 0);
/*			ts->command_result = 0;
			complete(&ts->command_done);*/
			break;
		case NOTIFICATION_INVALID_COMMAND:
			dev_err(&ts->client->dev, "invalid command: 0x%x (err_cnt: %d)", payload[RESPONSE_DATA], ++(ts->err_cnt) );
			break;
		default:
			dev_err(&ts->client->dev, "unrecognized response id: 0x%x (err_cnt: %d)\n", payload[RESPONSE_ID], ++(ts->err_cnt) );
			dev_err(&ts->client->dev, " response frame:  " );
			for (i = 0; i < (payload_buffer[PAYLOAD_LENGTH] + 2); i++ )
				printk( " 0x%02X ", payload_buffer[i] );
			printk( "\n" );
			break;
		}
	}
dev_err(&client->dev, "intend, gpio: %d, value: %d\n", pdata->gpio_int, gpio_get_value(pdata->gpio_int));

	return IRQ_HANDLED;
}

static int zforce_start(struct zforce_ts *ts)
{
	struct i2c_client *client = ts->client;
	const struct zforce_ts_platdata *pdata = client->dev.platform_data;
	int ret;

	dev_dbg(&client->dev, "starting device\n");

	ts->stopped = false;
	mb();
	enable_irq(client->irq);

	// We are now ready for some events..
	ret = zforce_command_wait(ts, COMMAND_INITIALIZE);
	if (ret) {
		dev_err(&client->dev, "Unable to initialize, %d\n", ret);
		return ret;
	}

	/* Set the touch panel dimensions */
	ret = zforce_resolution(ts, pdata->x_max, pdata->y_max);
	if (ret) {
		dev_err(&client->dev, "Unable to set resolution, %d\n", ret);
		goto error;
	}

	ret = zforce_scan_frequency(ts, 10, 100, 100);
	if (ret) {
		dev_err(&client->dev, "Unable to set scan frequency, %d\n", ret);
		goto error;
	}

/*
	#define ZF_SETCONFIG_DUALTOUCH 0x00000001
	// Set configuration, enable dual touch
	if (send_setconfig(ts, ZF_SETCONFIG_DUALTOUCH))
	{
		dev_err(&client->dev, "Unable to set config\n");
		goto err_free_irq;
	}

*/

	// This will start sending touch events.
	ret = zforce_command(ts, COMMAND_DATAREQUEST);
	if (ret) {
		dev_err(&client->dev, "Unable to request data\n");
		goto error;
	}

	/* Per NN, initial cal. take max. of 200msec.
	 * Allow time to complete this calibration
	 */
	msleep(200);

	return 0;

error:
	zforce_command_wait(ts, COMMAND_DEACTIVATE);
	disable_irq(client->irq);
	ts->stopped = true;
	return ret;
}

static int zforce_stop(struct zforce_ts *ts)
{
	struct i2c_client *client = ts->client;
	const struct zforce_ts_platdata *pdata = client->dev.platform_data;
	int ret;

	dev_dbg(&client->dev, "stopping device\n");
dev_err(&client->dev, "zforce_stop, gpio: %d, value: %d\n", pdata->gpio_int, gpio_get_value(pdata->gpio_int));

	/* deactivates touch sensing and puts the device into sleep */
	ret = zforce_command_wait(ts, COMMAND_DEACTIVATE);
	if (ret != 0) {
		dev_err(&client->dev, "could not deactivate device, %d\n",
			ret);
		return ret;
	}

	disable_irq(client->irq);
	ts->stopped = true;

	return 0;
}

static int zforce_input_open(struct input_dev *dev)
{
	struct zforce_ts *ts = input_get_drvdata(dev);
	int ret;

	ret = zforce_start(ts);
	if (ret)
		return ret;

	return 0;
}

static void zforce_input_close(struct input_dev *dev)
{
	struct zforce_ts *ts = input_get_drvdata(dev);

	zforce_stop(ts);

	return;
}

static int zforce_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct zforce_ts *ts = i2c_get_clientdata(client);
	struct input_dev *input = ts->input;
	int ret = 0;

	mutex_lock(&input->mutex);

	/* when configured as wakeup source, device should always wake system
	 * therefore start device if necessary
	 */
	if (true || device_may_wakeup(&client->dev)) {
		dev_dbg(&client->dev, "suspend while being a wakeup source\n");

		/* need to start device if not open, to be wakeup source */
		if (!input->users) {
			ret = zforce_start(ts);
			if (ret)
				goto unlock;
		}

		enable_irq_wake(client->irq);
	} else if (input->users) {
		ret = zforce_stop(ts);
	}

unlock:
	mutex_unlock(&input->mutex);

	return ret;
}

static int zforce_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct zforce_ts *ts = i2c_get_clientdata(client);
	struct input_dev *input = ts->input;
	int ret = 0;

	mutex_lock(&input->mutex);

	if (true || device_may_wakeup(&client->dev)) {
		dev_dbg(&client->dev, "resume from being a wakeup source\n");

		disable_irq_wake(client->irq);

		/* need to stop device if it was not open on suspend */
		if (!input->users) {
			ret = zforce_stop(ts);
			if (ret)
				goto unlock;
		}

		/* device wakes automatically from SLEEP */
	} else if (input->users) {
		ret = zforce_start(ts);
	}

unlock:
	mutex_unlock(&input->mutex);

	return ret;
}

static SIMPLE_DEV_PM_OPS(zforce_pm_ops, zforce_suspend, zforce_resume);

static int zforce_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	const struct zforce_ts_platdata *pdata = client->dev.platform_data;
	struct zforce_ts *ts;
	struct input_dev *input_dev;
	int ret;

	if (!pdata)
		return -EINVAL;

	ts = kzalloc(sizeof(struct zforce_ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

/* enable once we got rid of the ntx stuff
	ret = gpio_request(pdata->gpio_int, "zforce_ts_int");
	if (ret) {
		dev_err(&client->dev, "request of gpio %d failed, %d\n",
			pdata->gpio_int, ret);
		goto err_gpio_int;
	}
*/

	if (pdata->init_hw)
		pdata->init_hw(client);

	ts->client = client;

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&client->dev));

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "could not allocate input device\n");
		ret = -ENOMEM;
		goto err_input_alloc;
	}

	ts->input = input_dev;

	input_dev->name = "Neonode zForce touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_dev->open = zforce_input_open;
	input_dev->close = zforce_input_close;

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);

	__set_bit(BTN_TOUCH, input_dev->keybit);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X, 0, pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, pdata->y_max, 0, 0);

	/* FIXME: not for upstream, but for old tslib versions */
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 1048, 0, 0);

	/* For multi touch */
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
			     pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
			     pdata->y_max, 0, 0);

/* FIXME: do we get information on touch object dimensions? Then activate the following:
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0,
			     AUO_PIXCIR_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0,
			     AUO_PIXCIR_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION, 0, 1, 0, 0);
*/

	input_set_drvdata(ts->input, ts);
	ts->stopped = true;
	ts->err_cnt = 0;

	init_completion(&ts->command_done);

	/* FIXME: beware the IMX5 does not support EDGE triggers, I think there
	 * was a workaround for this around somewhere. Until then don't limit
	 * the trigger events.
	 * This is uncritical, as the ISR also does check the gpio value itself
	 */
	ret = request_threaded_irq(client->irq, NULL, zforce_interrupt,
				   /*IRQF_TRIGGER_FALLING |*/ IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   input_dev->name, ts);
	if (ret) {
		dev_err(&client->dev, "irq %d request failed\n", client->irq);
		goto err_irq_request;
	}

	i2c_set_clientdata(client, ts);

	/* need to start device to get version information */
	ret = zforce_command_wait(ts, COMMAND_INITIALIZE);
	if (ret) {
		dev_err(&client->dev, "Unable to initialize, %d\n", ret);
		goto err_input_register;
	}

	/* this gets the firmware version among other informations */
	ret = zforce_command_wait(ts, COMMAND_STATUS);
	if (ret < 0) {
		dev_err(&client->dev, "could not get device status\n");
		zforce_stop(ts);
		goto err_input_register;
	}


	/* stop device and put it into sleep until it is opened */
	ret = zforce_stop(ts);
	if (ret < 0)
		goto err_input_register;

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "could not register input device\n");
		goto err_input_register;
	}

	return 0;
err_input_register:
	free_irq(client->irq, ts);
err_irq_request:
	input_free_device(input_dev);
err_input_alloc:
	if (pdata->exit_hw)
		pdata->exit_hw(client);
//	gpio_free(pdata->gpio_int);
err_gpio_int:
	kfree(ts);

	return ret;
}

static int zforce_remove(struct i2c_client *client)
{
	struct zforce_ts *ts = i2c_get_clientdata(client);
	struct zforce_ts_platdata *pdata = client->dev.platform_data;

	free_irq(client->irq, ts);

	input_unregister_device(ts->input);

	if (pdata->exit_hw)
		pdata->exit_hw(client);

//	gpio_free(pdata->gpio_int);

	kfree(ts);

	return 0;
}

static struct i2c_device_id zforce_idtable[] = {
	{ "zforce-ts", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, zforce_idtable);

static struct i2c_driver zforce_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "zforce-ts",
		.pm	= &zforce_pm_ops,
	},
	.probe		= zforce_probe,
	.remove		= __devexit_p(zforce_remove),
	.id_table	= zforce_idtable,
};

static int __init zforce_init(void)
{
	return i2c_add_driver(&zforce_driver);
}

static void __exit zforce_exit(void)
{
	i2c_del_driver(&zforce_driver);
}

module_init(zforce_init);
module_exit(zforce_exit);

MODULE_AUTHOR("Heiko Stuebner <heiko@sntech.de>");
MODULE_DESCRIPTION("zForce TouchScreen Driver");
MODULE_LICENSE("GPL");

