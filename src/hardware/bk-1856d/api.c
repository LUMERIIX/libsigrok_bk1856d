/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2017 LJ <lj@lj>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <config.h>
#include <stdlib.h>
#include <string.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "protocol.h"
#include "protocol_wrappers.h"
#include <glib-2.0/glib.h>
#include <config.h>
#include "protocol.h"
#include <config.h>
#include "protocol.h"


#define BUF_MAX 50
#define SERIALCOMM "9600/8n1/dtr=1/rts=0"
//#define HAVE_LIBSERIAL


static const uint32_t drvopts[] = {
	SR_CONF_LOGIC_ANALYZER,
	SR_CONF_OSCILLOSCOPE,
	SR_CONF_MULTIMETER,
};

static const uint32_t scanopts[] = {
	SR_CONF_CONN,
	SR_CONF_SERIALCOMM,
};

static const uint32_t devopts[] = {
	SR_CONF_MULTIMETER,
	SR_CONF_CONTINUOUS,
	SR_CONF_TIMEBASE | SR_CONF_SET |SR_CONF_GET| SR_CONF_LIST,
	SR_CONF_DEVICE_MODE | SR_CONF_SET | SR_CONF_LIST,
    SR_CONF_LIMIT_SAMPLES | SR_CONF_SET | SR_CONF_GET,
    SR_CONF_SAMPLERATE | SR_CONF_SET | SR_CONF_GET,

};


static struct sr_dev_driver bk_1856_driver_info;

const uint64_t timebases[4][2] = {
    /*miliseconds*/
	{ 10, 1000 },
	{ 100, 1000 },
		/* seconds */
	{ 1, 1 },
	{ 10, 1 },
};

static const uint64_t samplerates[] = {
	SR_HZ(1),
	SR_GHZ(1),
	SR_HZ(1),
    SR_KHZ(1),
};


static GSList *scan(struct sr_dev_driver *di, GSList *options)
{

    struct sr_dev_inst *sdi;
	struct bk_1856d_context *devc;
	struct sr_config *src;
	struct sr_serial_dev_inst *serial;
	GSList *l, *devices;
	int len, cnt;
	const char *serialcomm, *conn;
	char *buf;
	char *req = "D0";
	char test;

	devices = NULL;
	conn = serialcomm = NULL;

	for (l = options; l; l = l->next)
    {
		src = l->data;
		switch (src->key)
		{
		case SR_CONF_CONN:
			conn = g_variant_get_string(src->data, NULL);
			break;
		case SR_CONF_SERIALCOMM:
			serialcomm = g_variant_get_string(src->data, NULL);
			break;
		}
	}
	if (!conn)
		return NULL;

    serial = sr_serial_dev_inst_new(conn, SERIALCOMM);
    strcpy(CONN,conn);

	if (serial_open(serial, 1) != SR_OK)
		return NULL;

	serial_flush(serial);

	buf = g_malloc(BUF_MAX);

	g_usleep(150 * 1000); /* Wait a little to allow serial port to settle. */
	for (cnt = 0; cnt < 7; cnt++)
    {
        sr_dbg("Checking data transfer to device...");
		if(serial_write_blocking(serial, req, strlen(req),
            serial_timeout(serial, strlen(req))) < 0)
        {
			sr_err("Unable to send identification request.");
			return NULL;
		}
		len = BUF_MAX;

		serial_readline(serial, &buf, &len, BK1856D_TIMEOUT_MS);
		if (!len)
			continue;
		buf[BUF_MAX - 1] = '\0';

        sdi = g_malloc0(sizeof(struct sr_dev_inst));
        sdi->status = SR_ST_INACTIVE;
        sdi->vendor = g_strdup("bk-precision");
        sdi->model = g_strdup("1856D");
        sdi->version = g_strdup(buf + 9);
        devc = g_malloc0(sizeof(struct bk_1856d_context));
        sr_sw_limits_init(&devc->limits);
        sdi->conn = serial;
        sdi->priv = devc;
        sr_channel_new(sdi, 0, SR_CHANNEL_ANALOG, TRUE, "P1");
        devices = g_slist_append(devices, sdi);
        break;
    }

	serial_close(serial);

    if (!(devices))
		sr_serial_dev_inst_free(serial);

	return std_scan_complete(di, devices);
}


static int dev_clear(const struct sr_dev_driver *di)
{
	return std_dev_clear(di, NULL);

}

static int config_get(uint32_t key, GVariant **data, const struct sr_dev_inst *sdi,
		const struct sr_channel_group *cg)
{

    int ret, cg_type;
	unsigned int i;
	struct bk_1856d_context *devc;
	const struct scope_config *model;
	struct scope_state *state;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;

	model = devc->model_config;
	state = devc->model_state;


    switch (key)
    {
    case SR_CONF_TIMEBASE:
		*data = g_variant_new("(tt)",
				timebases[state->timebase][0],
				timebases[state->timebase][1]);
				sr_dbg("timebase: %f",data);
		ret = SR_OK;
		break;
    case SR_CONF_SAMPLERATE:
		*data = g_variant_new_double(devc->cur_samplerate);
		break;
    case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->limit_samples);
		break;
    case SR_CONF_CONTINUOUS:
        sr_dbg("\n\n CONTINUOUS SAMPLING \n\n");
        break;
    default:
		ret = SR_ERR_NA;

	return ret;
    }
}


static GVariant *build_tuples(const uint64_t (*array)[][2], unsigned int n)
{
	unsigned int i;
	GVariant *rational[2];
	GVariantBuilder gvb;

	g_variant_builder_init(&gvb, G_VARIANT_TYPE_ARRAY);

	for (i = 0; i < n; i++) {
		rational[0] = g_variant_new_uint64((*array)[i][0]);
		rational[1] = g_variant_new_uint64((*array)[i][1]);

		/* FIXME: Valgrind reports a memory leak here. */
		g_variant_builder_add_value(&gvb, g_variant_new_tuple(rational, 2));
	}

	return g_variant_builder_end(&gvb);
}

static int config_set(uint32_t key, GVariant *data, const struct sr_dev_inst *sdi,
		const struct sr_channel_group *cg)
{
    int ret, cg_type;
	unsigned int i;
    char *tmp;
	struct bk_1856d_context *devc;
	struct scope_state *state;
	uint64_t p, q;
	gboolean update_sample_rate;
    char gatetime[30];
	ret = SR_ERR_NA;

	devc = sdi->priv;

switch (key) {
	case SR_CONF_TIMEBASE:
		g_variant_get(data, "(tt)", &p, &q);

		for (i = 0; i < ARRAY_SIZE(timebases); i++) {
			if (p != timebases[i][0] ||
					q != timebases[i][1])
				continue;
			g_ascii_formatd(gatetime, sizeof(gatetime),
					"%E", (float) p / q);
			update_sample_rate = TRUE;
			if(strcmp(gatetime,"1.000000E+01")==0) {
                send_buf = "G3";
                request_delay = 10000;}
            if(strcmp(gatetime,"1.000000E+00")==0) {
                send_buf = "G2";
                request_delay = 1000; }
            if(strcmp(gatetime,"1.000000E-01")==0) {
                send_buf = "G1";
                request_delay = 100;}
            if(strcmp(gatetime,"1.000000E-02")==0) {
                send_buf = "G0";
                request_delay = 10; }

		}
		break;

    case SR_CONF_CONTINUOUS:
        break;

    case SR_CONF_DEVICE_MODE:
        tmp = g_variant_get_string(data, NULL);
        sr_dbg("Device_Mode:%s",tmp);
             if(strcmp(tmp,"FREQA")==0)
                send_buf = "F0";
            if(strcmp(tmp,"FREQC")==0)
                send_buf = "F2";
            if(strcmp(tmp,"PERIOD")==0)
                send_buf = "F3";
            if(strcmp(tmp,"TOTAL")==0)
                send_buf = "F4";
            if(strcmp(tmp,"RPM")==0)
                send_buf = "F5";

            break;

    case SR_CONF_LIMIT_SAMPLES:
		devc->limit_msec = 0;
		devc->limit_samples = g_variant_get_uint64(data);
		break;

        case SR_CONF_SAMPLERATE:
		devc->cur_samplerate = g_variant_get_uint64(data);
		break;

    default:
		send_buf = "D0";
		break;
        }

	return SR_OK;
}

static int config_list(uint32_t key, GVariant **data, const struct sr_dev_inst *sdi,
		const struct sr_channel_group *cg)
{
    (void)sdi;
	(void)cg;

    GVariant *gvar;
	GVariantBuilder gvb;

    struct bk_1856d_context *devc = NULL;
	const struct scope_config *model = NULL;

	if (key == SR_CONF_SCAN_OPTIONS)
    {
		*data = g_variant_new_fixed_array(G_VARIANT_TYPE_UINT32,
				scanopts, ARRAY_SIZE(scanopts), sizeof(uint32_t));
		return SR_OK;
	}

	/* If sdi is NULL, nothing except SR_CONF_DEVICE_OPTIONS can be provided. */
	if (key == SR_CONF_DEVICE_OPTIONS && !sdi)
    {
		*data = g_variant_new_fixed_array(G_VARIANT_TYPE_UINT32,
				drvopts, ARRAY_SIZE(drvopts), sizeof(uint32_t));
		return SR_OK;
	}

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;
	model = devc->model_config;

	/*
	 * If cg is NULL, only the SR_CONF_DEVICE_OPTIONS that are not
	 * specific to a channel group must be returned.
	 */
	if (!cg)
    {
		switch (key)
		{
		case SR_CONF_DEVICE_OPTIONS:
			*data = g_variant_new_fixed_array(G_VARIANT_TYPE_UINT32,
					devopts, ARRAY_SIZE(devopts), sizeof(uint32_t));
			return SR_OK;
		case SR_CONF_TIMEBASE:
			*data = build_tuples(&timebases, ARRAY_SIZE(timebases));
			return SR_OK;
        case SR_CONF_SCAN_OPTIONS:
            *data = g_variant_new_fixed_array(G_VARIANT_TYPE_UINT32,
				scanopts, ARRAY_SIZE(scanopts), sizeof(uint32_t));
            return SR_OK;
        case SR_CONF_DEVICE_MODE:
            //*data = device_modes;
        case SR_CONF_SAMPLERATE:
			g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
			gvar = g_variant_new_fixed_array(G_VARIANT_TYPE("t"), samplerates,
					ARRAY_SIZE(samplerates), sizeof(uint64_t));
			g_variant_builder_add(&gvb, "{sv}", "samplerate-steps", gvar);
			*data = g_variant_builder_end(&gvb);
			break;
		default:
			return SR_ERR_NA;
		}
	}


    return SR_OK;
}


static int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
    struct bk_1856d_context *devc;
	struct sr_serial_dev_inst *serial;

	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR_DEV_CLOSED;

	devc = sdi->priv;

	sr_sw_limits_acquisition_start(&devc->limits);

    bk_1856d_send_data(sdi);

	/* Poll every 50ms, or whenever some data comes in. */
    serial = sdi->conn;
        serial_source_add(sdi->session, serial, G_IO_IN, 50,
    			bk_1856d_receive_data, (struct sr_dev_inst *)sdi);


    std_session_send_df_header(sdi);

	return SR_OK;
}


static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
    struct sr_serial_dev_inst *serial;
	sr_dbg("Stopping acquisition.");
	std_session_send_df_end(sdi);
	sr_session_source_remove(sdi->session, -1);
	serial_close(serial);

	return SR_OK;
}

static struct sr_dev_driver bk_1856d_driver_info = {
	.name = "bk-1856d",
	.longname = "bk-1856d",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = std_serial_dev_open,
	.dev_close = std_serial_dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(bk_1856d_driver_info);
