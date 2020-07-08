/*
 * main.c
 *
 *  Created on: Jun 13, 2017
 *      Author: chiz
 */

#include <stdio.h>
#include "smartpooltmr.h"
#include "mgos_aws_shadow.h"
#include "mgos_http_server.h"

/* Global variables	*/
char *msg = "This is a message";
struct sys_config *device_cfg;
dpt_system_t system_data;
tmSchedule_t pump_schedule[NUM_WEEKDAYS];
bool gADCConfigured = false;

/* Definitions 	*/
#define MGOS_F_RELOAD_CONFIG MG_F_USER_5


static struct device_settings s_settings = {"ssid", "password"};

/*
 * Handle save routine. This puts the grabs the ssid/password from the
 * web page and calls
 *
 */
static void handle_save(struct mg_connection *nc, struct http_message *hm) {
  struct mgos_config_wifi_sta device_cfg_sta;

  memset(&device_cfg_sta, 0, sizeof(device_cfg_sta));

  // Get form variables and store settings values
  LOG(LL_INFO, ("Getting variables from %s", (&hm->uri)->p));
  LOG(LL_DEBUG, ("mg_get_http_var hm->body->p: %s hm->body.len: %d\n", (&hm->body)->p, hm->body.len));
  mg_get_http_var(&hm->body, "ssid", s_settings.ssid, sizeof(s_settings.ssid));
  mg_get_http_var(&hm->body, "password", s_settings.pswd, sizeof(s_settings.pswd));

  // Send response... trying to reload index page
  mg_http_send_redirect(nc, 302, mg_mk_str("/"), mg_mk_str(NULL));

  /*
  // Using this peice of code didn't cause a crash
  LOG(LL_DEBUG, ("Rebooting now... "));
  mg_send_head(nc, 200, 0, "Connection: close\r\nContent-Type: application/json");
  nc->flags |= (MG_F_SEND_AND_CLOSE | MGOS_F_RELOAD_CONFIG);
   */

  /*
  // Use chunked encoding in order to avoid calculating Content-Length
  mg_printf(nc, "%s", "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n");
  mg_send_http_chunk(nc, "", 0);
  nc->flags |= MG_F_SEND_AND_CLOSE;
  */

  LOG(LL_INFO, ("WiFi Station: ssid = %s; password = %s", s_settings.ssid, s_settings.pswd));
  device_cfg_sta.ssid = s_settings.ssid;
  device_cfg_sta.pass = s_settings.pswd;
  LOG(LL_INFO, ("Device Config WiFi Station: ssid = %s; password = %s", device_cfg_sta.ssid, device_cfg_sta.pass));

  becomeStation(&device_cfg_sta);
}

/*
 * handle_get_cpu_usage. This reports CPU usage, and also provides connection status
 *
 */
static void handle_get_cpu_usage(struct mg_connection *nc) {

	int cpu_usage = 0;
	struct mbuf fb;
	struct json_out fout = JSON_OUT_MBUF(&fb);
	mbuf_init(&fb, 256);

	cpu_usage = ((double)mgos_get_free_heap_size()/(double)mgos_get_heap_size()) * 100.0;
	json_printf(&fout, STATUS_FMT, cpu_usage, mgos_wifi_get_connected_ssid(), mgos_wifi_get_sta_ip());
    mbuf_trim(&fb);

	struct mg_str f = mg_mk_str_n(fb.buf, fb.len);	/* convert to string	*/

	// Use chunked encoding in order to avoid calculating Content-Length
	mg_printf(nc, "%s", "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n");
	//	mg_printf_http_chunk(nc, f.p);
	mg_send_http_chunk(nc, f.p, f.len);
	mg_send_http_chunk(nc, "", 0);
	nc->flags |= MG_F_SEND_AND_CLOSE;

	LOG(LL_INFO, ("%s\n", f.p));

	mbuf_free(&fb);
}

/*
 * SSI handler for SSI events...
 * Don't need this now... May delete
 *
 */
static void handle_ssi_call(struct mg_connection *nc, const char *param) {
  if (strcmp(param, "ssid") == 0) {
    mg_printf_html_escape(nc, "%s", s_settings.ssid);
  }
  else if (strcmp(param, "password") == 0) {
    mg_printf_html_escape(nc, "%s", s_settings.pswd);
  }
}

/*
 * Event Handler for http post events "/save"
 *
 */
static void http_post_ev_handler(struct mg_connection *nc, int ev, void *ev_data, void *user_data) {
  struct http_message *hm = (struct http_message *) ev_data;

  printf("Event: %d, uri: %s\n", ev, hm->uri.p);

  switch (ev) {
    case MG_EV_HTTP_REQUEST:
      if (mg_vcmp(&hm->uri, "/save") == 0) {
        handle_save(nc, hm);
      }
      break;
    case MG_EV_SSI_CALL:
      handle_ssi_call(nc, ev_data);
      break;
    default:
      break;
  }
  (void) user_data;
}


/*
 * Event Handler for http get events
 *
 */
static void http_get_ev_handler(struct mg_connection *nc, int ev, void *ev_data, void *user_data) {
  struct http_message *hm = (struct http_message *) ev_data;

  printf("Event: %d, uri: %s\n", ev, hm->uri.p);

  switch (ev) {
    case MG_EV_HTTP_REQUEST:
      if (mg_vcmp(&hm->uri, "/get_cpu_usage") == 0) {
        handle_get_cpu_usage(nc);
      }
      else {
    	mg_http_send_redirect(nc, 302, mg_mk_str("/"), mg_mk_str(NULL));
      }
      break;
    case MG_EV_SSI_CALL:
      handle_ssi_call(nc, ev_data);
      break;
    default:
      break;
  }
  (void) user_data;
}


/**
 * Become a station connecting to an existing access point.
 */
void becomeStation(struct mgos_config_wifi_sta *device_cfg_sta) {
	struct mgos_config_wifi_sta *wifi_sta;

	/* get current config settings	*/
	device_cfg = get_cfg();
	wifi_sta = &device_cfg->wifi.sta;
	LOG(LL_DEBUG, ("Getting config at address\n"));

	wifi_sta->ssid = device_cfg_sta->ssid;
	wifi_sta->pass = device_cfg_sta->pass;
	wifi_sta->enable = true;
	device_cfg->wifi.sta = *wifi_sta;
	device_cfg->mqtt.enable = true;
	LOG(LL_DEBUG, ("Device config set SSID = %s, Password = %s \n", device_cfg_sta->ssid, device_cfg_sta->pass));

	if (save_cfg(device_cfg, &msg)) {
		LOG(LL_DEBUG, ("SUCCESS: Saved Config. System Restarting... \n"));
		mgos_wifi_setup_sta(&device_cfg->wifi.sta);
//		mgos_system_restart(0);
	}
	else {
		LOG(LL_DEBUG, ("FAIL: %s\n", msg));
	}

	(void)device_cfg_sta;
} // becomeStation


/*
 * report_state. This reports state to the AWS shadow
 *
 */

void report_state(void) {
  struct mbuf fb;
  struct json_out fout = JSON_OUT_MBUF(&fb);
  mbuf_init(&fb, 256);

  json_printf(&fout, JSON_EPOOLTIMER_FMT, system_data.pump_voltage, system_data.pump_current, system_data.pump_power,
		  system_data.box_temperature, system_data.box_pressure, system_data.manOverride, system_data.pumpCmd);

  mbuf_trim(&fb);

//  LOG(LL_INFO, ("== Reporting state: %s", fb.buf));
  mgos_aws_shadow_updatef(0, "{reported:" JSON_EPOOLTIMER_FMT "}", system_data.pump_voltage, system_data.pump_current, system_data.pump_power,
		  system_data.box_temperature, system_data.box_pressure, system_data.manOverride, system_data.pumpCmd);

  mbuf_free(&fb);
}


/*
 * update_state. This commands pump on/off
 *
 */
void update_state(void) {
  cmdPumpOnOff(system_data.pumpCmd);
  LOG(LL_INFO, ("Relay: %d\n", system_data.pumpCmd));
}

/*
 * Main AWS Device Shadow state callback handler. Will get invoked when
 * connection is established or when new versions of the state arrive via one of the topics.
 *
 * CONNECTED event comes with no state.
 *
 * For DELTA events, state is passed as "desired", reported is not set.
 *
 */
static void aws_shadow_state_handler(void *arg, enum mgos_aws_shadow_event ev,
                                     uint64_t version,
                                     const struct mg_str reported,
                                     const struct mg_str desired,
                                     const struct mg_str reported_md,
                                     const struct mg_str desired_md) {

  dpt_system_t tempSystemData;

  LOG(LL_INFO, ("== Event: %d (%s), version: %llu", ev,
                mgos_aws_shadow_event_name(ev), version));

  if (ev == MGOS_AWS_SHADOW_CONNECTED) {
    report_state();
    return;
  }
  if (ev != MGOS_AWS_SHADOW_GET_ACCEPTED &&
      ev != MGOS_AWS_SHADOW_UPDATE_DELTA) {
    return;
  }
  LOG(LL_INFO, ("Reported state: %.*s\n", (int) reported.len, reported.p));
  LOG(LL_INFO, ("Desired state : %.*s\n", (int) desired.len, desired.p));
  LOG(LL_INFO, ("Reported metadata: %.*s\n", (int) reported_md.len, reported_md.p));
  LOG(LL_INFO, ("Desired metadata : %.*s\n", (int) desired_md.len, desired_md.p));
  /*
   * Here we extract values from previously reported state (if any)
   * and then override it with desired state (if present).
   */
  json_scanf(reported.p, reported.len, JSON_EPOOLTIMER_FMT, &tempSystemData.pump_voltage, &tempSystemData.pump_current,
		  	  	  	  	  	  	  	  	  	  	  	  	  	&tempSystemData.pump_power, &tempSystemData.box_temperature,
															&tempSystemData.box_pressure, &tempSystemData.manOverride,
															&tempSystemData.pumpCmd);

  json_scanf(desired.p, desired.len, JSON_EPOOLTIMER_FMT, &tempSystemData.pump_voltage, &tempSystemData.pump_current,
  	  	  	  	  	  										&tempSystemData.pump_power, &tempSystemData.box_temperature,
															&tempSystemData.box_pressure, &tempSystemData.manOverride,
															&system_data.pumpCmd);

  update_state();
  if (ev == MGOS_AWS_SHADOW_UPDATE_DELTA) {
    report_state();
  }
  (void) arg;
}

/*
 * periodicCallBackHandler: Function called periodically which updates system
 * 							variables. Gets current state of sensors and inputs
 *
 */
static void periodicCallBackHandler(void *arg) {
	struct bmp180_data_t sensor_data;

	/* Get Current System Data	*/
	bmp180_sensor_initialize();
	bmp180_sensor_data(&sensor_data);

	system_data.dc_input_voltage = ADC_ATTEN * (float)mgos_adc_read(ADC_CH5_GPIO)/ADC_MAX_RES;
//	system_data.dc_input_voltage = ADC_ATTEN * (float)adc1_get_voltage(ADC1_CHANNEL_5)/ADC_MAX_RES;

	system_data.box_temperature = (sensor_data.temperature * 1.8) + 32;
	system_data.box_pressure = sensor_data.pressure;		// convert farenheit
	system_data.manOverride = mgos_gpio_read(MANUAL_OVRD_GPIO);
	system_data.upTime = mgos_uptime();

	LOG(LL_INFO, ("system_data.dc_input_voltage = %.2f", system_data.dc_input_voltage));
	LOG(LL_INFO, ("system_data.upTime = %lf", system_data.upTime));

	/* Send updates to shadow	*/
//	report_state();

	checkPumpSchedule();
	(void)arg;
}

/*
 * measurePumpTask: Function called periodically which measures ADC voltage, current and power
 *
 */
static void measurePumpTask(void *arg) {
	bool status = false;

	if (!gADCConfigured) {
		ade7912_init(eFRQ4khz);
	}
	else {
		status = ade7912_trigger_capture();
		system_data.pump_power = 0.001* system_data.pump_voltage * system_data.pump_current;

		if (status == true) {
			LOG(LL_DEBUG, ("system_data.pump_current = %.2f", system_data.pump_current));
			LOG(LL_DEBUG, ("system_data.pump_voltage = %.2f", system_data.pump_voltage));
			LOG(LL_DEBUG, ("system_data.pump_power = %.2f\n", system_data.pump_power));
		}
	}
	(void)arg;
}

/*
 * checkPumpSchedule: Function determines date and time
 * 					  checking the schedule to see if the pump should be turned on
 *
 */
void checkPumpSchedule(void) {
	/* TODO
	 * Figure out how to get create a schedule for turning pump on off
	 */
	char dtString[100] = {'\0'};

	struct timezone tzEST;
	struct timeval timeNow;
	gettimeofday(&timeNow, &tzEST);
	struct tm *dateAndTime = localtime(&timeNow.tv_sec);
	time_t local_ts = mktime(dateAndTime);

	if (ISTIMESYNCED(dateAndTime->tm_year)) {
//		mgos_strftime(dtString, 100, "%a, %x - %I:%M%p", timeNow.tv_sec);
		if (dateAndTime->tm_isdst == 1) {
			local_ts += (TIME_ZONE_EST-1)*3600;
		}
		else {
			local_ts += TIME_ZONE_EST*3600;
		}
		dateAndTime=localtime(&local_ts);
		strftime(dtString, 100, "%a, %x - %I:%M%p", dateAndTime);
		LOG(LL_DEBUG, ("Local time %s, daylight_savings: %d\n", dtString, dateAndTime->tm_isdst));
	}
}

/*
 * mgos_app_init: Initializes application
 *
 */
enum mgos_app_init_result mgos_app_init(void) {

//	cs_log_set_level(LL_DEBUG);					// Set log level to debug

	device_cfg = get_cfg();
	printf ("mgos_app_init		Device id is: %s \r\n", device_cfg->device.id);

	if (mgos_wifi_validate_sta_cfg(&device_cfg->wifi.sta, &msg) == false) {
		LOG(LL_DEBUG, ("%s\n", msg));
	}
	if (mgos_wifi_validate_ap_cfg(&device_cfg->wifi.ap, &msg) == false) {
		LOG(LL_DEBUG, ("%s\n", msg));
	}

	LOG(LL_DEBUG, ("registering end points..."));
	mgos_register_http_endpoint("/save", http_post_ev_handler, NULL);
	mgos_register_http_endpoint("/get_cpu_usage", http_get_ev_handler, NULL);

	/* Initialize GPIO	*/
	deviceInit();
	system_data.pumpCmd = false;
	cmdPumpOnOff(system_data.pumpCmd);

	/* Initialize bmp180 sensor and i2c peripheral	*/
	LOG(LL_INFO, ("bmp180 sensor initialization result = %d \n", bmp180_sensor_initialize()));
	LOG(LL_DEBUG, ("setup timer call back : %d msec \n", CALLBACK_PERIOD));
	mgos_set_timer(CALLBACK_PERIOD, true, periodicCallBackHandler, NULL);


	/* Initialize ade7912 a/d converter and HSPI peripheral	*/
	gADCConfigured = ade7912_init(eFRQ4khz);
	LOG(LL_DEBUG, ("ade7912 initialization result: %d [1=SUCCESS, 0=FAIL] \n", gADCConfigured));
	LOG(LL_DEBUG, ("setup timer call back : %d msec \n", CALLBACK_PERIOD));
	mgos_set_timer(990, true, measurePumpTask, NULL);

	/* Register AWS shadow callback handler	*/
	mgos_aws_shadow_set_state_handler(aws_shadow_state_handler, NULL);

	LOG(LL_INFO, ("MGOS_APP_INIT_SUCCESS"));
  return MGOS_APP_INIT_SUCCESS;
}

