/* main.c - Application main entry point */


/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/slist.h>
#include <zephyr/types.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/drivers/gpio.h>

#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
// This enables you to unpair with all devices paired
// by pluging in the USB with the user button pressed
#define PAIR_BUTTON

#ifdef PAIR_BUTTON


#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
#endif

LOG_MODULE_REGISTER(myapp, CONFIG_MYAPP_LOG_LEVEL);

static void start_scan(void);

static struct bt_conn *default_conn;

static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_read_params read_params;

static uint16_t hid_service_start_handle;
static uint16_t hid_service_end_handle;

struct hid_report_description {
	sys_snode_t node;

	uint16_t handle;
	uint16_t value_handle;
	uint16_t ccc_handle;
	uint8_t report_id;
	uint8_t report_type;
};

struct subscribe_params_for_list {
	sys_snode_t node;
	struct bt_gatt_subscribe_params subscribe_params;
};

static sys_slist_t hid_report_desc_list;
static sys_slist_t subscribe_params_list;

// static struct bt_gatt_subscribe_params subscribe_params;

#define REPORT_MAP_MAX_SIZE 512
static uint8_t report_map_buf[REPORT_MAP_MAX_SIZE];
static uint16_t report_map_size = 0;

static struct hid_report_description *current_report_description = NULL;
static struct hid_report_description *next_report_description = NULL;

static bool usb_configured;
static const struct device *hdev;
static ATOMIC_DEFINE(hid_ep_in_busy, 1);

#define HID_EP_BUSY_FLAG 0

struct report_queue_item {
	uint8_t *data;
	uint16_t length;
};
struct k_fifo reports;

static K_THREAD_STACK_DEFINE(report_thread_stack, 512);
static struct k_thread report_thread_data;

K_SEM_DEFINE(usb_ready, 0, 1);

static void fill_reports(struct bt_conn *conn);

static bool iterate_report_description()
{
	// Initial setup
	if (current_report_description == NULL) {
		sys_snode_t *node = sys_slist_peek_head(&hid_report_desc_list);
		if (node == NULL) {
			return false;
		}
		current_report_description =
			SYS_SLIST_CONTAINER(node, current_report_description, node);

		sys_snode_t *next_node = sys_slist_peek_next(node);
		if (next_node != NULL) {
			next_report_description =
				SYS_SLIST_CONTAINER(next_node, next_report_description, node);
		}

		return true;
	}

	current_report_description = next_report_description;
	next_report_description = NULL;
	if (current_report_description == NULL) {
		return false;
	}

	sys_snode_t *next_node = sys_slist_peek_next(&current_report_description->node);
	if (next_node != NULL) {
		next_report_description =
			SYS_SLIST_CONTAINER(next_node, next_report_description, node);
	}
	return true;
}

static uint8_t notify_func(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	if (!data) {
		printk("[UNSUBSCRIBED?]\n");
	}

	uint8_t report_id = 0;

	struct hid_report_description *desc;
	SYS_SLIST_FOR_EACH_CONTAINER(&hid_report_desc_list, desc, node) {
		// Skip non-input reports
		if (desc->ccc_handle == params->ccc_handle) {
			report_id = desc->report_id;
			break;
		}
	}

	printk("[NOTIFICATION] ID %hu: ", report_id);
	const uint8_t *data_bytes = data;
	for (uint16_t i = 0; i < length; i++) {
		printk("%02x", data_bytes[i]);
	}
	printk("\n");

	uint8_t *item_buf = k_malloc(sizeof(struct report_queue_item) + length + 1);
	struct report_queue_item *item = item_buf;
	item->data = item_buf + sizeof(struct report_queue_item);
	item->length = length + 1;
	item->data[0] = report_id;
	memcpy(item->data + 1, data, length);

	k_fifo_alloc_put(&reports, item_buf);

	// if (!atomic_test_and_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
	// 	ret = hid_int_ep_write(hdev, output_buf, length + 1, &wrote);
	// 	k_free(output_buf);
	// 	if (ret != 0) {
	// 		/*
	// 		 * Do nothing and wait until host has reset the device
	// 		 * and hid_ep_in_busy is cleared.
	// 		 */
	// 		LOG_ERR("Failed to submit report");
	// 	} else {
	// 		LOG_DBG("Report submitted");
	// 	}
	// } else {
	// 	LOG_DBG("HID IN endpoint busy");

	// }

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t handle_report_ref_data(struct bt_conn *conn, uint8_t err,
				      struct bt_gatt_read_params *params, const void *data,
				      uint16_t length)
{
	if (err) {
		printk("Failed to read report ref: %u\n", err);
		return BT_GATT_ITER_STOP;
	}

	if (length != 2) {
		printk("Report reference had the wrong length: %hu\n", length);
		return BT_GATT_ITER_STOP;
	}

	const uint8_t *char_data = data;
	current_report_description->report_id = char_data[0];
	current_report_description->report_type = char_data[1];

	printk("Finished filling report %hu:\n\tValue handle: %hu\n\tCCC handle: %hu\n\tReport ID: "
	       "%hu\n\tReport Type: %hu\n\n",
	       current_report_description->handle, current_report_description->value_handle,
	       current_report_description->ccc_handle, current_report_description->report_id,
	       current_report_description->report_type);

	fill_reports(conn);

	return BT_GATT_ITER_STOP;
}

static uint8_t report_ref_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
					struct bt_gatt_discover_params *params)
{
	if (!attr) {
		printk("Finished searching for report descriptors.\n");
		(void)memset(params, 0, sizeof(*params));
		fill_reports(conn);
		return BT_GATT_ITER_STOP;
	}

	printk("Found report descriptor %hu\n", attr->handle);

	memset(&read_params, 0, sizeof(struct bt_gatt_read_params));
	read_params.handle_count = 1;
	read_params.single.handle = attr->handle;
	read_params.single.offset = 0;
	read_params.func = handle_report_ref_data;
	if (bt_gatt_read(conn, &read_params)) {
		printk("Error reading report map\n");
	}

	return BT_GATT_ITER_STOP;
}

static void discover_report_ref_for_report(struct bt_conn *conn)
{
	memcpy(&uuid, BT_UUID_HIDS_REPORT_REF, sizeof(uuid));
	discover_params.uuid = &uuid.uuid;
	discover_params.func = report_ref_discover_func;
	discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;

	uint8_t err = bt_gatt_discover(default_conn, &discover_params);
	if (err) {
		printk("Discover failed(err %d)\n", err);
	}
}

static uint8_t report_ccc_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
					struct bt_gatt_discover_params *params)
{
	if (!attr) {
		printk("Finished searching for CCC reports.\n");
		(void)memset(params, 0, sizeof(*params));
		discover_report_ref_for_report(conn);
		return BT_GATT_ITER_STOP;
	}

	current_report_description->ccc_handle = attr->handle;

	printk("Found report CCC descriptor %hu\n", attr->handle);

	discover_report_ref_for_report(conn);

	return BT_GATT_ITER_STOP;
}

static void discover_ccc_for_report(struct bt_conn *conn)
{
	memset(&discover_params, 0, sizeof(struct bt_gatt_discover_params));
	memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
	discover_params.uuid = &uuid.uuid;
	discover_params.func = report_ccc_discover_func;
	discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;

	discover_params.start_handle = current_report_description->handle;
	discover_params.end_handle = next_report_description != NULL
					     ? next_report_description->handle
					     : hid_service_end_handle;

	uint8_t err = bt_gatt_discover(default_conn, &discover_params);
	if (err) {
		printk("Discover failed(err %d)\n", err);
	}
}

static void subscribe(struct bt_conn *conn)
{
	sys_slist_init(&subscribe_params_list);

	struct hid_report_description *desc;
	SYS_SLIST_FOR_EACH_CONTAINER(&hid_report_desc_list, desc, node) {
		// Skip non-input reports
		if (desc->report_type != 0x01) {
			continue;
		}

		struct subscribe_params_for_list *params_container =
			k_malloc(sizeof(struct subscribe_params_for_list));
		memset(params_container, 0, sizeof(struct subscribe_params_for_list));
		sys_slist_append(&subscribe_params_list, &params_container->node);

		params_container->subscribe_params.value_handle = desc->value_handle;
		params_container->subscribe_params.ccc_handle = desc->ccc_handle;
		params_container->subscribe_params.value = BT_GATT_CCC_NOTIFY;
		params_container->subscribe_params.notify = notify_func;

		uint8_t err = bt_gatt_subscribe(conn, &params_container->subscribe_params);
		if (err && err != -EALREADY) {
			printk("Subscribe failed (err %d)\n", err);
		} else {
			printk("[SUBSCRIBED] value handle: %hu, ccc_handle: %hu\n",
			       params_container->subscribe_params.value_handle,
			       params_container->subscribe_params.ccc_handle);
		}
	}
}

static void int_in_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	k_sem_give(&usb_ready);
}

static void process_reports()
{
	LOG_INF("Processing reports...");

	while (true) {
		struct report_queue_item *item = k_fifo_get(&reports, K_FOREVER);
		uint32_t wrote;
		k_sem_take(&usb_ready, K_FOREVER);
		uint8_t ret = hid_int_ep_write(hdev, item->data, item->length, &wrote);
		k_free(item);
		if (ret != 0) {
			LOG_ERR("Failed to submit report");
		} else {
			LOG_DBG("Report submitted");
		}
	}
}

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	switch (status) {
	case USB_DC_RESET:
		usb_configured = false;
		break;
	case USB_DC_CONFIGURED:
		if (!usb_configured) {
			int_in_ready_cb(hdev);
			usb_configured = true;
		}
		break;
	case USB_DC_SOF:
		break;
	default:
		LOG_DBG("status %u unhandled", status);
		break;
	}
}

static const struct hid_ops ops = {
	.int_in_ready = int_in_ready_cb,
};

static void setup_usb_hid()
{
	hdev = device_get_binding("HID_0");
	if (hdev == NULL) {
		LOG_ERR("Cannot get USB HID Device");
		return;
	}

	LOG_INF("HID Device: dev %p", hdev);

	k_fifo_init(&reports);
	k_thread_create(&report_thread_data, report_thread_stack,
			K_THREAD_STACK_SIZEOF(report_thread_stack), process_reports, NULL, NULL,
			NULL, 2, 0, K_NO_WAIT);

	usb_hid_register_device(hdev, report_map_buf, report_map_size, &ops);
	usb_hid_init(hdev);

	uint8_t ret = usb_enable(status_cb);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}
}

static void fill_reports(struct bt_conn *conn)
{
	if (iterate_report_description()) {
		printk("Filling report %hu\n", current_report_description->handle);
		discover_ccc_for_report(conn);
		return;
	}
	printk("Done filling reports\n");
	subscribe(conn);
	setup_usb_hid();
}

static uint8_t report_chrc_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
					 struct bt_gatt_discover_params *params)
{
	if (!attr) {
		printk("Finished searching for HID reports.\n");
		(void)memset(params, 0, sizeof(*params));
		fill_reports(conn);
		return BT_GATT_ITER_STOP;
	}

	struct hid_report_description *report_chrc_desc =
		k_malloc(sizeof(struct hid_report_description));
	memset(report_chrc_desc, 0, sizeof(struct hid_report_description));
	sys_slist_append(&hid_report_desc_list, &report_chrc_desc->node);

	report_chrc_desc->handle = attr->handle;
	report_chrc_desc->value_handle = bt_gatt_attr_value_handle(attr);

	printk("Found HID report handle %hu, value handle %hu\n", report_chrc_desc->handle,
	       report_chrc_desc->value_handle);

	return BT_GATT_ITER_CONTINUE;
}

static void discover_reports(struct bt_conn *conn)
{
	sys_slist_init(&hid_report_desc_list);

	memset(&discover_params, 0, sizeof(struct bt_gatt_discover_params));
	memcpy(&uuid, BT_UUID_HIDS_REPORT, sizeof(uuid));
	discover_params.uuid = &uuid.uuid;
	discover_params.func = report_chrc_discover_func;
	discover_params.start_handle = hid_service_start_handle;
	discover_params.end_handle = hid_service_end_handle;
	discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

	uint8_t err = bt_gatt_discover(default_conn, &discover_params);
	if (err) {
		printk("Discover failed(err %d)\n", err);
	}
}

static uint8_t handle_report_map_data(struct bt_conn *conn, uint8_t err,
				      struct bt_gatt_read_params *params, const void *data,
				      uint16_t length)
{
	if (err) {
		printk("Failed to read report map: %u\n", err);
		return BT_GATT_ITER_STOP;
	}

	if (data == NULL) {
		printk("Found report map, %u bytes\n", report_map_size);
		printk("[Report Map] ");
		for (uint16_t i = 0; i < report_map_size; i++) {
			printk("%x", report_map_buf[i]);
		}
		printk("\n");

		discover_reports(conn);

		return BT_GATT_ITER_STOP;
	}

	memcpy(report_map_buf + report_map_size, data, length);
	report_map_size += length;

	read_params.single.offset = report_map_size;

	if (bt_gatt_read(conn, &read_params)) {
		printk("Error reading report map\n");
	}

	return BT_GATT_ITER_STOP;
}

static uint8_t report_map_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
					struct bt_gatt_discover_params *params)
{
	if (!attr) {
		printk("No HID report map found.\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	read_params.handle_count = 1;
	read_params.single.handle = bt_gatt_attr_value_handle(attr);
	read_params.single.offset = 0;
	read_params.func = handle_report_map_data;
	if (bt_gatt_read(conn, &read_params)) {
		printk("Error reading report map\n");
	}
	return BT_GATT_ITER_STOP;
}

static void discover_report_map(struct bt_conn *conn)
{
	memset(&discover_params, 0, sizeof(struct bt_gatt_discover_params));
	memcpy(&uuid, BT_UUID_HIDS_REPORT_MAP, sizeof(uuid));
	discover_params.uuid = &uuid.uuid;
	discover_params.func = report_map_discover_func;
	discover_params.start_handle = hid_service_start_handle;
	discover_params.end_handle = hid_service_end_handle;
	discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

	uint8_t err = bt_gatt_discover(default_conn, &discover_params);
	if (err) {
		printk("Discover failed(err %d)\n", err);
	}
}

static uint8_t service_discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				     struct bt_gatt_discover_params *params)
{
	if (!attr) {
		printk("No HID service found.\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	hid_service_start_handle = attr->handle + 1;
	hid_service_end_handle = ((struct bt_gatt_service_val *)attr->user_data)->end_handle;
	printk("Found HID service from %hu to %hu\n", hid_service_start_handle,
	       hid_service_end_handle);

	discover_report_map(conn);

	return BT_GATT_ITER_STOP;
}

static bool eir_found(struct bt_data *data, void *user_data)
{
	bt_addr_le_t *addr = user_data;
	int i;

	printk("[AD]: %u data_len %u\n", data->type, data->data_len);

	switch (data->type) {
	case BT_DATA_UUID16_SOME:
	case BT_DATA_UUID16_ALL:
		if (data->data_len % sizeof(uint16_t) != 0U) {
			printk("AD malformed\n");
			return true;
		}

		for (i = 0; i < data->data_len; i += sizeof(uint16_t)) {
			struct bt_le_conn_param *param;
			struct bt_uuid *uuid;
			uint16_t u16;
			int err;

			memcpy(&u16, &data->data[i], sizeof(u16));
			uuid = BT_UUID_DECLARE_16(sys_le16_to_cpu(u16));
			if (bt_uuid_cmp(uuid, BT_UUID_HIDS)) {
				continue;
			}

			err = bt_le_scan_stop();
			if (err) {
				printk("Stop LE scan failed (err %d)\n", err);
				continue;
			}

			param = BT_LE_CONN_PARAM_DEFAULT;
			err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &default_conn);
			if (err) {
				printk("Create conn failed (err %d)\n", err);
				start_scan();
			}

			return false;
		}
	}

	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char dev[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, dev, sizeof(dev));
	printk("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i\n", dev, type, ad->len, rssi);

	/* We're only interested in connectable events */
	if (type == BT_GAP_ADV_TYPE_ADV_IND || type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		bt_data_parse(ad, eir_found, (void *)addr);
	}
}

static void start_scan(void)
{
	int err;

	/* Use active scanning and disable duplicate filtering to handle any
	 * devices that might update their advertising data at runtime. */
	struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_ACTIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};

	err = bt_le_scan_start(&scan_param, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		printk("Failed to connect to %s (%u)\n", addr, conn_err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	printk("Connected: %s\n", addr);

	err = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (err) {
		printk("Pairing L2 failed(err %d)\n", err);
	}

	// if (conn == default_conn) {

	// }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{

	sys_reboot(SYS_REBOOT_COLD);

	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	if (default_conn != conn) {
		return;
	}

	bt_conn_unref(default_conn);
	default_conn = NULL;

	struct hid_report_description *desc;
	struct hid_report_description *desc_safe;
	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&hid_report_desc_list, desc, desc_safe, node) {
		k_free(desc);
	}

	struct subscribe_params_for_list *params;
	struct subscribe_params_for_list *params_safe;
	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&subscribe_params_list, params, params_safe, node) {
		k_free(params);
	}

	start_scan();
}

static void identity_resolved(struct bt_conn *conn, const bt_addr_le_t *rpa,
			      const bt_addr_le_t *identity)
{
	char addr_identity[BT_ADDR_LE_STR_LEN];
	char addr_rpa[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(identity, addr_identity, sizeof(addr_identity));
	bt_addr_le_to_str(rpa, addr_rpa, sizeof(addr_rpa));

	printk("Identity resolved %s -> %s\n", addr_rpa, addr_identity);
}

static void discover_service(struct bt_conn *conn)
{
	memset(&discover_params, 0, sizeof(struct bt_gatt_discover_params));
	memcpy(&uuid, BT_UUID_HIDS, sizeof(uuid));
	discover_params.uuid = &uuid.uuid;
	discover_params.func = service_discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	uint8_t err = bt_gatt_discover(default_conn, &discover_params);
	if (err) {
		printk("Discover failed(err %d)\n", err);
		return;
	}
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Security failed: %s level %u err %d\n", addr, level, err);
		return;
	}

	printk("Security changed: %s level %u\n", addr, level);

	discover_service(conn);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.identity_resolved = identity_resolved,
	.security_changed = security_changed,
};

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
};

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	if (settings_save()) {
		printk("Failed to save settings.\n");
	}
	printk("Pairing Complete\n");
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	printk("Pairing Failed (%d). Disconnecting.\n", reason);
	bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
}

static struct bt_conn_auth_info_cb auth_cb_info = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};

void main(void)
{
	// if (usb_enable(NULL)) {
	// 	return;
	// }

	// k_sleep(K_SECONDS(10));

	int err;
	err = bt_enable(NULL);
	if (settings_load()) {
		printk("Failed to load settings.\n");
	}


	bt_conn_auth_cb_register(&auth_cb_display);
	bt_conn_auth_info_cb_register(&auth_cb_info);

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");


#if IS_ENABLED(CONFIG_BLE_CLEAR_BONDS_ON_START)

	if(bt_unpair(BT_ID_DEFAULT,BT_ADDR_LE_ANY)){
		printk("Failed to unpair all connections.\n");
	}
	if(settings_save()){
		printk("Failed to save settings.\n");
	};

	return;

#endif

	#ifdef PAIR_BUTTON


	int ret;
	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return 0;
	}


	if (!gpio_is_ready_dt(&button)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return 0;
	}

	if(gpio_pin_get_raw(button.port, button.pin) == 0){


		if(bt_unpair(BT_ID_DEFAULT,BT_ADDR_LE_ANY)){
			printk("Failed to unpair all connections.\n");
		}
		if(settings_save()){
			printk("Failed to save settings.\n");
		};
		// just a signal when the pair is resetted
		for(int i = 0; i < 6; i++) {
			gpio_pin_toggle_dt(&led);
			k_msleep(1000);
		}
	}
	#endif
	start_scan();
}
