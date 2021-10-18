/* SPDX-License-Identifier: ISC */
/* Copyright (C) 2021 MediaTek Inc. */

#define FIRMWARE_MT7622		"mediatek/mt7622pr2h.bin"
#define FIRMWARE_MT7663		"mediatek/mt7663pr2h.bin"
#define FIRMWARE_MT7668		"mediatek/mt7668pr2h.bin"
#define FIRMWARE_MT7961		"mediatek/BT_RAM_CODE_MT7961_1_2_hdr.bin"
#define FIRMWARE_MT7925		"mediatek/mt7925/BT_RAM_CODE_MT7925_1_1_hdr.bin"

#define HCI_EV_WMT 0xe4
#define HCI_WMT_MAX_EVENT_SIZE		64

enum {
	BTMTK_WMT_PATCH_DWNLD = 0x1,
	BTMTK_WMT_TEST = 0x2,
	BTMTK_WMT_WAKEUP = 0x3,
	BTMTK_WMT_HIF = 0x4,
	BTMTK_WMT_FUNC_CTRL = 0x6,
	BTMTK_WMT_RST = 0x7,
	BTMTK_WMT_SEMAPHORE = 0x17,
};

enum {
	BTMTK_WMT_INVALID,
	BTMTK_WMT_PATCH_UNDONE,
	BTMTK_WMT_PATCH_PROGRESS,
	BTMTK_WMT_PATCH_DONE,
	BTMTK_WMT_ON_UNDONE,
	BTMTK_WMT_ON_DONE,
	BTMTK_WMT_ON_PROGRESS,
};

struct btmtk_wmt_hdr {
	u8	dir;
	u8	op;
	__le16	dlen;
	u8	flag;
} __packed;

struct btmtk_hci_wmt_cmd {
	struct btmtk_wmt_hdr hdr;
	u8 data[];
} __packed;

struct btmtk_hci_wmt_evt {
	struct hci_event_hdr hhdr;
	struct btmtk_wmt_hdr whdr;
} __packed;

struct btmtk_hci_wmt_evt_funcc {
	struct btmtk_hci_wmt_evt hwhdr;
	__be16 status;
} __packed;

struct btmtk_tci_sleep {
	u8 mode;
	__le16 duration;
	__le16 host_duration;
	u8 host_wakeup_pin;
	u8 time_compensation;
} __packed;

struct btmtk_hci_wmt_params {
	u8 op;
	u8 flag;
	u16 dlen;
	const void *data;
	u32 *status;
};

typedef int (*btmtk_reset_sync_func_t)(struct hci_dev *, void *);

struct btmediatek_data {
	u32 dev_id;
	btmtk_reset_sync_func_t reset_sync;
};

typedef int (*wmt_cmd_sync_func_t)(struct hci_dev *,
				   struct btmtk_hci_wmt_params *);

#if IS_ENABLED(CONFIG_BT_MTK)

int btmtk_set_bdaddr(struct hci_dev *hdev, const bdaddr_t *bdaddr);

int btmtk_setup_firmware_79xx(struct hci_dev *hdev, const char *fwname,
			      wmt_cmd_sync_func_t wmt_cmd_sync);

int btmtk_setup_firmware(struct hci_dev *hdev, const char *fwname,
			 wmt_cmd_sync_func_t wmt_cmd_sync);

void btmtk_reset_sync(struct hci_dev *hdev);
#else

static inline int btmtk_set_bdaddr(struct hci_dev *hdev,
				   const bdaddr_t *bdaddr)
{
	return -EOPNOTSUPP;
}

static int btmtk_setup_firmware_79xx(struct hci_dev *hdev, const char *fwname,
				     wmt_cmd_sync_func_t wmt_cmd_sync)
{
	return -EOPNOTSUPP;
}

static int btmtk_setup_firmware(struct hci_dev *hdev, const char *fwname,
				wmt_cmd_sync_func_t wmt_cmd_sync)
{
	return -EOPNOTSUPP;
}

static void btmtk_reset_sync(struct hci_dev *hdev)
{
}
#endif
