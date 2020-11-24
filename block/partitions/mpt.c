#include "check.h"
#include <linux/printk.h>

#define MAX_MPT_PART_NAME_LEN	16
#define MAX_MPT_PART_NUM		32

#define MPT_PARTITION_SECTOR_OFFSET	2048

struct partition_mpt {
	char name[MAX_MPT_PART_NAME_LEN];
	uint64_t size;
	uint64_t offset;
	unsigned int mask_flags;
};

struct ptable_t {
	char magic[4];
	unsigned char version[12];
	int nr_parts;
	int checksum;
	struct partition_mpt partitions[MAX_MPT_PART_NUM];
};
/*
 * MPT Magic Label "MPT"
 */
#define MPT_LABEL_MAGIC1	0x4D
#define MPT_LABEL_MAGIC2	0x50
#define MPT_LABEL_MAGIC3	0x54

static inline int
mpt_magic_present(const char *p)
{
	return (p[0] == MPT_LABEL_MAGIC1 &&
			p[1] == MPT_LABEL_MAGIC2 &&
			p[2] == MPT_LABEL_MAGIC3);
}

#define MPT_VERSION		"01.00.00"

static inline int
mpt_version_present(const unsigned char *p)
{
	return strncmp(p, MPT_VERSION, strlen(MPT_VERSION));
}

static uint32_t mpt_partition_checksum(const struct partition_mpt *part, int count)
{
	uint32_t sum = 0;

	while (count--) {
		uint32_t *p = (uint32_t *) part;
		int j = sizeof(struct partition_mpt) / sizeof(sum);

		for (; j > 0; j--)
			sum += *p++;
	}

	return sum;
}

static inline int is_mpt_valid(const struct ptable_t *mpt, int strict)
{
	if (!mpt_magic_present(mpt->magic) || mpt_version_present(mpt->version))
		return 0;

	if (strict) {
		if (mpt->checksum != mpt_partition_checksum(mpt->partitions,
					mpt->nr_parts))
			return 0;
	}

	return 1;
}

static void applyInfo(struct parsed_partitions *state, int slot, struct ptable_t *mpt)
{
	struct partition_meta_info *info = &state->parts[state->next].info;

	info->uuid[0] = 0;
	snprintf(info->volname, sizeof(info->volname), "\x09%s",
			mpt->partitions[slot].name);

	state->parts[state->next].has_info = true;
}

#if defined(CONFIG_ARCH_MESON64_ODROID_COMMON)
#define BOOT_DEVICE_LENGTH 12

static char boot_device[BOOT_DEVICE_LENGTH] = "";

static int __init get_boot_device(char *str)
{
	if (str) {
		strncpy(boot_device, str, sizeof(str) - 1);
		boot_device[sizeof(str)-1] = '\0';
	}
	return 0;
}
__setup("boot_device=", get_boot_device);
#endif

int mpt_partition(struct parsed_partitions *state)
{
	Sector sect;
	unsigned char *data;
	struct ptable_t *mpt;
	struct gendisk *disk;

	data = read_part_sector(state, MPT_PARTITION_SECTOR_OFFSET, &sect);
	if (!data)
		return -1;

	mpt = (struct ptable_t *) data;

	if (!is_mpt_valid(mpt, 1)) {
		put_dev_sector(sect);
		return 0;
	}

	disk = state->bdev->bd_disk;
	if (strncmp(disk->disk_name, boot_device, strlen(disk->disk_name))) {
		printk(KERN_CONT "current, boot - (%s, %s)\n",
				disk->disk_name, boot_device);

		return 1;
	}

	printk(KERN_CONT "magic - %s\n", mpt->magic);
	printk(KERN_CONT "version - %s\n", mpt->version);
	printk(KERN_CONT "checksum - %x, result - %x\n", mpt->checksum, mpt_partition_checksum(mpt->partitions,
				mpt->nr_parts));

	{
		int slot;
		for (slot = 0; slot < mpt->nr_parts; slot++) {
			put_partition(state, state->next,
					le32_to_cpu(mpt->partitions[slot].offset),
					le32_to_cpu(mpt->partitions[slot].size));
			applyInfo(state, slot, mpt);
			state->parts[state->next++].flags =  mpt->partitions[slot].mask_flags;
		}
	}
	put_dev_sector(sect);
	return 1;
}
