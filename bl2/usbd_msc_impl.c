#include "usbd_core.h"
#include "usbd_msc.h"

#include "main.h"
#include "bl2.h"
#include "board.h"
#include "py25q16.h"
#include "fat/fat.h"
#include <string.h>
#include "printf.h"

#define MSC_IN_EP 0x81
#define MSC_OUT_EP 0x02

#define USBD_VID 0x36b7
#define USBD_PID 0xFFFF
#define USBD_MAX_POWER 100
#define USBD_LANGID_STRING 1033

#define USB_CONFIG_SIZE (9 + MSC_DESCRIPTOR_LEN)

const uint8_t msc_flash_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0200, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    MSC_DESCRIPTOR_INIT(0x00, MSC_OUT_EP, MSC_IN_EP, 0x02),
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x0A,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'K', 0x00,                  /* wcChar0 */
    'I', 0x00,                  /* wcChar1 */
    'S', 0x00,                  /* wcChar2 */
    'S', 0x00,                  /* wcChar3 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x12,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'K', 0x00,                  /* wcChar0 */
    'I', 0x00,                  /* wcChar1 */
    'S', 0x00,                  /* wcChar2 */
    'S', 0x00,                  /* wcChar3 */
    '-', 0x00,                  /* wcChar4 */
    'B', 0x00,                  /* wcChar5 */
    'L', 0x00,                  /* wcChar6 */
    '2', 0x00,                  /* wcChar7 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x0A,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    '5', 0x00,                  /* wcChar0 */
    '0', 0x00,                  /* wcChar1 */
    '2', 0x00,                  /* wcChar2 */
    '1', 0x00,                  /* wcChar3 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
#endif
    0x00};

struct usbd_interface intf0;

static volatile struct
{
    bool fmt;
    uint16_t skip_sectors;
} init_params;

void msc_init(bool fmt, uint16_t skip_sectors)
{

    // printf("msc_init \n");

    init_params.fmt = fmt;
    init_params.skip_sectors = skip_sectors;

    backlight_on(20000);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USBD);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    usbd_desc_register(msc_flash_descriptor);
    usbd_add_interface(usbd_msc_init_intf(&intf0, MSC_OUT_EP, MSC_IN_EP));
    usbd_initialize();

    /* Enable USB interrupt */
    NVIC_SetPriority(USBD_IRQn, 3);
    NVIC_EnableIRQ(USBD_IRQn);
}

// -----------------------

#define VOLUME_CREATE_DATE FAT_MK_DATE(2026, 1, 10)
#define VOLUME_CREATE_TIME FAT_MK_TIME(10, 0, 0)
#define VOLUME_LABEL "KISS       " // Volume label, in 8+3 file name format

#define DIR_ENTRIES_PER_SECTOR (FAT_SECTOR_SIZE / FAT_DIR_ENTRY_SIZE)

// Volume layout
enum
{
    // Boot
    BOOT_SECTOR_NUM = 0,
    BOOT_SECTORS = 1,
    // FAT
    FAT_SECTOR_NUM = BOOT_SECTOR_NUM + BOOT_SECTORS,
    FAT_SECTORS = 12,
    FAT_CNT = 1,
    // Root
    ROOT_SECTOR_NUM = FAT_SECTOR_NUM + FAT_CNT * FAT_SECTORS,
    ROOT_SECTORS = 16,
    // Data
    DATA_SECTOR_NUM = ROOT_SECTOR_NUM + ROOT_SECTORS,
};

enum
{
    TOTAL_SECTORS = PY25Q16_SIZE / FAT_SECTOR_SIZE,
    ROOT_ENTRY_CNT = ROOT_SECTORS * FAT_SECTOR_SIZE / FAT_DIR_ENTRY_SIZE,
    BOOT_SECTOR_MEDIA = 0xf0,
};

#define SKIP_SECTORS (init_params.skip_sectors)
#define BL_DUR 30000

static volatile struct
{
    bool fs_initialized;
    bool connected;
    uint32_t alive_time;
} usb_state = {0};

static void print_sector(const uint8_t *buf, uint32_t size);
static bool check_fs();
static void init_fs();

void usbd_configure_done_callback(void)
{
}

void usbd_msc_heart_beat()
{
    // printf("usbd_msc_heart_beat \n");

    usb_state.alive_time = systick_get_ticks_ms();
}

bool msc_update_alive()
{
    bool alive;

    __disable_irq();
    if (usb_state.connected)
    {
        uint32_t dt = systick_get_ticks_ms() - usb_state.alive_time;
        alive = dt < 1000;
    }
    __enable_irq();

    if (!alive)
    {
        // printf("msc_update_alive: not alive \n");
        backlight_off();
        // TODO:
    }

    return alive;
}

void usbd_msc_get_cap(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
    // printf("usb connected\n");
    // backlight_on(BL_DUR);

    usb_state.connected = true;
    usb_state.alive_time = systick_get_ticks_ms();

    *block_size = FAT_SECTOR_SIZE;
    *block_num = TOTAL_SECTORS - SKIP_SECTORS;
}

int usbd_msc_sector_read(uint32_t sector, uint8_t *buffer, uint32_t length)
{
    if (!usb_state.fs_initialized)
    {
        init_fs();
        usb_state.fs_initialized = true;
    }

    // printf("usbd_msc_sector_read: length = %d\n", length);

    if (FAT_SECTOR_SIZE != length)
    {
        // printf("usbd_msc_sector_read: length = %d\n", length);
        return 1;
    }

    backlight_on(BL_DUR);

    uint32_t addr = (SKIP_SECTORS + sector) * FAT_SECTOR_SIZE;
    py25q16_read(addr, buffer, FAT_SECTOR_SIZE);
    return 0;
}

int usbd_msc_sector_write(uint32_t sector, uint8_t *buffer, uint32_t length)
{
    if (FAT_SECTOR_SIZE != length)
    {
        // printf("usbd_msc_sector_write: length = %d\n", length);
        return 1;
    }

    // printf("usbd_msc_sector_write: sector = %d\n", sector);
    // print_sector(buffer, length);

    backlight_on(BL_DUR);

    if (BOOT_SECTOR_NUM == sector)
    {
        // printf("Boot sector write skipped\n");
        return 0;
    }

    uint32_t addr = (SKIP_SECTORS + sector) * FAT_SECTOR_SIZE;
    py25q16_write(addr, buffer, FAT_SECTOR_SIZE);
    return 0;
}

static bool check_fs()
{
    // Boot sector
    do
    {
        uint16_t sig;
        do
        {
            uint32_t addr = SKIP_SECTORS * FAT_SECTOR_SIZE + 510;
            py25q16_read(addr, (uint8_t *)&sig, 2);
        } while (0);

        if (FAT_SIGNATURE_WORD != sig)
        {
            return false;
        }
    } while (0);

    return true;
}

static void init_fs()
{
    if (init_params.fmt)
    {
        // OK
    }
    else
    {
        if (check_fs())
        {
            return;
        }
    }

    // printf("initializing fs..\n");

    uint8_t buf[FAT_SECTOR_SIZE] __attribute__((aligned(4)));

    // Boot
    do
    {
        memset(buf, 0, sizeof(buf));

        FAT_boot_sector_t *boot_sec = (FAT_boot_sector_t *)buf;
        memcpy(boot_sec->jump_boot, "\xeb\x3c\x90", sizeof(boot_sec->jump_boot));
        memcpy(boot_sec->OEM_name, "KISS-BL2", sizeof(boot_sec->OEM_name));
        boot_sec->sector_size = FAT_SECTOR_SIZE;
        boot_sec->cluster_size = 1;
        boot_sec->reserved_sectors = BOOT_SECTORS;
        boot_sec->FAT_cnt = FAT_CNT;
        boot_sec->root_entry_cnt = ROOT_ENTRY_CNT;
        boot_sec->total_sectors16 = TOTAL_SECTORS - SKIP_SECTORS;
        boot_sec->total_sectors32 = 0;
        boot_sec->media = BOOT_SECTOR_MEDIA;
        boot_sec->FAT_size16 = FAT_SECTORS;
        boot_sec->track_sectors = 4;
        boot_sec->heads = 1;
        boot_sec->hidden_sectors = 0;
        boot_sec->drive_num = 0x80;
        boot_sec->reserved1 = 0; //
        boot_sec->boot_signature = 0x29;
        boot_sec->volume_ID = (VOLUME_CREATE_DATE << 16) | VOLUME_CREATE_TIME;
        memcpy(boot_sec->volume_label, VOLUME_LABEL, sizeof(boot_sec->volume_label));
        memcpy(boot_sec->fs_type, "FAT12   ", sizeof(boot_sec->fs_type));

        FAT_set_hword(&buf[FAT_SECTOR_SIZE - 2], FAT_SIGNATURE_WORD);

        uint32_t addr = FAT_SECTOR_SIZE * (SKIP_SECTORS + BOOT_SECTOR_NUM);
        py25q16_write(addr, buf, FAT_SECTOR_SIZE);

        // printf("Initialize boot sector: \n");
        // print_sector(buf, FAT_SECTOR_SIZE);
    } while (0);

    // FAT
    do
    {
        memset(buf, 0, FAT_SECTOR_SIZE);

        // The 1st sector
        do
        {
            FAT12_set_entry_value(buf + FAT12_ENTRY_OFFSET(0), 0xf00 | BOOT_SECTOR_MEDIA, 0);
            FAT12_set_entry_value(buf + FAT12_ENTRY_OFFSET(1), FAT12_EOF, 1);

            uint32_t addr = FAT_SECTOR_SIZE * (SKIP_SECTORS + FAT_SECTOR_NUM);
            py25q16_write(addr, buf, FAT_SECTOR_SIZE);
        } while (0);

        //
        memset(buf, 0, FAT_SECTOR_SIZE);

        for (uint32_t i = 1; i < FAT_SECTORS; i++)
        {
            uint32_t addr = FAT_SECTOR_SIZE * (SKIP_SECTORS + FAT_SECTOR_NUM + i);
            py25q16_write(addr, buf, FAT_SECTOR_SIZE);
        }
    } while (0);

    // Root
    do
    {
        memset(buf, 0, FAT_SECTOR_SIZE);

        // Volume label
        do
        {
            FAT_dir_entry_t *de = (FAT_dir_entry_t *)buf;
            memcpy(de->name, VOLUME_LABEL, sizeof(de->name));
            de->attr = FAT_DIR_ATTR_VOLUME_ID;
            de->create_date = VOLUME_CREATE_DATE;
            de->create_time = VOLUME_CREATE_TIME;
            de->access_date = VOLUME_CREATE_DATE;
            de->write_date = VOLUME_CREATE_DATE;
            de->write_time = VOLUME_CREATE_TIME;

            uint32_t addr = FAT_SECTOR_SIZE * (SKIP_SECTORS + ROOT_SECTOR_NUM);
            py25q16_write(addr, buf, FAT_SECTOR_SIZE);
        } while (0);

        memset(buf, 0, FAT_SECTOR_SIZE);

        //
        for (uint32_t i = 1; i < ROOT_SECTORS; i++)
        {
            uint32_t addr = FAT_SECTOR_SIZE * (SKIP_SECTORS + ROOT_SECTOR_NUM + i);
            py25q16_write(addr, buf, FAT_SECTOR_SIZE);
        }

    } while (0);

    // printf("initializing fs done.\n");
}

static void print_sector(const uint8_t *buf, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        if (0 == i % 16)
        {
            printf("\n%04X:", i);
        }

        printf(" %02x", buf[i]);
    }

    printf("\n");
}
