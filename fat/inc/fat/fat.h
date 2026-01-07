#ifndef _FAT_H
#define _FAT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>

    // -------------------------

    static inline void FAT_set_hword(uint8_t *buf, uint16_t n)
    {
        buf[0] = 0xff & n;
        buf[1] = 0xff & (n >> 8);
    }

    static inline uint16_t FAT_get_hword(const uint8_t *buf)
    {
        return (buf[1] << 8) | buf[0];
    }

    static inline void FAT_set_word(uint8_t *buf, uint32_t n)
    {
        buf[0] = 0xff & n;
        buf[1] = 0xff & (n >> 8);
        buf[2] = 0xff & (n >> 16);
        buf[3] = 0xff & (n >> 24);
    }

    static inline uint32_t FAT_get_word(const uint8_t *buf)
    {
        return (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
    }

    // -------------------------

#define FAT_SECTOR_SIZE 512
#define FAT_SIGNATURE_WORD 0xaa55 // At the end of the boot sector (byte offset 510-511 )

    /**
     *  Boot sector
     */
    typedef struct
    {
        uint8_t jump_boot[3];      // Jump instruction to boot code
        char OEM_name[8];          // OEM Name Identifier
        uint16_t sector_size;      // Count of bytes per sector
        uint8_t cluster_size;      // Number of sectors per allocation unit
        uint16_t reserved_sectors; // Number of (reserved) sectors in the reserved region
        uint8_t FAT_cnt;           // The count of file allocation tables (FATs)
        uint16_t root_entry_cnt;   // The count of 32-byte directory entries in the root directory
        uint16_t total_sectors16;  // The old 16-bit total count of sectors on the volume
        uint8_t media;             //  For removable media, 0xF0 is frequently used
        uint16_t FAT_size16;       //  The FAT12/FAT16 16-bit count of sectors occupied by one FAT
        uint16_t track_sectors;    // Sectors per track for interrupt 0x13
        uint16_t heads;            // Number of heads for interrupt 0x13
        uint32_t hidden_sectors;   // Count of hidden sectors preceding the partition
        uint32_t total_sectors32;  // The new 32-bit total count of sectors on the volume
        // Extended BPB structure  ---
        uint8_t drive_num;      // Interrupt 0x13 drive number
        uint8_t reserved1;      // Reserved. Set value to 0x0
        uint8_t boot_signature; // Extended boot signature
        uint32_t volume_ID;     // Volume serial number
        char volume_label[11];  // Volume label
        char fs_type[8];        //  One of the strings “FAT12 ”, “FAT16 ”, or “FAT”.
    } __attribute__((packed)) FAT_boot_sector_t;

    static_assert(62 == sizeof(FAT_boot_sector_t));

    // -------------------------

#define FAT12_ENTRY_OFFSET(index) ((index) + (index) / 2)
#define FAT12_ENTRY_SHIFT(index) (((index) & 1) ? 4 : 0)
#define FAT12_ENTRY_MASK 0xfff

    /**
     * FAT12 entry values
     */
    enum
    {
        FAT12_FREE = 0,    // Cluster is free
        FAT12_BAD = 0xff7, // Indicates a bad (defective) cluster
        FAT12_EOF = 0xfff, // Cluster is allocated and is the final cluster for the file (indicates end-of-file)
    };

    static inline uint16_t FAT12_get_entry_value(const uint8_t *buf, uint32_t index)
    {
        return FAT12_ENTRY_MASK & (FAT_get_hword(buf) >> FAT12_ENTRY_SHIFT(index));
    }

    static inline void FAT12_set_entry_value(uint8_t *buf, uint16_t value, uint32_t index)
    {
        const uint32_t shift = FAT12_ENTRY_SHIFT(index);
        FAT_set_hword(buf, (FAT_get_hword(buf) & ~(FAT12_ENTRY_MASK << shift)) | (value << shift));
    }

    // ------------------

#define FAT_DIR_ENTRY_SIZE 32

#define FAT_MK_DATE(y, m, d) ((uint16_t)(((0x7f & ((y) - 1980)) << 9) | ((0xf & (m)) << 5) | (0x1f & (d))))
#define FAT_MK_TIME(h, m, s) ((uint16_t)(((0x1f & (h)) << 11) | ((0x3f & (m)) << 5) | (0x1f & ((s) / 2))))

    // File attributes
    enum
    {
        FAT_DIR_ATTR_READ_ONLY = 0x01,
        FAT_DIR_ATTR_HIDDEN = 0x02,
        FAT_DIR_ATTR_SYSTEM = 0x04,
        FAT_DIR_ATTR_VOLUME_ID = 0x08,
        FAT_DIR_ATTR_DIRECTORY = 0x10,
        FAT_DIR_ATTR_ARCHIVE = 0x20,
    };

    typedef struct
    {
        char name[11];             // “Short” file name
        uint8_t attr;              // File attribute
        uint8_t reserved1;         // Reserved. Must be set to 0
        uint8_t create_time_10th;  // Component of the file creation time. Count of tenths of a second
        uint16_t create_time;      // Creation time. Granularity is 2 seconds
        uint16_t create_date;      // Creation date
        uint16_t access_date;      // Last access date
        uint16_t first_cluster_HI; //  High word of first data cluster number
        uint16_t write_time;       // Last modification (write) time
        uint16_t write_date;       // Last modification (write) date
        uint16_t first_cluster_LO; //  Low word of first data cluster number
        uint32_t file_size;        // File size in bytes
    } FAT_dir_entry_t;

    static_assert(FAT_DIR_ENTRY_SIZE == sizeof(FAT_dir_entry_t));

#ifdef __cplusplus
} // extern "C"
#endif

#endif
