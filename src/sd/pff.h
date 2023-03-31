/*---------------------------------------------------------------------------/
/  Petit FatFs - FAT file system module include file  R0.03   (C)ChaN, 2014
/----------------------------------------------------------------------------/
/ Petit FatFs module is an open source software to implement FAT file system to
/ small embedded systems. This is a free software and is opened for education,
/ research and commercial developments under license policy of following trems.
/
/  Copyright (C) 2014, ChaN, all right reserved.
/
/ * The Petit FatFs module is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial use UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/----------------------------------------------------------------------------*/

#ifndef _PFATFS
#define _PFATFS 4004 /* Revision ID */

#ifdef __cplusplus
extern "C" {
#endif

#include "pffconf.h"
#include <stdint.h>

#if _PFATFS != _PFFCONF
#error Wrong configuration file (pffconf.h).
#endif

#if _FS_FAT32
#define CLUST uint32_t
#else
#define CLUST uint16_t
#endif

/* File system object structure */

typedef struct {
	uint8_t  fs_type; /* FAT sub type */
	uint8_t  flag;    /* File status flags */
	uint8_t  csize;   /* Number of sectors per cluster */
	uint8_t  pad1;
	uint16_t  n_rootdir;  /* Number of root directory entries (0 on FAT32) */
	CLUST n_fatent;   /* Number of FAT entries (= number of clusters + 2) */
	uint32_t fatbase;    /* FAT start sector */
	uint32_t dirbase;    /* Root directory start sector (Cluster# on FAT32) */
	uint32_t database;   /* Data start sector */
	uint32_t fptr;       /* File R/W pointer */
	uint32_t fsize;      /* File size */
	CLUST org_clust;  /* File start cluster */
	CLUST curr_clust; /* File current cluster */
	uint32_t dsect;      /* File current data sector */
} FATFS;

/* Directory object structure */

typedef struct {
	uint16_t  index;  /* Current read/write index number */
	uint8_t *fn;     /* Pointer to the SFN (in/out) {file[8],ext[3],status[1]} */
	CLUST sclust; /* Table start cluster (0:Static table) */
	CLUST clust;  /* Current cluster */
	uint32_t sect;   /* Current sector */
} DIR;

/* File status structure */

typedef struct {
	uint32_t fsize;     /* File size */
	uint16_t  fdate;     /* Last modified date */
	uint16_t  ftime;     /* Last modified time */
	uint8_t  fattrib;   /* Attribute */
	char  fname[13]; /* File name */
} FILINFO;

/* File function return code (FRESULT) */

typedef enum {
	FR_OK = 0,       /* 0 */
	FR_DISK_ERR,     /* 1 */
	FR_NOT_READY,    /* 2 */
	FR_NO_FILE,      /* 3 */
	FR_NOT_OPENED,   /* 4 */
	FR_NOT_ENABLED,  /* 5 */
	FR_NO_FILESYSTEM /* 6 */
} FRESULT;

/*--------------------------------------------------------------*/
/* Petit FatFs module application interface                     */

FRESULT pf_mount(FATFS *fs);                            /* Mount/Unmount a logical drive */
FRESULT pf_open(const char *path);                      /* Open a file */
FRESULT pf_read(void *buff, uint16_t btr, uint16_t *br);        /* Read data from the open file */
FRESULT pf_write(const void *buff, uint16_t btw, uint16_t *bw); /* Write data to the open file */
FRESULT pf_lseek(uint32_t ofs);                            /* Move file pointer of the open file */
FRESULT pf_opendir(DIR *dj, const char *path);          /* Open a directory */
FRESULT pf_readdir(DIR *dj, FILINFO *fno);              /* Read a directory item from the open directory */

/*--------------------------------------------------------------*/
/* Flags and offset address                                     */

/* File status flag (FATFS.flag) */

#define FA_OPENED 0x01
#define FA_WPRT 0x02
#define FA__WIP 0x40

/* FAT sub type (FATFS.fs_type) */

// #define FS_FAT12 1
#define FS_FAT16 2
#define FS_FAT32 3

/* File attribute bits for directory entry */

#define ATTRIB_RDO 0x01  /* Read only */
#define ATTRIB_HID 0x02  /* Hidden */
#define ATTRIB_SYS 0x04  /* System */
#define ATTRIB_VOL 0x08  /* Volume label */
#define ATTRIB_LFN 0x0F  /* LFN entry */
#define ATTRIB_DIR 0x10  /* Directory */
#define ATTRIB_ARC 0x20  /* Archive */
#define ATTRIB_MASK 0x3F /* Mask of defined bits */

/*--------------------------------*/
/* Multi-byte word access macros  */

// Endian conversions
// Swap 2 byte, 16 bit values:

#define Swap2Bytes(val) \
( (((val) >> 8) & 0x00FF) | (((val) << 8) & 0xFF00) )

// Swap 4 byte, 32 bit values:

#define Swap4Bytes(val) \
( (((val) >> 24) & 0x000000FF) | (((val) >>  8) & 0x0000FF00) | \
(((val) <<  8) & 0x00FF0000) | (((val) << 24) & 0xFF000000) )

// Swap 8 byte, 64 bit values:

#define Swap8Bytes(val) \
( (((val) >> 56) & 0x00000000000000FF) | (((val) >> 40) & 0x000000000000FF00) | \
(((val) >> 24) & 0x0000000000FF0000) | (((val) >>  8) & 0x00000000FF000000) | \
(((val) <<  8) & 0x000000FF00000000) | (((val) << 24) & 0x0000FF0000000000) | \
(((val) << 40) & 0x00FF000000000000) | (((val) << 56) & 0xFF00000000000000) )
#endif

#if _WORD_ACCESS == 1 /* Enable word access to the FAT structure */
#define LD_WORD(ptr) (uint16_t)(*(uint16_t *)(uint8_t *)(ptr))
#define LD_DWORD(ptr) (uint32_t)(*(uint32_t *)(uint8_t *)(ptr))
#define ST_WORD(ptr, val) *(uint16_t *)(uint8_t *)(ptr)   = (uint16_t)(val)
#define ST_DWORD(ptr, val) *(uint32_t *)(uint8_t *)(ptr) = (uint32_t)(val)
#else /* Use byte-by-byte access to the FAT structure */
#define LD_WORD(ptr) (uint16_t)(((uint16_t) * ((uint8_t *)(ptr) + 1) << 8) | (uint16_t) * (uint8_t *)(ptr))
#define LD_DWORD(ptr)                                                                                                  \
	(uint32_t)(((uint32_t) * ((uint8_t *)(ptr) + 3) << 24) | ((uint32_t) * ((uint8_t *)(ptr) + 2) << 16)                              \
	        | ((uint16_t) * ((uint8_t *)(ptr) + 1) << 8)                                                                      \
	        | *(uint8_t *)(ptr))
#define ST_WORD(ptr, val)                                                                                              \
	*(uint8_t *)(ptr)       = (uint8_t)(val);                                                                                \
	*((uint8_t *)(ptr) + 1) = (uint8_t)((uint16_t)(val) >> 8)
#define ST_DWORD(ptr, val)                                                                                             \
	*(uint8_t *)(ptr)       = (uint8_t)(val);                                                                                \
	*((uint8_t *)(ptr) + 1) = (uint8_t)((uint16_t)(val) >> 8);                                                                   \
	*((uint8_t *)(ptr) + 2) = (uint8_t)((uint32_t)(val) >> 16);                                                                 \
	*((uint8_t *)(ptr) + 3) = (uint8_t)((uint32_t)(val) >> 24)
#endif

#ifdef __cplusplus
}

#endif /* _PFATFS */
