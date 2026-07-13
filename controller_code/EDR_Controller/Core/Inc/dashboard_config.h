/*
 * dashboard_config.h
 *
 * Replaces the Python dashboard's JSON-file persistence (five_bar_dashboard
 * _config.json) with a struct written to internal flash. Mirrors exactly
 * what got saved there: link geometry, axis gear/offset/direction, home
 * reference angle, trajectory limits.
 *
 * STM32H7 flash notes:
 *  - Program/erase happens in 256-bit (32-byte) "flash words" - writes must
 *    be a multiple of that, hence the padding in FiveBarConfigBlob.
 *  - You MUST point FIVEBAR_CONFIG_FLASH_ADDR at a sector your linker
 *    script does NOT place code/data in. Check your .ld file's FLASH
 *    region and carve out the last sector (or a dedicated one) for this.
 *    The value below is a placeholder - update it for your memory map.
 *  - Erase is sector-granular (128 KB on most H7 parts), so every save
 *    erases and rewrites the whole sector. Don't call this from a fast
 *    loop; it's meant for "Apply Config" style user actions, same as the
 *    Python version.
 *  - This uses HAL_FLASH_* directly (blocking, interrupts stay enabled
 *    unless you disable them yourself). If you're on FreeRTOS, take a
 *    mutex around save/load if any other task could be doing a flash
 *    operation concurrently - the H7 flash controller does not like
 *    concurrent access from both cores if you're running the dual-core
 *    variant.
 */
#ifndef DASHBOARD_CONFIG_H
#define DASHBOARD_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "five_bar_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TODO: set this to an address inside a sector reserved for config storage
 * in YOUR linker script / memory map. This placeholder assumes bank 1,
 * last 128KB sector on a 1MB-flash H750 (sector 7 @ 0x080E0000) - verify
 * against your actual part/linker script before relying on it. */
#define FIVEBAR_CONFIG_FLASH_ADDR   0x080E0000UL
#define FIVEBAR_CONFIG_FLASH_SECTOR FLASH_SECTOR_7
#define FIVEBAR_CONFIG_FLASH_BANK   FLASH_BANK_1

typedef struct {
    uint32_t magic;       /* validity marker, see .c for value */
    uint32_t version;     /* bump if you change this struct's layout */

    FiveBarParams params;
    AxisConfig axis_cfg[2];
    TrajConfig traj_cfg;
    float home_angle_deg;

    uint32_t checksum;     /* simple additive checksum over the fields above */
} FiveBarConfigBlob;
/* Actual flash writes round sizeof(FiveBarConfigBlob) up to a 32-byte
 * boundary internally (see dashboard_config.c) - you don't need to hand-pad
 * this struct yourself, and it's fine to add/remove fields (bump `version`
 * when you do, since old blobs won't match the new checksum/layout). */

/* Loads config from flash into *out. Returns true and fills *out if a valid
 * (magic+checksum match) blob was found; returns false (leaving *out
 * untouched) if flash at the config address is blank/corrupt - caller
 * should fall back to compiled-in defaults in that case, same as the
 * Python version falling back to hardcoded defaults when no JSON file
 * exists yet. */
bool fivebar_config_load(FiveBarConfigBlob *out);

/* Erases the config sector and writes *cfg. Returns true on success.
 * Blocking; takes on the order of tens to hundreds of ms depending on part/
 * clock (sector erase dominates). */
bool fivebar_config_save(const FiveBarConfigBlob *cfg);

#ifdef __cplusplus
}
#endif

#endif /* DASHBOARD_CONFIG_H */
