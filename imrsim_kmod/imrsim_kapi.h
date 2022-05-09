#ifndef _IMRSIM_KAPI_H
#define _IMRSIM_KAPI_H

/*
 * IMRSIM_GET_LAST_WERROR
 *
 * Get the error result of the last write function call.
 *
 * Returns 0 if operation is successful, negative otherwise. 
 *
 */
int imrsim_get_last_wd_error(__u32 *last_error);

/*
 * IMRSIM_GET_LAST_RERROR
 *
 * Get the error result of the last read function call.
 *
 * Returns 0 if operation is successful, negative otherwise. 
 *
 */
int imrsim_get_last_rd_error(__u32 *last_error);

/*
 * IMRSIM_SET_LOGENABLE
 *
 * Turn off or on logging; 0 turns it off, non-zero enables
 *
 * Returns 0 if operation is successful, negative otherwise. 
 *
 */
int imrsim_set_log_enable(__u32 zero_is_disable);

/*
 * IMRSIM_GET_NUMZONES
 *
 * Get number of zones configured on device.
 *
 * Returns 0 if operation is successful, negative otherwise. 
 *
 */
int imrsim_get_num_zones(__u32 *num_zones);

/*
 * IMRSIM_GET_SIZZONEDEFAULT
 *
 * Get default size of zones configured.
 *
 * Returns 0 if operation is successful, negative otherwise.
 *
 */
int imrsim_get_size_zone_default(__u32 *siz_zone);

/*
 * IMRSIM_SET_SIZZONEDEFAULT
 *
 * Set size of zones configured.
 *
 * Returns 0 if operation is successful, negative otherwise.
 *
 */
int imrsim_set_size_zone_default(__u32 siz_zone);

/*
 * IMRSIM_RESET_ZONE
 *
 * Reset the write pointer for a sequential write zone.
 *
 */
int imrsim_blkdev_reset_zone_ptr(sector_t start_sector);

/*
 * IMRSIM_QUERY
 *
 * Query imrsim zone list that matches the criteria specified by
 * free_sectors_criteria.  Matched zone data for at most max_zones will
 * be placed into the user supplied memory ret_zones in ascending LBA order.
 * The return value will be a kernel error code if negative, or the number 
 * of zones actually returned if there is no error.
 *
 * If free_sectors_criteria is positive, then return zones that have
 * at least that many sectors available to be written.  If it is zero,
 * then match all zones.  If free_sectors_criteria is negative, then
 * return the zones that match the following criteria:
 *
 * -1 ZONE_MATCH_FULL  Match all full zones
 * -2 ZONE_MATCH_NFULL  Match all zones where each zone isn't a full zone.
 *   (the zone has at least one written sector and is not full - zone
 *   condition internally is in CLOSED state according to ZACr08.)
 *
 * -3 ZONE_MATCH_FREE  Match all free zones
 *    (the zone has no written sectors)
 *
 * -4 ZONE_MATCH_RNLY  Match all read-only zones
 * -5 ZONE_MATCH_OFFL  Match all offline zones
 * -6 ZONE_MATCH_WNEC  Match all zones where the write ptr != the checkpoint ptr
 *
 * The negative values are taken from Table 4 of 14-010r1, with the
 * exception of -6, which is not in the draft spec --- but IMHO should
 * be :-) It is anticipated, though, that the kernel will keep this
 * info in in memory and so will handle matching zones which meet
 * these criteria itself, without needing to issue a ZBC command for
 * each call to blkdev_query_zones().
 */
int imrsim_query_zones(sector_t start_sector,
                       int free_sectors_criteria, 
                       __u32 *max_zones,
                       struct imrsim_zone_status *ret_zones);


/*
 * IMRSIM_GET_STATS
 *
 * Get IMRSIM stats values.
 *
 * Returns 0 if operation is successful, negative otherwise.
 *
 */
int imrsim_get_stats(struct imrsim_stats *stats);

/*
 * IMRSIM_RESET_STATS
 *
 * Resets IMRSIM stats values to default (0).
 *
 * Returns 0 if operation is successful, negative otherwise.
 *
 */
int imrsim_reset_stats(void);

/*
 * IMRSIM_RESET_ZONESTATS
 *
 * Resets IMRSIM stats of a zone to default values (0).
 *
 * Returns 0 if operation is successful, -EINVAL if the start_sector is
 * not the beginning of a sequential write zone.
 *
 */
int imrsim_reset_zone_stats(sector_t start_sector);

/*
 * IMRSIM_GET_DEVSTATS
 *
 * Get IMRSIM device stats.
 *
 * Returns 0 if operation is successful, negative otherwise.
 *
 */
int imrsim_get_device_stats(struct imrsim_dev_stats *device_stats);
/*
 * IMRSIM_GET_ZONESTATS
 *
 * Get IMRSIM stats of a zone.
 *
 * Returns 0 if operation is successful, negative otherwise.
 *
 * Returns -EINVAL if the start_sector is not the beginning of a
 * sequential write zone.
 *
 */
int imrsim_get_zone_stats(sector_t start_sector,
                          struct imrsim_zone_stats *zone_stats);
/*
 * IMRSIM_RESET_DEFAULTCONFIG
 *
 * Reset IMRSIM config (zone and device) values to default.
 *
 * Returns 0 if operation is successful, negative otherwise.
 *
 */
int imrsim_reset_default_config(void);

/*
 *
 * IMRSIM_RESET_ZONECONFIG
 *
 * Reset IMRSIM zone configuration to default (divide LBA space into zones
 * with the first and last zone CMR, the rest SMR)
 *
 * Returns 0 if operation is successful, negative otherwise.
 *
 */
int imrsim_reset_default_zone_config(void);

/*
 * IMRSIM_RESET_DEVCONFIG
 *
 * Reset IMRSIM device configuration to default.
 *
 * Returns 0 if operation is successful, negative otherwise.
 *
 */
int imrsim_reset_default_device_config(void);

/*
 * IMRSIM_GET_DEVCONFIG
 *
 * Get IMRSIM device config values.
 *
 * Returns 0 if operation is successful, negative otherwise.
 *
 */
int imrsim_get_device_config(struct imrsim_dev_config *device_config);

/*
 * IMRSIM_SET_DEVRCONFIG
 *
 * Set read IMRSIM device config values.
 *
 * Returns 0 if operation is successful, negative otherwise.
 *
 */
int imrsim_set_device_rconfig(struct imrsim_dev_config *device_config);

/*
 * IMRSIM_SET_DEVWCONFIG
 *
 * Set write IMRSIM device config values.
 *
 * Returns 0 if operation is successful, negative otherwise.
 *
 */
int imrsim_set_device_wconfig(struct imrsim_dev_config *device_config);

/*
 * IMRSIM_SET_DEVRCONFIG_DELAY
 *
 * Set read IMRSIM device config values.
 *
 * Returns 0 if operation is successful, negative otherwise.
 *
 */
int imrsim_set_device_rconfig_delay(struct imrsim_dev_config *device_config);

/*
 * IMRSIM_SET_DEVWCONFIG_DELAY
 *
 * Set write IMRSIM device config values.
 *
 * Returns 0 if operation is successful, negative otherwise.
 *
 */
int imrsim_set_device_wconfig_delay(struct imrsim_dev_config *device_config);

/*
 * IMRSIM_CLEAR_ZONECONFIG
 *
 * Clear all zones from IMRSIM (number of zones will be 0 on completion of this
 *  function).
 *
 * Returns 0 if operation is successful, negative otherwise.
 */
int imrsim_clear_zone_config(void);

/*
 * IMRSIM_ADD_ZONECONFIG
 *
 * Add one more zone starting at the end of last zone configuration in LBA space.
 *
 * Error if zone_config does not start at the end of current last zone config.
 * Error if zone_zonfig goes beyond device LBA space.
 * Error if zone type is not allowed by ZBC/ZAC spec.
 *
 * Returns 0 if operation is successful, negative otherwise.
 *
 */
//int imrsim_add_zone_config(struct imrsim_zone_status *zone_status);

/*
 * IMRSIM_MODIFY_ZONECONFIG
 *
 * Modify selected zone.
 *
 * Error if zone config does not exist or if zone type is not allowed by
 * ZBC/ZAC spec.
 *
 * Returns 0 if operation is successful, negative otherwise.
 *
 */
//int imrsim_modify_zone_config(struct imrsim_zone_status *zone_status);


#endif