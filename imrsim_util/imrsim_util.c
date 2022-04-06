#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/types.h>

#define   u8       __u8
#define   u32      __u32
#define   u64      __u64
#define   sector_t __u64

#include "imrsim_types.h"
#include "imrsim_ioctl.h"

/*
 * Hold the retrieved zone status - user applications need to keep sync with
 * imrsim device with the most recent update for its accuracy. This is an 
 * example usage only.
 */
imrsim_zbc_query  *zbc_query_cache;


int imrsim_util_print_help()
{
    printf("\nimrsim_util error: Invalid command line!\n\nHELP:\n\n");
    printf("./imrsim_util /dev/mapper/<dm_device_name> <code> <seq> <arg>\n");
    printf("\nExample:\n\n");
    printf("Show last read error     : imrsim_util /dev/mapper/imrsim e 1\n");
    printf("Show last write error    : imrsim_util /dev/mapper/imrsim e 2\n");
    printf("Enable logging           : imrsim_util /dev/mapper/imrsim e 3\n");
    printf("Disable logging          : imrsim_util /dev/mapper/imrsim e 4\n");
    printf("\n");
    printf("Get number of zones      : imrsim_util /dev/mapper/imrsim z 1\n");
    printf("Get default zone size    : imrsim_util /dev/mapper/imrsim z 2\n");
    printf("Set default zone size    : imrsim_util /dev/mapper/imrsim z 3 <size_in_sectors>\n");
    printf("\n");
    printf("reset zone status    : imrsim_util /dev/mapper/imrsim z 4 <lba>\n");
    printf("reset zone status    : imrsim_util /dev/mapper/imrsim z 5 <zone_index>\n");
    printf("\n");
    printf("query zone status    : imrsim_util /dev/mapper/imrsim z 6 <number_of_zones>\n");
    printf("query zone status    : imrsim_util /dev/mapper/imrsim z 7 <lba>\n");
    printf("query zone status    : imrsim_util /dev/mapper/imrsim z 8 <zone_index>\n");
    printf("\n"); 
    printf("Get all zone stats       : imrsim_util /dev/mapper/imrsim s 1\n");
    printf("Get zone stats           : imrsim_util /dev/mapper/imrsim s 2 <number_of_zones>\n");
    printf("Get zone stats by idx    : imrsim_util /dev/mapper/imrsim s 3 <zone_index>\n");
    printf("Reset all zone stats     : imrsim_util /dev/mapper/imrsim s 4\n");
    printf("Reset zone stats by lba  : imrsim_util /dev/mapper/imrsim s 5 <lba>\n");
    printf("Reset zone stats by idx  : imrsim_util /dev/mapper/imrsim s 6 <zone_index>\n");
    printf("\n");
    printf("Set all default config   : imrsim_util /dev/mapper/imrsim l 1\n");
    printf("Set zone default config  : imrsim_util /dev/mapper/imrsim l 2\n");
    printf("Reset dev default config : imrsim_util /dev/mapper/imrsim l 3\n");
    printf("Get dev config           : imrsim_util /dev/mapper/imrsim l 4\n");
    printf("Set Read penalty delay   : imrsim_util /dev/mapper/imrsim l 5 <number_seconds>\n");
    printf("Set Write penalty delay  : imrsim_util /dev/mapper/imrsim l 6 <number_seconds>\n");
    printf("\n");
    printf("\n\n");

    return 0;
}

void imrsim_err_iot(int fd, int seq)
{
    int num = 0;

    switch(seq)
    {
        case 1:
            if (!ioctl(fd, IOCTL_IMRSIM_GET_LAST_RERROR, &num)) {
                printf("Last read error: %d\n", num);
            } else {
                printf("Operation failed\n");
            }
            break;
        case 2:
            if (!ioctl(fd, IOCTL_IMRSIM_GET_LAST_WERROR, &num)) {
                printf("Last write error: %d\n", num);
            } else {
                printf("Operation failed\n");
            }
            break;
        case 3:
            if (!ioctl(fd, IOCTL_IMRSIM_SET_LOGENABLE)) {
                printf("Log enabled\n");
            } else {
                printf("Operation failed\n");
            }
            break;
        case 4:
            if (!ioctl(fd, IOCTL_IMRSIM_SET_LOGDISABLE)) {
                printf("Log disabled\n");
            } else {
                printf("Operation failed\n");
            }
            break;
        default:
            printf("ioctl error: Invalid command\n");
    }
}

void imrsim_report_zbc_query(imrsim_zbc_query *zbc_query)
{
    int i = 0;

    if (!zbc_query) {
        printf("zbc query null pointer\n");
        return;
    }
    if (zbc_query->num_zones == 0) {
        printf("Query result: no zone matched the query condition\n");
        return;   
    }
    printf("zbc query starting lba     : %llu\n", zbc_query->lba);
    printf("zbc query starting criteria: %d\n", zbc_query->criteria);
    printf("zbc query num of zones     : %u\n", zbc_query->num_zones);
    printf("\n");
    printf("zone status contents retrived as the following:\n");
    printf("\n");
    for (i = 0; i < zbc_query->num_zones; i++) {
        printf("zone index        : %llu\n", zbc_query->ptr[i].z_start);
        printf("zone length       : %u\n", zbc_query->ptr[i].z_length);
        printf("zone type         : 0x%x\n", zbc_query->ptr[i].z_type);
        switch (zbc_query->ptr[i].z_conds) {
            case Z_COND_NO_WP:
            printf("zone condition    : ZONE NO WP\n");
            break;
            case Z_COND_EMPTY:
            printf("zone condition    : ZONE EMPTY\n");       
            break;
            case Z_COND_RO:
            printf("zone condition    : ZONE READ ONLY\n");
            break;
            case Z_COND_CLOSED:
            printf("zone condition    : ZONE CLOSED\n");
            break;
            case Z_COND_FULL:
            printf("zone condition    : ZONE FULL\n");
            break;
            case Z_COND_OFFLINE:
            printf("zone condition    : ZONE OFFLINE\n");
            break;
            default:
            break;
        }
        printf("zone control      : 0x%x\n", zbc_query->ptr[i].z_flag);
        printf("zone map_size     : 0x%u\n", zbc_query->ptr[i].z_map_size);
        printf("\n");
   }
}

void imrsim_zone_iot(int fd, int seq, char *argv[])
{
   u32 num32     = 0;      // zone_idx
   u32 num_zones = 0;      // how many zones need to query
   u64 num64     = 0;      // sectors in zone
   u64 lba       = 0;
   u8  num8      = 0;        
   imrsim_zbc_query  *zbc_query;

   switch(seq)
   {
    case 1:
        if (!ioctl(fd, IOCTL_IMRSIM_GET_NUMZONES, &num32)) {
        printf("Number of zones: %u\n", num32);
        } else {
        printf("operation failed\n");
        }
        break;
    case 2:
        if (!ioctl(fd, IOCTL_IMRSIM_GET_SIZZONEDEFAULT, &num32)) {
        printf("Get zone default size: %u\n", num32);
        } else {
            printf("operation failed\n");
        }
        break;
    case 3:
        if (argv[4] == NULL) {
            imrsim_util_print_help();
            break;
        }
        num32 = atoi(argv[4]);
        if (!ioctl(fd, IOCTL_IMRSIM_SET_SIZZONEDEFAULT, &num32)) {
        printf("Set zone default size: %u\n", num32);
        } else {
            printf("operation failed\n");
        }
        break;      
    case 4: 
        if (argv[4] == NULL) {
            imrsim_util_print_help();
            break;
        }
        num64 = atol(argv[4]);
        if (!ioctl(fd, IOCTL_IMRSIM_RESET_ZONE, &num64)) {
        printf("Reset zone: %llu\n", num64);
        } else {
            printf("operation failed\n");
        }
        break;
    case 5: 
        if (argv[4] == NULL) {
            imrsim_util_print_help();
            break;
        }
        if (ioctl(fd, IOCTL_IMRSIM_GET_NUMZONES, &num_zones)) {
        printf("Unable to obtain zone number. Operation failed\n");
        }
        num32 = atoi(argv[4]);
        if (num32 >= num_zones) {
            printf("Zone index out of range\n");
            break;
        }         
        if (ioctl(fd, IOCTL_IMRSIM_GET_SIZZONEDEFAULT, &num64)) {
            printf("Cannot get zone size. Operation failed.\n");
            break;
        }
        num64 = num32 * num64;
        if (!ioctl(fd, IOCTL_IMRSIM_RESET_ZONE, &num64)) {
        printf("Reset zone: %u\n", num32);
        } else {
            printf("Operation failed\n");
        }
        break;
    case 6: 
    case 7:
        if (ioctl(fd, IOCTL_IMRSIM_GET_NUMZONES, &num_zones)) {
            printf("Unable to get number of zones. operation failed\n");
            break;
        }
        if (ioctl(fd, IOCTL_IMRSIM_GET_SIZZONEDEFAULT, &num64)) {
            printf("Cannot get zone size. Operation failed.\n");
            break;
        }
        if (argv[4] == NULL) {
            imrsim_util_print_help();
            break;
        }
        if (seq == 6) {  
        lba = 0;
        num32 = atoi(argv[4]);
        if (num32 == 0 || num32 > num_zones) {
            printf("The number of zones is out of boundary\n");
            imrsim_util_print_help();
            break;
        }
        } else {
        num32 = 1;
        lba = atol(argv[4]);
        if (lba >= num64 * num_zones) {
            printf("LBA is out of boundary.\n");
            imrsim_util_print_help();
            break;
        }
        }
        zbc_query = malloc(sizeof(imrsim_zbc_query) + (num32 - 1)
                    * sizeof(struct imrsim_zone_status));
        if (!zbc_query) {
            printf("No enough memory to continue\n");
            break;
        }
        /*
        * ZONE_MATCH_ALL
        */
        printf("\nMatch all:\n");
        zbc_query->criteria =  ZONE_MATCH_ALL; 
        zbc_query->lba = lba;     
        zbc_query->num_zones = num32;
        if (!ioctl(fd, IOCTL_IMRSIM_QUERY, zbc_query)) {
        imrsim_report_zbc_query(zbc_query);
        } else {
            printf("Operation failed\n");
        }
        /*
        * ZONE_MATCH_FULL
        */
        printf("\nMatch full:\n");
        zbc_query->criteria =  ZONE_MATCH_FULL; 
        zbc_query->lba = lba;     
        zbc_query->num_zones = num32;
        if (!ioctl(fd, IOCTL_IMRSIM_QUERY, zbc_query)) {
        imrsim_report_zbc_query(zbc_query);
        } else {
            printf("Operation failed\n");
        }
        /*
        * ZONE_MATCH_NFULL
        */
        printf("\nMatch to non-full zones:\n");
        zbc_query->criteria =  ZONE_MATCH_NFULL; 
        zbc_query->lba = lba;     
        zbc_query->num_zones = num32;
        if (!ioctl(fd, IOCTL_IMRSIM_QUERY, zbc_query)) {
        imrsim_report_zbc_query(zbc_query);
        } else {
        printf("Operation failed\n");
        }
        /*
        * ZONE_MATCH_FREE
        */
        printf("\nMatch free Sequential Zones:\n");
        zbc_query->criteria =  ZONE_MATCH_FREE; 
        zbc_query->lba = lba;     
        zbc_query->num_zones = num32;
        if (!ioctl(fd, IOCTL_IMRSIM_QUERY, zbc_query)) {
        imrsim_report_zbc_query(zbc_query);
        } else {
            printf("Operation failed\n");
        }
        /*
        * ZONE_MATCH_RNLY
        */
        printf("\nMatch read only:\n");
        zbc_query->criteria =  ZONE_MATCH_RNLY; 
        zbc_query->lba = lba;     
        zbc_query->num_zones = num32;
        if (!ioctl(fd, IOCTL_IMRSIM_QUERY, zbc_query)) {
        imrsim_report_zbc_query(zbc_query);
        } else {
            printf("Operation failed\n");
        }

        /*
        * ZONE_MATCH_OFFL
        */
        printf("\nMatch offline:\n");
        zbc_query->criteria =  ZONE_MATCH_OFFL; 
        zbc_query->lba = lba;     
        zbc_query->num_zones = num32;
        if (!ioctl(fd, IOCTL_IMRSIM_QUERY, zbc_query)) {
        imrsim_report_zbc_query(zbc_query);
        } else {
            printf("Operation failed\n");
        }
    case 8:
        if (ioctl(fd, IOCTL_IMRSIM_GET_NUMZONES, &num_zones)) {
            printf("Unable to get number of zones. Operation failed.\n");
            break;
        }
        if (ioctl(fd, IOCTL_IMRSIM_GET_SIZZONEDEFAULT, &num64)) {
            printf("Cannot get zone size. Operation failed.\n");
            break;
        }
        if (argv[4] == NULL) {
            imrsim_util_print_help();
            break;
        }
        num32 = atoi(argv[4]);
        if (num32 >= num_zones) {
        printf("The zone index is out of boundary\n");
        imrsim_util_print_help();
        break;
        }
        zbc_query = malloc(sizeof(imrsim_zbc_query));
        if (!zbc_query) {
            printf("No enough memory to continue.\n");
            break;
        }
        /*
        * ZONE_MATCH_ALL
        */
        printf("\nMatch all:\n");
        zbc_query->criteria =  ZONE_MATCH_ALL; 
        zbc_query->lba = num32 * num64;     
        zbc_query->num_zones = 1;
        if (!ioctl(fd, IOCTL_IMRSIM_QUERY, zbc_query)) {
        imrsim_report_zbc_query(zbc_query);
        } else {
            printf("Operation failed\n");
        }
        break; 
    default:
        printf("ioctl error: Invalid command.\n");
   }
}

void imrsim_report_zone_stats(struct imrsim_stats  *stats, u32 idx)
{
    if (!stats) {
        return;   
    }
    printf("Device idle time max: %u\n",
            stats->dev_stats.idle_stats.dev_idle_time_max);
    printf("Device idle time min: %u\n",
            stats->dev_stats.idle_stats.dev_idle_time_min);
    printf("zone[%u] imrsim out of policy read stats: span zones count: %u\n",
            idx, stats->zone_stats[idx].out_of_policy_read_stats.span_zones_count);
    printf("zone[%u] imrsim out of policy write stats: span zones count: %u\n",
            idx, stats->zone_stats[idx].out_of_policy_write_stats.span_zones_count);
    printf("zone[%u] imrsim out of policy write stats: unaligned count: %u\n",
            idx, stats->zone_stats[idx].out_of_policy_write_stats.unaligned_count);
    printf("zone[%u] imrsim rewrite_total count: %u\n",
                        idx, stats->zone_stats[idx].rewrite_total);    
    printf("zone[%u] imrsim rewrite_single count: %u\n",
                    idx, stats->zone_stats[idx].rewrite_single);  
    printf("\n");
}

void imrsim_report_stats(struct imrsim_stats  *stats, u32 num32)
{
    u32 i = 0;
    printf("\nDevice idle time max: %u\n",
            stats->dev_stats.idle_stats.dev_idle_time_max);
    printf("Device idle time min: %u\n",
            stats->dev_stats.idle_stats.dev_idle_time_min);
   
    for (i = 0; i < num32; i++) {
        printf("zone[%u] imrsim out of policy read stats: span zones count: %u\n",
                    i, stats->zone_stats[i].out_of_policy_read_stats.span_zones_count);
        printf("zone[%u] imrsim out of policy write stats: span zones count: %u\n",
                    i, stats->zone_stats[i].out_of_policy_write_stats.span_zones_count);
        printf("zone[%u] imrsim out of policy write stats: unaligned count: %u\n",
                    i, stats->zone_stats[i].out_of_policy_write_stats.unaligned_count);
        printf("zone[%u] imrsim rewrite_total count: %u\n",
                        i, stats->zone_stats[i].rewrite_total);    
        printf("zone[%u] imrsim rewrite_single count: %u\n",
                    i, stats->zone_stats[i].rewrite_single);  
        printf("\n");
    }
}

void imrsim_stats_iot(int fd, int seq, char *argv[])
{
    struct imrsim_stats *stats;
    u32    num32     = 0;
    u32    num_zones = 0; 
    u64    num64     = 0;

    if (ioctl(fd, IOCTL_IMRSIM_GET_NUMZONES, &num_zones)) {
        printf("unable to get number of zones\n");
        return;
    }
    stats = (struct imrsim_stats *)malloc(sizeof(struct imrsim_dev_stats)
        + sizeof(u32) + sizeof(struct imrsim_zone_stats) * num_zones);
    if (!stats) {
        printf("No enough memory to continue.\n");
        return;
    }
    switch(seq)
    {
        case 1:
            if (!ioctl(fd, IOCTL_IMRSIM_GET_STATS, stats)) {
            printf("Get Stats:\n");
            imrsim_report_stats(stats, num_zones);
            } else {
            printf("Operation failed\n");
            }
            break;
        case 2:
            if (argv[4] == NULL) {
            imrsim_util_print_help();
            break;
            }
            num32 = atoi(argv[4]);
            if (num32 > num_zones) {
            printf("Too much zones specified\n");
            break;
            }
            if (!ioctl(fd, IOCTL_IMRSIM_GET_STATS, stats)) {
            printf("Get Stats:\n");
            imrsim_report_stats(stats, num32);
            } else {
            printf("Operation failed\n");
            }
            break;
        case 3:
            if (argv[4] == NULL) {
            imrsim_util_print_help();
            break;
            }
            num32 = atoi(argv[4]);
            if (num32 >= num_zones) {
            printf("Zone index out of range\n");
            break;
            }
            if (!ioctl(fd, IOCTL_IMRSIM_GET_STATS, stats)) {
            printf("Get Stats:\n");
            imrsim_report_zone_stats(stats, num32);
            } else {
            printf("Operation failed.\n");
            }
            break;  
        case 4:
            if (!ioctl(fd, IOCTL_IMRSIM_RESET_STATS)) {
            printf("Stats reset success\n");
            } else {
            printf("Operation failed\n");
            }
            break;
        case 5:
            if (argv[4] == NULL) {
            imrsim_util_print_help();
            break;
            }
            num64 = atol(argv[4]);
            if (!ioctl(fd, IOCTL_IMRSIM_RESET_ZONESTATS, &num64)) {
            printf("Zone stats reset Success\n");
            } else {
            printf("operation failed\n");
            }
            break;
        case 6:
            if (argv[4] == NULL) {
            imrsim_util_print_help();
            break;
            }
            num32 = atoi(argv[4]);
            if (num32 >= num_zones) {
            printf("zone index out of range\n");
            break;
            }
            if (ioctl(fd, IOCTL_IMRSIM_GET_SIZZONEDEFAULT, &num64)) {
            printf("Cannot get zone size. Operation failed.\n");
            break;
            }
            num64 = num32 * num64;
            if (!ioctl(fd, IOCTL_IMRSIM_RESET_ZONESTATS, &num64)) {
            printf("Zone stats reset Success\n");
            } else {
            printf("Operation failed\n");
            }
            break;
        default:
            printf("ioctl error: Invalid command.\n");
    }
    free(stats);
}

static void imrsim_report_devconf(struct imrsim_dev_config *dev_conf)
{
    printf("imrsim dev out of policy read flag    : %u\n", 
            dev_conf->out_of_policy_read_flag);
    printf("imrsim dev out of policy write flag   : %u\n", 
            dev_conf->out_of_policy_write_flag);
    printf("imrsim dev out of policy read penalty : %u microseconds\n",
            dev_conf->r_time_to_rmw_zone);
    printf("imrsim dev out of policy write penalty: %u microseconds\n",
            dev_conf->w_time_to_rmw_zone);
}

u32 imrsim_num_seq_zones(imrsim_zbc_query *zbc_query_cache)
{
    u32 index;
    u32 cnt = 0;

    if (!zbc_query_cache && !zbc_query_cache->num_zones) {
        printf("No zone exist yet. Start to create it from the begining.\n");
        return cnt;   
    }
    for (index = 0; index < zbc_query_cache->num_zones; index++) {
        if (zbc_query_cache->ptr[index].z_type == Z_TYPE_SEQUENTIAL) {
            cnt++;
        }
    }
    return cnt;
}

void imrsim_config_iot(int fd, int seq, char *argv[])
{
    u32    num32  = 0; 
    u64    num64  = 0;
    u32    size32 = 0;
    u32    n_zone = 0;
    int    option = 0;
    u32    s_cnt  = 0;
    u8     flag   = 0; 
    enum imrsim_zone_conditions cond;

    struct imrsim_dev_config  dev_conf;
    struct imrsim_zone_status zone_status;

    if (ioctl(fd, IOCTL_IMRSIM_GET_NUMZONES, &num32)) {
        printf("Unable to get number of zones.\n");
        return;
    }   
    switch(seq)
    {
        case 1:    
            if (!ioctl(fd, IOCTL_IMRSIM_RESET_DEFAULTCONFIG)) {
                printf("Set imrsim default config success\n");
            } else {
                printf("Operation failed.\n");
            }
            break;
        case 2:    
            if (!ioctl(fd, IOCTL_IMRSIM_RESET_ZONECONFIG)) {
                printf("Set imrsim default zone config success\n");
            } else {
                printf("Operation failed\n");
            }
            break;
        case 3:    
            if (!ioctl(fd, IOCTL_IMRSIM_RESET_DEVCONFIG)) {
                printf("Reset imrsim default dev config success\n");
            } else {
                printf("Operation failed\n");
            }
            break;
        case 4:
            memset(&dev_conf, 0, sizeof(struct imrsim_dev_config));
            if (!ioctl(fd, IOCTL_IMRSIM_GET_DEVCONFIG, &dev_conf)) {
                printf("Get dev config Success\n");
                imrsim_report_devconf(&dev_conf);
            } else {
                printf("Operation failed\n");
            }
            break;
        case 5:
            memset(&dev_conf, 0, sizeof(struct imrsim_dev_config));
            if (argv[4] == NULL) {
                imrsim_util_print_help();
                break;
            }
            num32 = atoi(argv[4]);           
            dev_conf.r_time_to_rmw_zone = num32 * 1000;
            if (!ioctl(fd, IOCTL_IMRSIM_SET_DEVRCONFIG_DELAY, &dev_conf)) {
                printf("Set dev config read penalty Success\n");
            } else {
                printf("Operation failed\n");
            }
            break;
        case 6:
            memset(&dev_conf, 0, sizeof(struct imrsim_dev_config));
            if (argv[4] == NULL) {
                imrsim_util_print_help();
                break;
            }
            num32 = atoi(argv[4]);       
            dev_conf.w_time_to_rmw_zone = num32 * 1000;
            if (!ioctl(fd, IOCTL_IMRSIM_SET_DEVWCONFIG_DELAY, &dev_conf)) {
                printf("Set dev config write penalty Success\n");
            } else {
                printf("Operation failed\n");
            }
            break;

        default:
            printf("ioctl error: Invalid command\n");
    }
}


int main(int argc, char* argv[])
{
    int   fd;           // file description: 0:success，-1:fail
    int   seq;
    char  code;

    if (4 != argc && 5 != argc) {
        return imrsim_util_print_help();
    }
    code = argv[2][0];
    seq = atoi(argv[3]);
    fd = open(argv[1], O_RDWR);      

    if (-1 == fd)
    {
        printf("Error: %s open failed\n", argv[1]);
        return -1;
    }
    switch (code) {
        case 'e':
            imrsim_err_iot(fd, seq);
            break;
        case 'z':
            imrsim_zone_iot(fd, seq, argv);
            break;
        case 's':
            imrsim_stats_iot(fd, seq, argv);
            break;
        case 'l':
            imrsim_config_iot(fd, seq, argv);
            break;
        default:
            return imrsim_util_print_help(); 
    } 
    close(fd);
    return 0;
}
