#ifndef _IMRSIM_TYPES_H
#define _IMRSIM_TYPES_H

#define IMRSIM_VERSION(a,b,c)  ((a<<16)|(b<<8)|c)
#define TOP_TRACK_NUM_TOTAL 64
#define TOP_TRACK_SIZE 456
#define BOTTOM_TRACK_SIZE 568

// 映射表项的总个数
#define TOTAL_ITEMS (TOP_TRACK_SIZE+BOTTOM_TRACK_SIZE)*TOP_TRACK_NUM_TOTAL

enum imrsim_zone_conditions{
    Z_COND_NO_WP      = 0x00,
    Z_COND_EMPTY      = 0x01,
    Z_COND_CLOSED     = 0x02,
    Z_COND_RO         = 0x0D,     /* read only */
    Z_COND_FULL       = 0x0E,
    Z_COND_OFFLINE    = 0x0F
};

enum imrsim_zone_type{
    Z_TYPE_RESERVED     = 0x00,
    Z_TYPE_CONVENTIONAL = 0x01,
    Z_TYPE_SEQUENTIAL   = 0x02,
    Z_TYPE_PREFERRED    = 0x04
};

struct imrsim_zone_track{
    __u8     isUsedBlock[TOP_TRACK_SIZE];
};

struct imrsim_zone_status
{
    sector_t                     z_start;                /* blocks  */
    __u32                        z_length;               /* sectors */
    __u16                        z_conds;
    __u8                         z_type;
    __u8                         z_flag;
    struct imrsim_zone_track     z_tracks[TOP_TRACK_NUM_TOTAL];  /* 仅保存一个zone中所有顶部磁道的记录，是否有数据 */
    //映射表
    __u32                        z_map_size;
    int                          z_pba_map[TOTAL_ITEMS];
};

struct imrsim_state_header
{
    __u32  magic;
    __u32  length;
    __u32  version;
    __u32  crc32;
};

struct imrsim_idle_stats
{
    __u32 dev_idle_time_max;
    __u32 dev_idle_time_min;
};

struct imrsim_dev_stats
{
    struct imrsim_idle_stats idle_stats;
};

struct imrsim_out_of_policy_read_stats
{
    __u32 span_zones_count;
};

struct imrsim_out_of_policy_write_stats
{
    __u32 span_zones_count;
    __u32 unaligned_count;
};

struct imrsim_zone_stats
{
    struct imrsim_out_of_policy_read_stats   out_of_policy_read_stats;
    struct imrsim_out_of_policy_write_stats  out_of_policy_write_stats;
    __u32 rewrite_total;      // 记录重写次数
    __u32 rewrite_single;     // 记录单边重写次数
};

struct imrsim_stats
{
    struct imrsim_dev_stats  dev_stats;
    __u32                    num_zones;
    struct imrsim_zone_stats zone_stats[1];           
};

struct imrsim_dev_config
{
    /* flag: 0 to reject with erro, 1 to add latency and satisfy request. */
    __u32 out_of_policy_read_flag;
    __u32 out_of_policy_write_flag;
    __u16 r_time_to_rmw_zone;    /* read time */
    __u16 w_time_to_rmw_zone;    /* write time */
};

struct imrsim_config
{
    struct imrsim_dev_config  dev_config;
};

struct imrsim_state
{
    struct imrsim_state_header header;
    struct imrsim_config       config;
    struct imrsim_stats        stats;
};


typedef struct
{
  __u64                      lba;          /* IN            */
  __u32                      num_zones;    /* IN/OUT        */  // 要查询的zone的个数
  int                        criteria;     /* IN            */
  struct imrsim_zone_status  ptr[1];       /* OUT           */
} imrsim_zbc_query;


enum imrsim_zbcquery_criteria {
   ZONE_MATCH_ALL  =           0, /* Match all zones                       */
   ZONE_MATCH_FULL =          -1, /* Match all zones full                  */
   ZONE_MATCH_NFULL =         -2, /* Match all zones not full              */
   ZONE_MATCH_FREE =          -3, /* Match all zones free                  */
   ZONE_MATCH_RNLY =          -4, /* Match all zones read-only             */
   ZONE_MATCH_OFFL =          -5, /* Match all zones offline               */
};

#endif