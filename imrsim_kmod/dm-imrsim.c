#include <linux/module.h>
#include <linux/init.h>
#include <linux/blkdev.h>
#include <linux/bio.h>
#include <linux/device-mapper.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/kthread.h>
#include <linux/crc32.h>
#include <linux/gfp.h>
#include <linux/mutex.h>
#include <linux/math64.h>
#include <linux/version.h>
#include <asm/ptrace.h>
#include "imrsim_types.h"
#include "imrsim_ioctl.h"
#include "imrsim_kapi.h"
#include "imrsim_zerror.h"

/*
    The kernel module in the Device Mapper framework is mainly responsible for building 
the target driver to build the disk structure and function.
*/

/* Some basic disk information */
#define IMR_ZONE_SIZE_SHIFT_DEFAULT      16      /* number of blocks/zone, e.g. 2^16=65536 */
#define IMR_BLOCK_SIZE_SHIFT_DEFAULT     3       /* number of sectors/block, 8   */
#define IMR_PAGE_SIZE_SHIFT_DEFAULT      3       /* number of sectors/page, 8    */
#define IMR_SECTOR_SIZE_SHIFT_DEFAULT    9       /* number of bytes/sector, 512  */
#define IMR_TRANSFER_PENALTY             60      /* usec */
#define IMR_TRANSFER_PENALTY_MAX         1000    /* usec */
#define IMR_ROTATE_PENALTY               11000   /* usec ,  5400rpm->  rotate time: 11ms*/


#define IMR_ALLOCATION_PHASE             2     /* phase of data distribution (2-3)*/

#define TOP_TRACK_NUM_TOTAL 64
/*
 * The size of a zone is 256MB, divided into 64 track groups (top-bottom), with an average track of 2MB.
 * A group of top-bottom has 4MB, that is, 1024 blocks, and there are 64 groups of top-bottom in a zone.
 */

#define IMR_MAX_CAPACITY                 21474836480

static __u64   IMR_CAPACITY;            /* disk capacity (in sectors) */
static __u32   IMR_NUMZONES;            /* number of zones */
static __u32   IMR_NUMZONES_DEFAULT;
static __u32   IMR_ZONE_SIZE_SHIFT;     
static __u32   IMR_BLOCK_SIZE_SHIFT;

static __u32 IMR_TOP_TRACK_SIZE = 456;      /* number of blocks/topTrack  456 */
static __u32 IMR_BOTTOM_TRACK_SIZE = 568;   /* number of blocks/bottomTrack  568 */

__u32 VERSION = IMRSIM_VERSION(1,0,0);      /* The version number of IMRSIM：VERSION(x,y,z)=>((x<<16)|(y<<8)|z) */

struct imrsim_c{             /* Mapped devices in the Device Mapper framework, also known as logical devices. */
    struct dm_dev *dev;      /* block device */
    sector_t       start;    /* starting address */
};

/* Mutex resource locks */
struct mutex                     imrsim_zone_lock;
struct mutex                     imrsim_ioctl_lock;

/* IMRSIM Statistics */
static struct imrsim_state       *zone_state = NULL;
/* Array of zone status information */
static struct imrsim_zone_status *zone_status = NULL;

/* error log */
static __u32 imrsim_dbg_rerr;
static __u32 imrsim_dbg_werr;
static __u32 imrsim_dbg_log_enabled = 0;
static unsigned long imrsim_dev_idle_checkpoint = 0;

/* Multi-device support, currently not supported */
int imrsim_single = 0;

/* Constants representing configuration changes */
enum imrsim_conf_change{
    IMR_NO_CHANGE     = 0x00,
    IMR_CONFIG_CHANGE = 0x01,
    IMR_STATS_CHANGE  = 0x02,
    IMR_STATUS_CHANGE = 0x04
};

/* persistent storage */
#define IMR_PSTORE_PG_EDG 92
#define IMR_PSTORE_PG_OFF 40
#define IMR_PSTORE_CHECK  1000
#define IMR_PSTORE_QDEPTH 128
#define IMR_PSTORE_PG_GAP 2

/* persistent storage task structure */
static struct imrsim_pstore_task
{
    struct task_struct  *pstore_thread;
    __u32                sts_zone_idx;
    __u32                stu_zone_idx[IMR_PSTORE_QDEPTH];
    __u8                 stu_zone_idx_cnt;
    __u8                 stu_zone_idx_gap;
    sector_t             pstore_lba;
    unsigned char        flag;              /* three bit for imrsim_conf_change */
}imrsim_ptask;

/* RMW scheme structure */
static struct imrsim_RMW_task
{
    sector_t            lba[2];
    __u8                lba_num;
}imrsim_rmw_task;

/* read/write completion structure */
static struct imrsim_completion_control
{
    struct completion   read_event;
    struct completion   write_event;    
}imrsim_completion;

/* To get the size of the imrsim_stats structure. */
static __u32 imrsim_stats_size(void)
{
    return (sizeof(struct imrsim_dev_stats) + sizeof(__u32) +
            sizeof(struct imrsim_zone_stats) * IMR_NUMZONES);
}

/* To get the size of the imrsim_state structure. */
static __u32 imrsim_state_size(void)
{
    return (sizeof(struct imrsim_state_header) + 
            sizeof(struct imrsim_config) + 
            sizeof(struct imrsim_dev_stats) + sizeof(__u32) +
            IMR_NUMZONES * sizeof(struct imrsim_zone_stats) + 
            IMR_NUMZONES * sizeof(struct imrsim_zone_status) +
            sizeof(__u32));
}

/* To get how many sectors a zone has. */
static __u32 num_sectors_zone(void)
{
    return (1 << IMR_BLOCK_SIZE_SHIFT << IMR_ZONE_SIZE_SHIFT);
}

/* To get the sector address where the zone starts. */
static __u64 zone_idx_lba(__u64 idx){
    return (idx << IMR_BLOCK_SIZE_SHIFT << IMR_ZONE_SIZE_SHIFT);
}

/* Returns the exponent of a power of 2. */
static __u64 index_power_of_2(__u64 num)
{
    __u64 index = 0;
    while(num >>= 1){
        ++index;
    }
    return index;
}

/* Device idle time initialization. */
static void imrsim_dev_idle_init(void)
{
    imrsim_dev_idle_checkpoint = jiffies;
    zone_state->stats.dev_stats.idle_stats.dev_idle_time_max = 0;
    zone_state->stats.dev_stats.idle_stats.dev_idle_time_min = jiffies / HZ;
}

/* Basic information for initializing zone. */
static void imrsim_init_zone_default(__u64 sizedev)   /* sizedev: in sectors */
{
    IMR_CAPACITY = sizedev;
    IMR_ZONE_SIZE_SHIFT = IMR_ZONE_SIZE_SHIFT_DEFAULT;
    IMR_BLOCK_SIZE_SHIFT = IMR_BLOCK_SIZE_SHIFT_DEFAULT;
    IMR_NUMZONES = (IMR_CAPACITY >> IMR_BLOCK_SIZE_SHIFT >> IMR_ZONE_SIZE_SHIFT);
    IMR_NUMZONES_DEFAULT = IMR_NUMZONES;
    printk(KERN_INFO "imrsim_init_zone_state: numzones=%d sizedev=%llu\n",
        IMR_NUMZONES, sizedev); 
}

/* Basic information for initializing the device state (zone_state) */
static void imrsim_init_zone_state_default(__u32 state_size)
{
    __u32 i;
    __u32 j;
    __u32 *magic;   /* magic number to identify the device (equipment identity) */

    /* head info. */
    zone_state->header.magic = 0xBEEFBEEF;
    zone_state->header.length = state_size;
    zone_state->header.version = VERSION;
    zone_state->header.crc32 = 0;

    /* config info. */
    zone_state->config.dev_config.out_of_policy_read_flag = 0;
    zone_state->config.dev_config.out_of_policy_write_flag = 0;
    zone_state->config.dev_config.r_time_to_rmw_zone = IMR_TRANSFER_PENALTY;
    zone_state->config.dev_config.w_time_to_rmw_zone = IMR_TRANSFER_PENALTY;

    zone_state->stats.num_zones = IMR_NUMZONES;
    imrsim_reset_stats();    
    /* To allocate space for the zone_status array and initialize it. */
    zone_status = (struct imrsim_zone_status *)&zone_state->stats.zone_stats[IMR_NUMZONES];
    for(i=0; i<IMR_NUMZONES; i++){
        zone_status[i].z_start = i;
        zone_status[i].z_length = num_sectors_zone();
        zone_status[i].z_type = Z_TYPE_CONVENTIONAL;
        zone_status[i].z_conds = Z_COND_NO_WP;
        zone_status[i].z_flag = 0;
        for(j=0;j<TOP_TRACK_NUM_TOTAL;j++){
            memset(zone_status[i].z_tracks[j].isUsedBlock,0,IMR_TOP_TRACK_SIZE*sizeof(__u8));
        }
        zone_status[i].z_map_size = 0;
        memset(zone_status[i].z_pba_map,-1,TOP_TRACK_NUM_TOTAL*(IMR_TOP_TRACK_SIZE+IMR_BOTTOM_TRACK_SIZE)*sizeof(int));
    }
    printk(KERN_INFO "imrsim: %s zone_status init!\n", __FUNCTION__);
    magic = (__u32 *)&zone_status[IMR_NUMZONES];
    *magic = 0xBEEFBEEF;
}

/* To initial a device. */
int imrsim_init_zone_state(__u64 sizedev)
{
    __u32 state_size;

    if(!sizedev){
        printk(KERN_ERR "imrsim: zero capacity detected\n");
        return -EINVAL;
    }
    imrsim_init_zone_default(sizedev);      /* Initialize the basic information of the zone. */
    /* zone_state should not have allocated space, if it already exists, reclaim the space. */
    if(zone_state){
        vfree(zone_state);
    }
    state_size = imrsim_state_size();
    zone_state = vzalloc(state_size);     /* Allocate memory space for zone_state */
    if(!zone_state){
        printk(KERN_ERR "imrsim: memory alloc failed for zone state\n");
        return -ENOMEM;
    }
    imrsim_init_zone_state_default(state_size);   
    imrsim_dev_idle_init();       
    return 0;
}

/* read completion */
static void imrsim_read_completion(struct bio *bio, int err)
{
    if(err){
        printk(KERN_ERR "imrsim: bio read err: %d\n", err);
    }
    if(bio){
        complete((struct completion *)bio->bi_private);
        printk(KERN_INFO "imrsim: ----release read completion---- \n");
    }
}

/* write completion */
static void imrsim_write_completion(struct bio *bio, int err)
{
    if(err){
        printk(KERN_ERR "imrsim: bio write err:%d\n", err);
    }
    if(bio){
        complete((struct completion *)bio->bi_private);
    }
}

/* read page (for meta-data) */
static int imrsim_read_page(struct block_device *dev, sector_t lba,
                            int size, struct page *page)
{
    //dump_stack();   // #include<asm/ptrace.h> Debugging: View function call stacks
    int ret = 0;
    struct bio *bio = bio_alloc(GFP_NOIO, 1);

    if(!bio){
        printk(KERN_ERR "imrsim: %s bio_alloc failed\n", __FUNCTION__);
        return -EFAULT;
    }
    bio->bi_bdev = dev;
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
    bio->bi_sector = lba;
    #else
    bio->bi_iter.bi_sector = lba;
    #endif
    bio_add_page(bio, page, size, 0);
    init_completion(&imrsim_completion.read_event);
    bio->bi_private = &imrsim_completion.read_event;
    bio->bi_end_io = imrsim_read_completion;
    printk(KERN_INFO "imrsim: read page progress 1\n");
    submit_bio(READ | REQ_SYNC, bio);
    wait_for_completion(&imrsim_completion.read_event);
    ret = test_bit(BIO_UPTODATE, &bio->bi_flags);
    if(!ret){
        printk(KERN_ERR "imrsim: pstore bio read failed\n");
        ret = -EIO;
    }
    bio_put(bio);
    printk(KERN_INFO "imrsim: read page progress 2\n");
    return ret;
}

/* write page (for meta-data) */
static int imrsim_write_page(struct block_device *dev, sector_t lba,
                            __u32 size, struct page *page)
{
    int ret = 0;
    struct bio *bio = bio_alloc(GFP_NOIO, 1);

    if(!bio){
        printk(KERN_ERR "imrsim: %s bio_alloc failed\n", __FUNCTION__);
        return -EFAULT;
    }
    bio->bi_bdev = dev;
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
    bio->bi_sector = lba;
    #else
    bio->bi_iter.bi_sector = lba;
    #endif
    bio_add_page(bio, page, size, 0);
    init_completion(&imrsim_completion.write_event);
    bio->bi_private = &imrsim_completion.write_event;
    bio->bi_end_io = imrsim_write_completion;
    submit_bio(WRITE_FLUSH_FUA, bio);
    wait_for_completion(&imrsim_completion.write_event);
    ret = test_bit(BIO_UPTODATE, &bio->bi_flags);
    if(!ret){
        printk(KERN_ERR "imrsim: pstore bio write failed\n");
        ret = -EIO;
    }
    bio_put(bio);
    return ret;
}

/* RMW event caused by bottom track update */
static void imrsim_rmw_process(struct bio *bio)
{
    __u64       lba;
    void        *page_addr;
    struct page *page;  // During the RMW process, for the extra overhead of the block, use the page to back up the data

    if(bio){
        __u8 i;
        for(i=0;i<imrsim_rmw_task.lba_num;i++)
        {
            // redistribute bio (R and W option to the top track)
            lba = imrsim_rmw_task.lba[i];
            printk(KERN_INFO "imrsim: rmw process %u lba: %llu.\n", i,lba>>IMR_BLOCK_SIZE_SHIFT);
            page = alloc_pages(GFP_KERNEL,0);
            if(!page){
                printk(KERN_ERR "imrsim: no enough memory to allocate a page\n");
                return -ENOMEM;
            }
            page_addr = page_address(page);
            if(!page_addr){
                printk(KERN_ERR "imrsim: write page vm addr null\n");
                __free_pages(page,0);
                return -EINVAL;
            }
            memset(page_addr, 0, PAGE_SIZE);
            imrsim_read_page(bio->bi_bdev, lba, PAGE_SIZE, page);
            // imrsim_write_page(bio->bi_bdev, lba, PAGE_SIZE, page);
            __free_pages(page, 0);
        }
        imrsim_rmw_task.lba_num=0;
    }
}


/* To get page number and next page. */
static __u32 imrsim_pstore_pg_idx(__u32 idx, __u32 *pg_nxt)
{
    __u32 tmp = IMR_PSTORE_PG_OFF + sizeof(struct imrsim_zone_stats)*idx;
    __u32 pg_cur = tmp / PAGE_SIZE;

    *pg_nxt = tmp % PAGE_SIZE ? pg_cur+1 : pg_cur;
    return pg_cur;
}

/* Persistent storage - handled in a variety of cases depending on the type of metadata change.*/
static int imrsim_flush_persistence(struct dm_target *ti)
{
    void             *page_addr;
    struct page      *page;
    struct imrsim_c  *zdev;
    __u32            idx;
    __u32            crc;    
    __u32            pg_cur;
    __u32            pg_nxt;
    __u32            qidx;

    zdev = ti->private;
    page = alloc_pages(GFP_KERNEL, 0);
    if(!page){
        printk(KERN_ERR "imrsim: no enough memory to allocate a page\n");
        return -ENOMEM;
    }
    page_addr = page_address(page);
    if(!page_addr){
        printk(KERN_ERR "imrsim: write page vm addr null\n");
        __free_pages(page, 0);
        return -EINVAL;
    }
    crc = crc32(0, (unsigned char *)zone_state + sizeof(struct imrsim_state_header),
                zone_state->header.length - sizeof(struct imrsim_state_header));
    zone_state->header.crc32 = crc;
    if(imrsim_ptask.flag &= IMR_CONFIG_CHANGE){
        imrsim_ptask.flag &= ~IMR_CONFIG_CHANGE;
    }
    memcpy(page_addr, (unsigned char *)zone_state, PAGE_SIZE);
    imrsim_write_page(zdev->dev->bdev, imrsim_ptask.pstore_lba, PAGE_SIZE, page);

    /* Handling of Disk Statistics Changes. */
    if(imrsim_ptask.flag &= IMR_STATS_CHANGE){
        imrsim_ptask.flag &= ~IMR_STATS_CHANGE;
        if(imrsim_ptask.sts_zone_idx > IMR_PSTORE_PG_EDG){
            pg_cur = imrsim_pstore_pg_idx(imrsim_ptask.sts_zone_idx, &pg_nxt);
            imrsim_ptask.sts_zone_idx = 0;
            for(idx = pg_cur; idx <= pg_nxt; idx++){
                memcpy(page_addr, ((unsigned char *)zone_state + idx * PAGE_SIZE), PAGE_SIZE);
                imrsim_write_page(zdev->dev->bdev, imrsim_ptask.pstore_lba + 
                                (idx << IMR_PAGE_SIZE_SHIFT_DEFAULT),
                                PAGE_SIZE, page);
            }
        }
    }

    /* Handling of disk state changes. */
    if(imrsim_ptask.flag &= IMR_STATUS_CHANGE){
        imrsim_ptask.flag &= ~IMR_STATUS_CHANGE;
        for(qidx = 0; qidx < imrsim_ptask.stu_zone_idx_cnt; qidx++){
            pg_cur = imrsim_pstore_pg_idx(imrsim_ptask.stu_zone_idx[qidx], &pg_nxt);
            imrsim_ptask.stu_zone_idx[qidx] = 0;
            for(idx = pg_cur; idx <= pg_nxt; idx++){
                memcpy(page_addr, ((unsigned char *)zone_state + idx * PAGE_SIZE), PAGE_SIZE);
                imrsim_write_page(zdev->dev->bdev, imrsim_ptask.pstore_lba + 
                                (idx << IMR_PAGE_SIZE_SHIFT_DEFAULT),
                                PAGE_SIZE, page);
            }
        }
        imrsim_ptask.stu_zone_idx_cnt = 0;
        imrsim_ptask.stu_zone_idx_gap = 0;
    }

    if(imrsim_dbg_log_enabled && printk_ratelimit()){
        printk(KERN_ERR "imrsim: flush persist success\n");
    }
    __free_pages(page, 0);
    return 0;
}

/* To persist meta-data. */
static int imrsim_save_persistence(struct dm_target *ti)
{
    void             *page_addr;
    struct page      *page;
    struct imrsim_c  *zdev;
    __u32            num_pages;
    __u32            part_page;
    __u32            idx;
    __u32            crc;

    zdev = ti->private;
    page = alloc_pages(GFP_KERNEL, 0);
    if(!page){
        printk(KERN_ERR "imrsim: no enough memory to allocate a page\n");
        return -ENOMEM;
    }
    page_addr = page_address(page);
    if(!page_addr){
        printk(KERN_ERR "imrsim: write page vm addr null\n");
        __free_pages(page, 0);
        return -EINVAL;
    }
    num_pages = div_u64_rem(zone_state->header.length, PAGE_SIZE, &part_page);
    crc = crc32(0, (unsigned char *)zone_state + sizeof(struct imrsim_state_header),
                zone_state->header.length - sizeof(struct imrsim_state_header));
    zone_state->header.crc32 = crc;
    for(idx = 0; idx < num_pages; idx++){
        memcpy(page_addr, ((unsigned char *)zone_state + 
               idx * PAGE_SIZE), PAGE_SIZE);
        imrsim_write_page(zdev->dev->bdev, imrsim_ptask.pstore_lba + 
                         (idx << IMR_PAGE_SIZE_SHIFT_DEFAULT), PAGE_SIZE, page);
    }
    if(part_page){
        memcpy(page_addr, ((unsigned char *)zone_state + 
              num_pages * PAGE_SIZE), part_page);
        imrsim_write_page(zdev->dev->bdev, imrsim_ptask.pstore_lba + 
                          (num_pages << IMR_PAGE_SIZE_SHIFT_DEFAULT), PAGE_SIZE, page);
    }
    if(imrsim_dbg_log_enabled && printk_ratelimit()){
        printk(KERN_INFO "imrsim: save persist success\n");
    }
    __free_pages(page, 0);
    return 0;
}

/* To load metadata from persistent storage. */
static int imrsim_load_persistence(struct dm_target *ti)
{
    __u64            sizedev;
    void             *page_addr;
    struct page      *page;
    struct imrsim_c  *zdev;
    __u32            num_pages;
    __u32            part_page;     
    __u32            idx;
    __u32            crc;
    struct imrsim_state_header header;

    printk(KERN_INFO "imrsim: load persistence\n");

    zdev = ti->private;
    sizedev = ti->len;
    imrsim_init_zone_default(sizedev);
    /* The starting address for persistent storage. */
    imrsim_ptask.pstore_lba = IMR_NUMZONES_DEFAULT
                              << IMR_ZONE_SIZE_SHIFT_DEFAULT
                              << IMR_BLOCK_SIZE_SHIFT_DEFAULT;
    page = alloc_pages(GFP_KERNEL, 0);
    if(!page){
        printk(KERN_ERR "imrsim: no enough memory to allocate a page\n");
        goto pgerr;
    }
    page_addr = page_address(page);
    if(!page_addr){
        printk(KERN_ERR "imrsim: read page vm addr null\n");
        goto rderr;
    }
    memset(page_addr, 0, PAGE_SIZE);
    imrsim_read_page(zdev->dev->bdev, imrsim_ptask.pstore_lba, PAGE_SIZE, page);
    memcpy(&header, page_addr, sizeof(struct imrsim_state_header));
    if(header.magic == 0xBEEFBEEF){
        zone_state = vzalloc(header.length);
        if(!zone_state){
            printk(KERN_ERR "imrsim: zone_state error: no enough memory\n");
            goto rderr;
        }
        num_pages = div_u64_rem(header.length, PAGE_SIZE, &part_page);
        if(num_pages){
            memcpy((unsigned char *)zone_state, page_addr, PAGE_SIZE);  // load header
        }
        for(idx = 1; idx < num_pages; idx++){
            memset(page_addr, 0, PAGE_SIZE);
            imrsim_read_page(zdev->dev->bdev, imrsim_ptask.pstore_lba + 
                            (idx << IMR_PAGE_SIZE_SHIFT_DEFAULT), PAGE_SIZE, page);
            memcpy(((unsigned char *)zone_state + 
                  idx * PAGE_SIZE), page_addr, PAGE_SIZE);
        }
        if(part_page){
            if(num_pages){
                memset(page_addr, 0, PAGE_SIZE);
                imrsim_read_page(zdev->dev->bdev, imrsim_ptask.pstore_lba + 
                                (num_pages << IMR_PAGE_SIZE_SHIFT_DEFAULT), 
                                PAGE_SIZE, page);
            }
            memcpy(((unsigned char *)zone_state + 
                   idx * PAGE_SIZE), page_addr, part_page);
        }
        crc = crc32(0, (unsigned char *)zone_state + sizeof(struct imrsim_state_header), 
                   zone_state->header.length - sizeof(struct imrsim_state_header));
        if(crc != zone_state->header.crc32){
            printk(KERN_ERR "imrsim: error: crc checking. apply default config ...\n");
            goto rderr;
        }
        IMR_NUMZONES = zone_state->stats.num_zones;
        zone_status = (struct imrsim_zone_status *)&zone_state->stats.zone_stats[IMR_NUMZONES];
        IMR_ZONE_SIZE_SHIFT = index_power_of_2(zone_status[0].z_length >> IMR_BLOCK_SIZE_SHIFT);
        printk(KERN_INFO "imrsim: load persist success\n");
    }else{
        printk(KERN_ERR "imrsim: load persistence magic doesn't match. Setup the default\n");
        goto rderr;
    }
    __free_pages(page, 0);
    return 0;
    rderr:
        __free_pages(page, 0);
    pgerr:
        imrsim_init_zone_state(sizedev);
    return -EINVAL;
}

/* persistent storage task */
static int imrsim_persistence_task(void *arg)
{
    struct dm_target *ti = (struct dm_target *)arg;

    while(!kthread_should_stop()){
        if(imrsim_ptask.flag){
            mutex_lock(&imrsim_zone_lock);
            if(imrsim_ptask.flag & IMR_CONFIG_CHANGE){
                if(IMR_NUMZONES == 0){
                    imrsim_ptask.flag &= IMR_NO_CHANGE;
                }else{
                    imrsim_save_persistence(ti);
                    imrsim_ptask.flag &= IMR_NO_CHANGE;
                }
            }else{
                if(imrsim_ptask.stu_zone_idx_gap >= IMR_PSTORE_PG_GAP){
                    imrsim_save_persistence(ti);
                    imrsim_ptask.flag &= IMR_NO_CHANGE;
                    imrsim_ptask.stu_zone_idx_gap = 0;
                    memset(imrsim_ptask.stu_zone_idx, 0, sizeof(__u32) * IMR_PSTORE_QDEPTH);
                    imrsim_ptask.stu_zone_idx_cnt = 0;
                }else{
                    imrsim_flush_persistence(ti);
                }
            }
            mutex_unlock(&imrsim_zone_lock);
        }
        msleep_interruptible(IMR_PSTORE_CHECK);
    }
    return 0;
}

/* persistent storage thread */
static int imrsim_persistence_thread(struct dm_target *ti)
{
    int ret = 0;

    if(!ti){
        printk(KERN_ERR "imrsim: warning: null device target. Improper usage\n");
        return -EINVAL;
    }
    //init_completion(&imrsim_ptask.read_event);
    //init_completion(&imrsim_ptask.write_event);
    imrsim_ptask.flag = 0;
    imrsim_ptask.stu_zone_idx_cnt = 0;
    imrsim_ptask.stu_zone_idx_gap = 0;
    memset(imrsim_ptask.stu_zone_idx, 0, sizeof(__u32) * IMR_PSTORE_QDEPTH);
    ret = imrsim_load_persistence(ti);
    if(ret){
        imrsim_save_persistence(ti);
    }
    // create thread
    imrsim_ptask.pstore_thread = kthread_create(imrsim_persistence_task, 
                                                ti, "imrsim pthread");
    if(imrsim_ptask.pstore_thread){
        printk(KERN_INFO "imrsim persistence thread created\n");
        // 使用 kthread_create 创建一个线程后该线程并不会立马启动，而是需要在调用 wake_up_process 函数之后才会启动。
        wake_up_process(imrsim_ptask.pstore_thread);
    }else{
        printk(KERN_ERR "imrsim persistence thread create failed\n");
        return -EAGAIN;
    }
    return 0;
}

/* To update device idle time. */
static void imrsim_dev_idle_update(void)
{
    __u32 dt = 0;
    if(jiffies > imrsim_dev_idle_checkpoint){
        dt = (jiffies - imrsim_dev_idle_checkpoint) / HZ;
    }else{
        dt = (~(__u32)0 - imrsim_dev_idle_checkpoint + jiffies) / HZ;
    }
    if (dt > zone_state->stats.dev_stats.idle_stats.dev_idle_time_max) {
      zone_state->stats.dev_stats.idle_stats.dev_idle_time_max = dt;
   } else if (dt && (dt < zone_state->stats.dev_stats.idle_stats.dev_idle_time_min)) {
      zone_state->stats.dev_stats.idle_stats.dev_idle_time_min = dt;
   }
}

/* status report */
static void imrsim_report_stats(struct imrsim_stats *stats)
{
    __u32 i;
    __u32 num32 = stats->num_zones;

    if (!stats) {
       printk(KERN_ERR "imrsim: NULL pointer passed through\n");
       return;
    }
    printk("Device idle time max: %u\n",
            stats->dev_stats.idle_stats.dev_idle_time_max);
    printk("Device idle time min: %u\n",
            stats->dev_stats.idle_stats.dev_idle_time_min);
    for (i = 0; i < num32; i++) {
        printk("zone[%u] imrsim out of policy read stats: span zones count: %u\n",
                    i, stats->zone_stats[i].out_of_policy_read_stats.span_zones_count);
        printk("zone[%u] imrsim out of policy write stats: span zones count: %u\n",
                    i, stats->zone_stats[i].out_of_policy_write_stats.span_zones_count);
        printk("zone[%u] imrsim out of policy write stats: unaligned count: %u\n",
                    i, stats->zone_stats[i].out_of_policy_write_stats.unaligned_count);
        printk("zone[%u] imrsim rewrite_total count: %u\n",
                    i, stats->zone_stats[i].rewrite_total); 
        printk("zone[%u] imrsim rewrite_single count: %u\n",
                    i, stats->zone_stats[i].rewrite_single);    
    }
}

/* The following are interface methods with EXPORT_SYMBOL. */

/* To get the last read error. */
int imrsim_get_last_rd_error(__u32 *last_error)
{
    __u32 tmperr = imrsim_dbg_rerr;

    imrsim_dbg_rerr = 0;
    if(last_error){
        *last_error = tmperr;
    }
    return 0;
}
EXPORT_SYMBOL(imrsim_get_last_rd_error);

/* To get the last write error. */
int imrsim_get_last_wd_error(__u32 *last_error)
{
   __u32 tmperr = imrsim_dbg_werr;

   imrsim_dbg_werr  = 0;
   if(last_error)
      *last_error = tmperr;
   return 0;
}
EXPORT_SYMBOL(imrsim_get_last_wd_error);

/* Enable logging. */
int imrsim_set_log_enable(__u32 zero_is_disable)
{
   imrsim_dbg_log_enabled = zero_is_disable;
   return 0;
}
EXPORT_SYMBOL(imrsim_set_log_enable);

/* Disable logging. */
int imrsim_get_num_zones(__u32* num_zones)
{
   printk(KERN_INFO "imrsim: %s: called.\n", __FUNCTION__);
   if (!num_zones) {
      printk(KERN_ERR "imrsim: NULL pointer passed through\n");
      return -EINVAL;
   }
   mutex_lock(&imrsim_zone_lock);
   *num_zones = IMR_NUMZONES;
   mutex_unlock(&imrsim_zone_lock);
   return 0;
}
EXPORT_SYMBOL(imrsim_get_num_zones);

/* To get the number of sectors in a zone. */
int imrsim_get_size_zone_default(__u32 *size_zone)
{
    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__);
    if(!size_zone){
        printk(KERN_ERR "imrsim: NULL pointer passed through\n");
        return -EINVAL;
    }
    mutex_lock(&imrsim_zone_lock);
    *size_zone = num_sectors_zone();
    mutex_unlock(&imrsim_zone_lock);
    return 0;
}
EXPORT_SYMBOL(imrsim_get_size_zone_default);

/* To set the default zone size. */
int imrsim_set_size_zone_default(__u32 size_zone)
{
    struct imrsim_state *sta_tmp;

    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__);
    if((size_zone % (1 << IMR_BLOCK_SIZE_SHIFT)) || !(is_power_of_2(size_zone))){
        printk(KERN_ERR "imrsim: Wrong zone size specified\n");
        return -EINVAL;
    }
    mutex_lock(&imrsim_zone_lock);
    IMR_ZONE_SIZE_SHIFT = index_power_of_2((size_zone) >> IMR_BLOCK_SIZE_SHIFT);
    IMR_NUMZONES = ((IMR_CAPACITY >> IMR_BLOCK_SIZE_SHIFT) >> IMR_ZONE_SIZE_SHIFT);
    sta_tmp = vzalloc(imrsim_state_size());
    if(!sta_tmp){
        mutex_unlock(&imrsim_zone_lock);
        printk(KERN_ERR "imrsim: zone_state memory realloc failed\n");
        return -EINVAL;
    }
    vfree(zone_state);
    zone_state = sta_tmp;
    imrsim_init_zone_state_default(imrsim_state_size());
    mutex_unlock(&imrsim_zone_lock);
    return 0;
}
EXPORT_SYMBOL(imrsim_set_size_zone_default);

/* To reset default config. */
int imrsim_reset_default_config(void)
{
    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__);
    imrsim_reset_default_zone_config();
    imrsim_reset_default_device_config();
    return 0;
}
EXPORT_SYMBOL(imrsim_reset_default_config);

/* To reset default device config. */
int imrsim_reset_default_device_config(void)
{
    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__);
    mutex_lock(&imrsim_zone_lock);
    zone_state->config.dev_config.out_of_policy_read_flag = 0;
    zone_state->config.dev_config.out_of_policy_write_flag = 0;
    zone_state->config.dev_config.r_time_to_rmw_zone = IMR_TRANSFER_PENALTY;
    zone_state->config.dev_config.w_time_to_rmw_zone = IMR_TRANSFER_PENALTY;
    mutex_unlock(&imrsim_zone_lock);
    return 0;
}
EXPORT_SYMBOL(imrsim_reset_default_device_config);

/* To get device config. */
int imrsim_get_device_config(struct imrsim_dev_config *device_config)
{
    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__);
    if(!device_config){
        printk(KERN_ERR "imrsim: NULL pointer passed through\n");
        return -EINVAL;
    }
    mutex_lock(&imrsim_zone_lock);
    memcpy(device_config, &(zone_state->config.dev_config), 
           sizeof(struct imrsim_dev_config));
    mutex_unlock(&imrsim_zone_lock);
    return 0;
}
EXPORT_SYMBOL(imrsim_get_device_config);

/* To set device read config. */
int imrsim_set_device_rconfig(struct imrsim_dev_config *device_config)
{
    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__);
    if(!device_config){
        printk(KERN_ERR "imrsim: NULL pointer passed through\n");
        return -EINVAL;
    }
    mutex_lock(&imrsim_zone_lock);
    zone_state->config.dev_config.out_of_policy_read_flag = 
        device_config->out_of_policy_read_flag;
    mutex_unlock(&imrsim_zone_lock);
    return 0;
}
EXPORT_SYMBOL(imrsim_set_device_rconfig);

/* To set device write config. */
int imrsim_set_device_wconfig(struct imrsim_dev_config *device_config)
{
    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__);
    if(!device_config){
        printk(KERN_ERR "imrsim: NULL pointer passed through\n");
        return -EINVAL;
    }
    mutex_lock(&imrsim_zone_lock);
    zone_state->config.dev_config.out_of_policy_write_flag = 
        device_config->out_of_policy_write_flag;
    mutex_unlock(&imrsim_zone_lock);
    return 0;
}
EXPORT_SYMBOL(imrsim_set_device_wconfig);

/* To set read delay. */
int imrsim_set_device_rconfig_delay(struct imrsim_dev_config *device_config)
{
    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__);
    if(!device_config){
        printk(KERN_ERR "imrsim: NULL pointer passed through\n");
        return -EINVAL;
    }
    if(device_config->r_time_to_rmw_zone >= IMR_TRANSFER_PENALTY_MAX){
        printk(KERN_ERR "time delay exceeds default maximum\n");
        return -EINVAL;
    }
    mutex_lock(&imrsim_zone_lock);
    zone_state->config.dev_config.r_time_to_rmw_zone = 
        device_config->r_time_to_rmw_zone;
    mutex_unlock(&imrsim_zone_lock);
    return 0;
}
EXPORT_SYMBOL(imrsim_set_device_rconfig_delay);

/* To set write delay. */
int imrsim_set_device_wconfig_delay(struct imrsim_dev_config *device_config)
{
    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__);
    if(!device_config){
        printk(KERN_ERR "imrsim: NULL pointer passed through\n");
        return -EINVAL;
    }
    if(device_config->w_time_to_rmw_zone >= IMR_TRANSFER_PENALTY_MAX){
        printk(KERN_ERR "time delay exceeds default maximum\n");
        return -EINVAL;
    }
    mutex_lock(&imrsim_zone_lock);
    zone_state->config.dev_config.w_time_to_rmw_zone = 
        device_config->w_time_to_rmw_zone;
    mutex_unlock(&imrsim_zone_lock);
    return 0;
}
EXPORT_SYMBOL(imrsim_set_device_wconfig_delay);

/* To reset default zone config. */
int imrsim_reset_default_zone_config(void)
{
    struct imrsim_state *sta_tmp;

    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__);
    mutex_lock(&imrsim_zone_lock);
    IMR_NUMZONES = IMR_NUMZONES_DEFAULT;
    IMR_ZONE_SIZE_SHIFT = IMR_ZONE_SIZE_SHIFT_DEFAULT;
    sta_tmp = vzalloc(imrsim_state_size());
    vfree(zone_state);
    if(!sta_tmp){
        printk(KERN_ERR "imrsim: zone_state memory realloc failed\n");
        return -EINVAL;
    }
    zone_state = sta_tmp;
    imrsim_init_zone_state_default(imrsim_state_size());
    mutex_unlock(&imrsim_zone_lock);
    return 0;
}
EXPORT_SYMBOL(imrsim_reset_default_zone_config);

/* To clear config of a zone. */
int imrsim_clear_zone_config(void)
{
    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__);
    memset(zone_state->stats.zone_stats, 0, 
       zone_state->stats.num_zones * sizeof(struct imrsim_zone_stats));
    mutex_lock(&imrsim_zone_lock);
    zone_state->stats.num_zones = 0;
    memset(zone_status, 0, IMR_NUMZONES * sizeof(struct imrsim_zone_status));
    IMR_NUMZONES = 0;
    mutex_unlock(&imrsim_zone_lock);
    return 0;
}
EXPORT_SYMBOL(imrsim_clear_zone_config);

/* Count the number of Z_TYPE_SEQUENTIAL type zones. @Deprecated */
static int imrsim_zone_seq_count(void)
{
    __u32 count = 0;
    __u32 index;

    for(index = 0; index < IMR_NUMZONES; index++){
        if(zone_status[index].z_type == Z_TYPE_SEQUENTIAL){
            count++;
        }
    }
    return count;
}

/* To check if the zone status is correct. */
static int imrsim_zone_cond_check(__u16 cond)
{
    switch(cond){
        case Z_COND_NO_WP:
        case Z_COND_EMPTY:
        case Z_COND_CLOSED:
        case Z_COND_RO:
        case Z_COND_FULL:
        case Z_COND_OFFLINE:
            return 1;
        default:
            return 0;
    }
    return 0;
}

/* To modify zone configuration. @Deprecated */
int imrsim_modify_zone_config(struct imrsim_zone_status *z_status)
{
    __u32 count = imrsim_zone_seq_count();

    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__); 
    if(!z_status){
        printk(KERN_ERR "imrsim: NULL pointer passed through\n");
        return -EINVAL;
    }
    if(IMR_NUMZONES <= z_status->z_start){
        printk(KERN_ERR "imrsim: config does not exist\n");
        return -EINVAL;
    }
    if(1 >= count && (Z_TYPE_SEQUENTIAL == z_status->z_type) &&
      (Z_TYPE_SEQUENTIAL == zone_status[z_status->z_start].z_type))
    {
          printk(KERN_ERR "imrsim: zone type is not allowed to modify\n");
          return -EINVAL;
    }
    if(z_status->z_length != num_sectors_zone()){
        printk(KERN_ERR "imrsim: zone size is not allowed to change individually\n");
        return -EINVAL;
    }
    if(!imrsim_zone_cond_check(z_status->z_conds)){
        printk(KERN_ERR "imrsim: wrong zone condition\n");
        return -EINVAL;
    }
    if((z_status->z_conds == Z_COND_NO_WP) && 
        (z_status->z_type != Z_TYPE_CONVENTIONAL))
    {
        printk(KERN_ERR "imrsim: condition and type mismatch\n");
        return -EINVAL;
    }
    if ((Z_COND_EMPTY == z_status->z_conds) && 
       (Z_TYPE_SEQUENTIAL == z_status->z_type) ) {
      printk(KERN_ERR "imrsim: empty zone isn't empty\n");
      return -EINVAL;
    }

    mutex_lock(&imrsim_zone_lock);
    zone_status[z_status->z_start].z_conds = 
      (enum imrsim_zone_conditions)z_status->z_conds;
    zone_status[z_status->z_start].z_type = 
      (enum imrsim_zone_type)z_status->z_type;
    zone_status[z_status->z_start].z_flag = 0;
    mutex_unlock(&imrsim_zone_lock);
    printk(KERN_DEBUG "imrsim: zone[%lu] modified. type:0x%x conds:0x%x\n",
      zone_status[z_status->z_start].z_start,
      zone_status[z_status->z_start].z_type, 
      zone_status[z_status->z_start].z_conds);
    return 0;
}
EXPORT_SYMBOL(imrsim_modify_zone_config);

/* To add zone configuration. @Deprecated */
int imrsim_add_zone_config(struct imrsim_zone_status *zone_sts)
{
    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__);
    if(!zone_sts){
        printk(KERN_ERR "imrsim: NULL pointer passed through\n");
        return -EINVAL;
    }
    if(zone_sts->z_start >= IMR_NUMZONES_DEFAULT){
        printk(KERN_ERR "imrsim: zone config start lba is out of range\n");
        return -EINVAL;
    }
    if(zone_sts->z_start != IMR_NUMZONES){
        printk(KERN_ERR "imrsim: zone config does not start at the end of current zone\n");
        printk(KERN_INFO "imrsim: z_start: %u  IMR_NUMZONES: %u\n", (__u32)zone_sts->z_start,
             IMR_NUMZONES);
        return -EINVAL;
    }
    if ((zone_sts->z_type != Z_TYPE_CONVENTIONAL) && (zone_sts->z_type != Z_TYPE_SEQUENTIAL)) {
      printk(KERN_ERR "imrsim: zone config type is not allowed with current config\n");
      return -EINVAL;
   }
   if ((zone_sts->z_type == Z_TYPE_CONVENTIONAL) && (zone_sts->z_conds != Z_COND_NO_WP)) {
      printk(KERN_ERR "imrsim: zone config condition is wrong. Need to be NO WP\n");
      return -EINVAL;
   }
   if ((zone_sts->z_type == Z_TYPE_SEQUENTIAL) && (zone_sts->z_conds != Z_COND_EMPTY)) {
      printk(KERN_ERR "imrsim: zone config condition is wrong. Need to be EMPTY\n");
      return -EINVAL;
   }
   if (zone_sts->z_length != (1 << IMR_ZONE_SIZE_SHIFT << IMR_BLOCK_SIZE_SHIFT)) {
      printk(KERN_ERR "imrsim: zone config size is not allowed with current config\n");
      return -EINVAL;
   }
   zone_sts->z_flag = 0;
   mutex_lock(&imrsim_zone_lock);
   memcpy(&(zone_status[IMR_NUMZONES]), zone_sts, sizeof(struct imrsim_zone_status));
   zone_state->stats.num_zones++;
   IMR_NUMZONES++;
   mutex_unlock(&imrsim_zone_lock);
   return 0;
}
EXPORT_SYMBOL(imrsim_add_zone_config);

/* To reset statistics for a zone. */
int imrsim_reset_zone_stats(sector_t start_sector)
{
    __u32 zone_idx = start_sector >> IMR_BLOCK_SIZE_SHIFT >> IMR_ZONE_SIZE_SHIFT;

    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__);
    if(IMR_NUMZONES <= zone_idx){
        printk(KERN_ERR "imrsim: %s start sector is out of range\n", __FUNCTION__);
        return -EINVAL;
    }
    memset(&(zone_state->stats.zone_stats[zone_idx].out_of_policy_read_stats),
          0, sizeof(struct imrsim_out_of_policy_read_stats));
    memset(&(zone_state->stats.zone_stats[zone_idx].out_of_policy_write_stats),
          0, sizeof(struct imrsim_out_of_policy_write_stats));
    memset(&(zone_state->stats.zone_stats[zone_idx].rewrite_total),
          0, sizeof(__u32));
    memset(&(zone_state->stats.zone_stats[zone_idx].rewrite_single),
          0, sizeof(__u32));
    return 0;
}
EXPORT_SYMBOL(imrsim_reset_zone_stats);

/* To reset zone_stats. */
int imrsim_reset_stats(void)
{
    printk(KERN_INFO "imrsim: %s: called.\n", __FUNCTION__);
    memset(&zone_state->stats.dev_stats.idle_stats, 0, sizeof(struct imrsim_idle_stats));
    memset(zone_state->stats.zone_stats, 0, zone_state->stats.num_zones * 
          sizeof(struct imrsim_zone_stats));
    return 0;
}
EXPORT_SYMBOL(imrsim_reset_stats);

/* To get zone_stats. */
int imrsim_get_stats(struct imrsim_stats *stats)
{
    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__);
    if(!stats){
        printk(KERN_ERR "imrsim: NULL pointer passed through\n");
        return -EINVAL;
    }
    memcpy(stats, &(zone_state->stats), imrsim_stats_size());
    return 0;
}
EXPORT_SYMBOL(imrsim_get_stats);

/* @Deprecated */
int imrsim_blkdev_reset_zone_ptr(sector_t start_sector)
{
    //__u32 rem;
    __u32 zone_idx;

    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__);
    mutex_lock(&imrsim_zone_lock);
    zone_idx = start_sector >> IMR_BLOCK_SIZE_SHIFT >> IMR_ZONE_SIZE_SHIFT;
    if(IMR_NUMZONES <= zone_idx){
        mutex_unlock(&imrsim_zone_lock);
        printk(KERN_ERR "imrsim: %s start_sector is out of range\n", __FUNCTION__);
        return -EINVAL;
    }
    if (zone_status[zone_idx].z_type == Z_TYPE_CONVENTIONAL) {
      mutex_unlock(&imrsim_zone_lock);
      printk(KERN_ERR "imrsim:error: CMR zone dosen't have a write pointer.\n");
      return -EINVAL;
    }
    mutex_unlock(&imrsim_zone_lock);
    return 0;
}
EXPORT_SYMBOL(imrsim_blkdev_reset_zone_ptr);

/* error log */
void imrsim_log_error(struct bio* bio, __u32 uerr)
{
    __u64 lba;

    if (!bio) {
        printk(KERN_ERR "imrsim: NULL pointer passed through\n");
        return;
    }
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0)
    lba = bio->bi_sector;
    #else
    lba = bio->bi_iter.bi_sector;
    #endif
    if (imrsim_dbg_log_enabled) {
        switch(uerr)
        {
            case IMR_ERR_READ_BORDER:
                printk(KERN_DEBUG "%s: lba:%llu IMR_ERR_READ_BORDER\n", __FUNCTION__, lba);
                imrsim_dbg_rerr = uerr;
                break;
            case IMR_ERR_READ_POINTER: 
                printk(KERN_DEBUG "%s: lba:%llu: IMR_ERR_READ_POINTER\n",__FUNCTION__, lba);
                imrsim_dbg_rerr = uerr;
                break;
            case IMR_ERR_WRITE_RO:
                printk(KERN_DEBUG "%s: lba:%llu: IMR_ERR_WRITE_RO\n", __FUNCTION__, lba);
                imrsim_dbg_werr = uerr;
                break;
            case IMR_ERR_WRITE_POINTER :
                printk(KERN_DEBUG "%s: lba:%llu: IMR_ERR_WRITE_POINTER\n",__FUNCTION__, lba);
                imrsim_dbg_werr = uerr;
                break;
            case IMR_ERR_WRITE_ALIGN :
                printk(KERN_DEBUG "%s: lba:%llu: IMR_ERR_WRITE_ALIGN\n", __FUNCTION__, lba);
                imrsim_dbg_werr = uerr;
                break;
            case IMR_ERR_WRITE_BORDER:
                printk(KERN_DEBUG "%s: lba:%llu: IMR_ERR_WRITE_BORDER\n", __FUNCTION__, lba);
                imrsim_dbg_werr = uerr;
                break;
            case IMR_ERR_WRITE_FULL:
                printk(KERN_DEBUG "%s: lba:%llu: IMR_ERR_WRITE_FULL\n", __FUNCTION__, lba);
                imrsim_dbg_werr = uerr;
                break;
            default:
                printk(KERN_DEBUG "%s: lba:%llu: UNKNOWN ERR=%u\n", __FUNCTION__, lba, uerr);
        }
    }
}

/* The following is the relevant method to build the target_type structure. */
/* device creation */
static int imrsim_ctr(struct dm_target *ti,
                      unsigned int argc,
                      char **argv)
{
    unsigned long long tmp;
    int iRet;
    char dummy;
    struct imrsim_c *c = NULL;
    __u64 num;

    printk(KERN_INFO "imrsim: %s called\n", __FUNCTION__);
    if(imrsim_single){
        printk(KERN_ERR "imrsim: No multiple device support currently\n");
        return -EINVAL;
    }
    if(!ti){
        printk(KERN_ERR "imrsim: error: invalid device\n");
        return -EINVAL;
    }
    if(2 != argc){
        ti->error = "dm-imrsim: error: invalid argument count; !=2";
        return -EINVAL;
    }
    if(1 != sscanf(argv[1], "%llu%c", &tmp, &dummy)){
        ti->error = "dm-imrsim: error: invalid argument device sector";
        return -EINVAL;
    }
    c = kmalloc(sizeof(*c), GFP_KERNEL);    // To allocate physically contiguous memory.
    if(!c){
        ti->error = "dm-imrsim: error: no enough memory";
        return -ENOMEM;
    }
    c->start = tmp;
    // Fill in the bdev of the device specified by path and the corresponding interval, permission, mode, etc. into ti->table.
    iRet = dm_get_device(ti, argv[0], dm_table_get_mode(ti->table), &c->dev);
    if(iRet){
        ti->error = "dm-imrsim: error: device lookup failed";
        kfree(c);
        return iRet;
    }
    if(ti->len > IMR_MAX_CAPACITY){
        printk(KERN_ERR "imrsim: capacity %llu exceeds the maximum 10TB\n", (__u64)ti->len);
        kfree(c);
        return -EINVAL;
    }
    num = ti->len >> IMR_BLOCK_SIZE_SHIFT >> IMR_ZONE_SIZE_SHIFT;
    if((num << IMR_BLOCK_SIZE_SHIFT << IMR_ZONE_SIZE_SHIFT) != ti->len){
        printk(KERN_ERR "imrsim:error: total size must be zone size (256MB) aligned\n");
    }
    if (ti->len < (1 << IMR_BLOCK_SIZE_SHIFT << IMR_ZONE_SIZE_SHIFT)) {
      printk(KERN_INFO "imrsim: capacity: %llu sectors\n", (__u64)ti->len);
      printk(KERN_ERR "imrsim:error: capacity is too small. The default config is multiple of 256MB\n"); 
      kfree(c);
      return -EINVAL;
   }
   ti->num_flush_bios = ti->num_discard_bios = ti->num_write_same_bios = 1;
   ti->private = c;
   imrsim_dbg_rerr = imrsim_dbg_werr = imrsim_dbg_log_enabled = 0;
   mutex_init(&imrsim_zone_lock);
   mutex_init(&imrsim_ioctl_lock);
   // To open a persistent thread.
   if(imrsim_persistence_thread(ti)){
       printk(KERN_ERR "imrsim: error: metadata will not be persisted\n");
   }
   imrsim_single = 1;
   return 0;
}

/* device destory */
static void imrsim_dtr(struct dm_target *ti)
{
    struct imrsim_c *c = (struct imrsim_c *) ti->private;

    kthread_stop(imrsim_ptask.pstore_thread);   // To kill the persistent thread.
    mutex_destroy(&imrsim_zone_lock);
    mutex_destroy(&imrsim_ioctl_lock);
    dm_put_device(ti, c->dev);
    kfree(c);
    vfree(zone_state);
    imrsim_single = 0;
    printk(KERN_INFO "imrsim target destructed\n");
}

/* To get device mapping offset. */
static sector_t imrsim_map_sector(struct dm_target *ti, 
                                  sector_t bi_sector)
{
    struct imrsim_c *c = ti->private;
    return c->start + dm_target_offset(ti, bi_sector);
}

/* Device Write Rules */
int imrsim_write_rule_check(struct bio *bio, __u32 zone_idx,
                            sector_t bio_sectors, int policy_flag)
{
    __u64  lba;
    __u64  lba_offset;    // The offset of lba in the zone
    __u64  block_offset;  // The offset of the block in the zone
    __u64  boundary;
    __u64  elba;
    __u64  zlba;
    __u32  relocateTrackno;   // In a stage allocation, how many tracks are the relocated lba on?
    __u32  rv;       // rule violation
    __u32  z_size;
    __u32  trackno;  // on the top-bottom track group
    __u32  blockno;  // The number of the block corresponding to lba on the track
    __u32  trackrate;  // Track ratio, p.s. linux kernel does not support floating point calculation.
    __u16  wa_penalty;
    __u8   isTopTrack;
    __u8   rewriteSign;
    __u8   ret=1;       // Determine whether the block requested by lba is in the mapping table.

    zlba = zone_idx_lba(zone_idx);

    /* Relocate bio according to phase. */
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0)
    switch(IMR_ALLOCATION_PHASE)
    {
        case 1:
            lba = bio->bi_sector;
            break;
        case 2:
            lba = bio->bi_sector;
            //printk(KERN_INFO "imrsim: request- lba(sectors) is %llu\n", lba);
            lba_offset = bio->bi_sector - zlba;
            block_offset = lba_offset >> IMR_BLOCK_SIZE_SHIFT;
            //Check the mapping table, ret indicates whether the block where lba is located is in the mapping table
            ret = zone_status[zone_idx].z_pba_map[block_offset]!=-1?1:0;
            if(!ret){         // lba is not in the mapping table, indicating a new write operation
                // Fill the mapping table, allocate tracks according to the stage, and write data
                // Note: Multiply TOP_TRACK_NUM_TOTAL because the number of top and bottom tracks in the zone is equal
                boundary = IMR_BOTTOM_TRACK_SIZE * TOP_TRACK_NUM_TOTAL;
                if(zone_status[zone_idx].z_map_size < boundary){
                    // Indicates that the relocated lba should be on the bottom track, in the first stage allocation
                    isTopTrack = 0;
                    // Judgment should be redirected to the first few bottom tracks
                    relocateTrackno = zone_status[zone_idx].z_map_size / IMR_BOTTOM_TRACK_SIZE;
                    // Get the pba corresponding to the bio starting lba
                    bio->bi_sector = zlba  
                        + (((relocateTrackno+1)*(IMR_TOP_TRACK_SIZE)+relocateTrackno*IMR_BOTTOM_TRACK_SIZE)<<IMR_BLOCK_SIZE_SHIFT) 
                        + ((zone_status[zone_idx].z_map_size % IMR_BOTTOM_TRACK_SIZE) << IMR_BLOCK_SIZE_SHIFT);
                    printk(KERN_INFO "imrsim: write_ops(bottom) on zone %u - start LBA is %llu, PBA is %llu\n", zone_idx, lba>>IMR_BLOCK_SIZE_SHIFT, bio->bi_iter.bi_sector>>IMR_BLOCK_SIZE_SHIFT);
                    zone_status[zone_idx].z_pba_map[block_offset] = (bio->bi_sector - zlba) >> IMR_BLOCK_SIZE_SHIFT;
                    zone_status[zone_idx].z_map_size++;
                    lba = bio->bi_sector;
                }else{
                    // Indicates that the relocated lba should be on the top track, in a second stage allocation
                    isTopTrack = 1;
                    relocateTrackno = (zone_status[zone_idx].z_map_size - boundary) / IMR_TOP_TRACK_SIZE;
                    bio->bi_sector = zlba
                        + (relocateTrackno*(IMR_TOP_TRACK_SIZE+IMR_BOTTOM_TRACK_SIZE)<<IMR_BLOCK_SIZE_SHIFT)
                        + (((zone_status[zone_idx].z_map_size - boundary) % IMR_TOP_TRACK_SIZE)<<IMR_BLOCK_SIZE_SHIFT);
                    printk(KERN_INFO "imrsim: write_ops(top) on zone %u - start LBA is %llu, PBA is %llu\n", zone_idx, lba>>IMR_BLOCK_SIZE_SHIFT, bio->bi_iter.bi_sector>>IMR_BLOCK_SIZE_SHIFT);
                    zone_status[zone_idx].z_pba_map[block_offset] = (bio->bi_sector - zlba) >> IMR_BLOCK_SIZE_SHIFT;
                    zone_status[zone_idx].z_map_size++;
                    lba = bio->bi_sector;
                }
            }else{            // lba is in the mapping table, indicating an update operation
                // Get pba from the mapping table, modify lba in bio
                bio->bi_sector = zlba + (zone_status[zone_idx].z_pba_map[block_offset] << IMR_BLOCK_SIZE_SHIFT);
                printk(KERN_INFO "imrsim: update_ops on zone %u - start LBA is %llu, PBA is %llu\n", zone_idx, lba>>IMR_BLOCK_SIZE_SHIFT, bio->bi_iter.bi_sector>>IMR_BLOCK_SIZE_SHIFT);
                lba = bio->bi_sector;
            }
            break;
        case 3:
            lba = bio->bi_sector;
            lba_offset = bio->bi_sector - zlba;
            block_offset = lba_offset >> IMR_BLOCK_SIZE_SHIFT;
            ret = zone_status[zone_idx].z_pba_map[block_offset]!=-1?1:0;
            if(!ret){         // a new write operation
                boundary = IMR_BOTTOM_TRACK_SIZE * TOP_TRACK_NUM_TOTAL;
                __u32 mapSize = zone_status[zone_idx].z_map_size;
                if(mapSize < boundary){
                    // first stage allocation
                    isTopTrack = 0;
                    relocateTrackno = mapSize / IMR_BOTTOM_TRACK_SIZE;
                    bio->bi_sector = zlba  
                        + (((relocateTrackno+1)*(IMR_TOP_TRACK_SIZE)+relocateTrackno*IMR_BOTTOM_TRACK_SIZE)<<IMR_BLOCK_SIZE_SHIFT) 
                        + ((mapSize % IMR_BOTTOM_TRACK_SIZE) << IMR_BLOCK_SIZE_SHIFT);
                    printk(KERN_INFO "imrsim: write_ops_3(bottom) - start LBA is %llu, PBA is %llu\n", lba>>IMR_BLOCK_SIZE_SHIFT, bio->bi_iter.bi_sector>>IMR_BLOCK_SIZE_SHIFT);
                    zone_status[zone_idx].z_pba_map[block_offset] = (bio->bi_sector - zlba) >> IMR_BLOCK_SIZE_SHIFT;
                    zone_status[zone_idx].z_map_size++;
                    lba = bio->bi_sector;
                }else if(mapSize >= boundary 
                    && mapSize < boundary + IMR_TOP_TRACK_SIZE*TOP_TRACK_NUM_TOTAL/2){
                    // In second stage allocation Top(0,2,4,...)
                    isTopTrack = 1;
                    relocateTrackno = 2*((mapSize - boundary) / IMR_TOP_TRACK_SIZE);
                    bio->bi_sector = zlba
                        + (relocateTrackno*(IMR_TOP_TRACK_SIZE+IMR_BOTTOM_TRACK_SIZE)<<IMR_BLOCK_SIZE_SHIFT)
                        + (((mapSize - boundary) % IMR_TOP_TRACK_SIZE) << IMR_BLOCK_SIZE_SHIFT);
                    printk(KERN_INFO "imrsim: write_ops_3(top_1) - start LBA is %llu, PBA is %llu\n", lba>>IMR_BLOCK_SIZE_SHIFT, bio->bi_iter.bi_sector>>IMR_BLOCK_SIZE_SHIFT);
                    zone_status[zone_idx].z_pba_map[block_offset] = (bio->bi_sector - zlba) >> IMR_BLOCK_SIZE_SHIFT;
                    zone_status[zone_idx].z_map_size++;
                    lba = bio->bi_sector;
                }else{
                    // In the third stage allocation Top(1,3,5,...)
                    isTopTrack = 1;
                    relocateTrackno = 2*((mapSize - boundary - IMR_TOP_TRACK_SIZE*TOP_TRACK_NUM_TOTAL/2) 
                                    / IMR_TOP_TRACK_SIZE) + 1;
                    bio->bi_sector = zlba
                        + (relocateTrackno*(IMR_TOP_TRACK_SIZE+IMR_BOTTOM_TRACK_SIZE)<<IMR_BLOCK_SIZE_SHIFT)
                        + (((mapSize - boundary - IMR_TOP_TRACK_SIZE*TOP_TRACK_NUM_TOTAL/2) % IMR_TOP_TRACK_SIZE)<<IMR_BLOCK_SIZE_SHIFT);
                    printk(KERN_INFO "imrsim: write_ops_3(top_2) - start LBA is %llu, PBA is %llu\n", lba>>IMR_BLOCK_SIZE_SHIFT, bio->bi_iter.bi_sector);
                    zone_status[zone_idx].z_pba_map[block_offset] = (bio->bi_sector - zlba) >> IMR_BLOCK_SIZE_SHIFT;
                    zone_status[zone_idx].z_map_size++;
                    lba = bio->bi_sector;
                }
            }else{            // an update operation
                bio->bi_sector = zlba + (zone_status[zone_idx].z_pba_map[block_offset] << IMR_BLOCK_SIZE_SHIFT);
                printk(KERN_INFO "imrsim: update_ops - start lba is %llu, pba is %llu\n", lba, bio->bi_sector);
                lba = bio->bi_sector;
            }
            break;
        default:
            printk(KERN_ERR "imrsim: error: Allocation of more phases is not currently supported!\n");
    }
    #else
    switch(IMR_ALLOCATION_PHASE)
    {
        case 1:
            lba = bio->bi_iter.bi_sector;
            break;
        case 2:
            lba = bio->bi_iter.bi_sector;
            //printk(KERN_INFO "imrsim: request- lba(sectors) is %llu\n", lba);
            lba_offset = bio->bi_iter.bi_sector - zlba;
            block_offset = lba_offset >> IMR_BLOCK_SIZE_SHIFT;
            //Check the mapping table, ret indicates whether the block where lba is located is in the mapping table
            ret = zone_status[zone_idx].z_pba_map[block_offset]!=-1?1:0;
            if(!ret){         // lba is not in the mapping table, indicating a new write operation
                // Fill the mapping table, allocate tracks according to the stage, and write data
                // Note: Multiply TOP_TRACK_NUM_TOTAL because the number of top and bottom tracks in the zone is equal
                boundary = IMR_BOTTOM_TRACK_SIZE * TOP_TRACK_NUM_TOTAL;
                if(zone_status[zone_idx].z_map_size < boundary){
                    // Indicates that the relocated lba should be on the bottom track, in the first stage allocation
                    isTopTrack = 0;
                    // Judgment should be redirected to the first few bottom tracks
                    relocateTrackno = zone_status[zone_idx].z_map_size / IMR_BOTTOM_TRACK_SIZE;
                    // Get the pba corresponding to the bio starting lba
                    bio->bi_iter.bi_sector = zlba  
                        + (((relocateTrackno+1)*(IMR_TOP_TRACK_SIZE)+relocateTrackno*IMR_BOTTOM_TRACK_SIZE)<<IMR_BLOCK_SIZE_SHIFT) 
                        + ((zone_status[zone_idx].z_map_size % IMR_BOTTOM_TRACK_SIZE) << IMR_BLOCK_SIZE_SHIFT);
                    printk(KERN_INFO "imrsim: write_ops(bottom) on zone %u - start LBA is %llu, PBA is %llu\n", zone_idx, lba>>IMR_BLOCK_SIZE_SHIFT, bio->bi_iter.bi_sector>>IMR_BLOCK_SIZE_SHIFT);
                    zone_status[zone_idx].z_pba_map[block_offset] = (bio->bi_iter.bi_sector - zlba) >> IMR_BLOCK_SIZE_SHIFT;
                    zone_status[zone_idx].z_map_size++;
                    lba = bio->bi_iter.bi_sector;
                }else{
                    // Indicates that the relocated lba should be on the top track, in a second stage allocation
                    isTopTrack = 1;
                    relocateTrackno = (zone_status[zone_idx].z_map_size - boundary) / IMR_TOP_TRACK_SIZE;
                    bio->bi_iter.bi_sector = zlba
                        + (relocateTrackno*(IMR_TOP_TRACK_SIZE+IMR_BOTTOM_TRACK_SIZE)<<IMR_BLOCK_SIZE_SHIFT)
                        + (((zone_status[zone_idx].z_map_size - boundary) % IMR_TOP_TRACK_SIZE)<<IMR_BLOCK_SIZE_SHIFT);
                    printk(KERN_INFO "imrsim: write_ops(top) on zone %u - start LBA is %llu, PBA is %llu\n", zone_idx, lba>>IMR_BLOCK_SIZE_SHIFT, bio->bi_iter.bi_sector>>IMR_BLOCK_SIZE_SHIFT);
                    zone_status[zone_idx].z_pba_map[block_offset] = (bio->bi_iter.bi_sector - zlba) >> IMR_BLOCK_SIZE_SHIFT;
                    zone_status[zone_idx].z_map_size++;
                    lba = bio->bi_iter.bi_sector;
                }
            }else{            // lba is in the mapping table, indicating an update operation
                // Get pba from the mapping table, modify lba in bio
                bio->bi_iter.bi_sector = zlba + (zone_status[zone_idx].z_pba_map[block_offset] << IMR_BLOCK_SIZE_SHIFT);
                printk(KERN_INFO "imrsim: update_ops on zone %u - start LBA is %llu, PBA is %llu\n", zone_idx, lba>>IMR_BLOCK_SIZE_SHIFT, bio->bi_iter.bi_sector>>IMR_BLOCK_SIZE_SHIFT);
                lba = bio->bi_iter.bi_sector;
            }
            break;
        case 3:
            lba = bio->bi_iter.bi_sector;
            lba_offset = bio->bi_iter.bi_sector - zlba;
            block_offset = lba_offset >> IMR_BLOCK_SIZE_SHIFT;
            ret = zone_status[zone_idx].z_pba_map[block_offset]!=-1?1:0;
            if(!ret){         // a new write operation
                boundary = IMR_BOTTOM_TRACK_SIZE * TOP_TRACK_NUM_TOTAL;
                __u32 mapSize = zone_status[zone_idx].z_map_size;
                if(mapSize < boundary){
                    // first stage allocation
                    isTopTrack = 0;
                    relocateTrackno = mapSize / IMR_BOTTOM_TRACK_SIZE;
                    bio->bi_iter.bi_sector = zlba  
                        + (((relocateTrackno+1)*(IMR_TOP_TRACK_SIZE)+relocateTrackno*IMR_BOTTOM_TRACK_SIZE)<<IMR_BLOCK_SIZE_SHIFT) 
                        + ((mapSize % IMR_BOTTOM_TRACK_SIZE) << IMR_BLOCK_SIZE_SHIFT);
                    printk(KERN_INFO "imrsim: write_ops_3(bottom) - start LBA is %llu, PBA is %llu\n", lba>>IMR_BLOCK_SIZE_SHIFT, bio->bi_iter.bi_sector>>IMR_BLOCK_SIZE_SHIFT);
                    zone_status[zone_idx].z_pba_map[block_offset] = (bio->bi_iter.bi_sector - zlba) >> IMR_BLOCK_SIZE_SHIFT;
                    zone_status[zone_idx].z_map_size++;
                    lba = bio->bi_iter.bi_sector;
                }else if(mapSize >= boundary 
                    && mapSize < boundary + IMR_TOP_TRACK_SIZE*TOP_TRACK_NUM_TOTAL/2){
                    // In second stage allocation Top(0,2,4,...)
                    isTopTrack = 1;
                    relocateTrackno = 2*((mapSize - boundary) / IMR_TOP_TRACK_SIZE);
                    bio->bi_iter.bi_sector = zlba
                        + (relocateTrackno*(IMR_TOP_TRACK_SIZE+IMR_BOTTOM_TRACK_SIZE)<<IMR_BLOCK_SIZE_SHIFT)
                        + (((mapSize - boundary) % IMR_TOP_TRACK_SIZE) << IMR_BLOCK_SIZE_SHIFT);
                    printk(KERN_INFO "imrsim: write_ops_3(top_1) - start LBA is %llu, PBA is %llu\n", lba>>IMR_BLOCK_SIZE_SHIFT, bio->bi_iter.bi_sector>>IMR_BLOCK_SIZE_SHIFT);
                    zone_status[zone_idx].z_pba_map[block_offset] = (bio->bi_iter.bi_sector - zlba) >> IMR_BLOCK_SIZE_SHIFT;
                    zone_status[zone_idx].z_map_size++;
                    lba = bio->bi_iter.bi_sector;
                }else{
                    // In the third stage allocation Top(1,3,5,...)
                    isTopTrack = 1;
                    relocateTrackno = 2*((mapSize - boundary - IMR_TOP_TRACK_SIZE*TOP_TRACK_NUM_TOTAL/2) 
                                    / IMR_TOP_TRACK_SIZE) + 1;
                    bio->bi_iter.bi_sector = zlba
                        + (relocateTrackno*(IMR_TOP_TRACK_SIZE+IMR_BOTTOM_TRACK_SIZE)<<IMR_BLOCK_SIZE_SHIFT)
                        + (((mapSize - boundary - IMR_TOP_TRACK_SIZE*TOP_TRACK_NUM_TOTAL/2) % IMR_TOP_TRACK_SIZE)<<IMR_BLOCK_SIZE_SHIFT);
                    printk(KERN_INFO "imrsim: write_ops_3(top_2) - start LBA is %llu, PBA is %llu\n", lba>>IMR_BLOCK_SIZE_SHIFT, bio->bi_iter.bi_sector);
                    zone_status[zone_idx].z_pba_map[block_offset] = (bio->bi_iter.bi_sector - zlba) >> IMR_BLOCK_SIZE_SHIFT;
                    zone_status[zone_idx].z_map_size++;
                    lba = bio->bi_iter.bi_sector;
                }
            }else{            // an update operation
                bio->bi_iter.bi_sector = zlba + (zone_status[zone_idx].z_pba_map[block_offset] << IMR_BLOCK_SIZE_SHIFT);
                printk(KERN_INFO "imrsim: update_ops - start lba is %llu, pba is %llu\n", lba, bio->bi_iter.bi_sector);
                lba = bio->bi_iter.bi_sector;
            }
            break;
        default:
            printk(KERN_ERR "imrsim: error: Allocation of more phases is not currently supported!\n");
    }
    #endif
    /* relocate bio end */
    
    rv = 0;
    elba = lba + bio_sectors;
    z_size = num_sectors_zone();

    if ((policy_flag == 1) && (zone_status[zone_idx].z_conds == Z_COND_FULL)) {
        zone_status[zone_idx].z_conds = Z_COND_CLOSED;     
    } 
    if (imrsim_dbg_log_enabled && printk_ratelimit()) {
        printk(KERN_INFO "imrsim write PASS\n");
    }
    if (rv && (policy_flag ==1)) {
        printk(KERN_ERR "imrsim: out of policy passed rule violation: %u\n", rv); 
        return IMR_ERR_OUT_OF_POLICY;
    }
    //printk(KERN_INFO "imrsim: %s called! lba: %llu, zlba: %llu ~~\n", __FUNCTION__, lba, zlba);

    trackno = (lba - zlba) / ((IMR_TOP_TRACK_SIZE + IMR_BOTTOM_TRACK_SIZE) << 
                IMR_BLOCK_SIZE_SHIFT);
    // If it is a new write operation, there is no need to judge isTopTrack
    if(ret){
        isTopTrack = (lba - (zlba + (trackno * (IMR_TOP_TRACK_SIZE + IMR_BOTTOM_TRACK_SIZE) <<
                    IMR_BLOCK_SIZE_SHIFT))) < (IMR_TOP_TRACK_SIZE << IMR_BLOCK_SIZE_SHIFT) ? 1 : 0;
    }
    printk(KERN_INFO "imrsim: %s trackno: %u, isTopTrack: %u.\n",__FUNCTION__, trackno, isTopTrack);
    // If lba is on the top track, mark the top track with data, and on the bottom track, determine whether to rewrite
    if(isTopTrack){
        blockno = (lba - (zlba + (trackno * (IMR_TOP_TRACK_SIZE + IMR_BOTTOM_TRACK_SIZE) <<
                IMR_BLOCK_SIZE_SHIFT))) >> IMR_BLOCK_SIZE_SHIFT;
        zone_status[zone_idx].z_tracks[trackno].isUsedBlock[blockno]=1;
        //printk(KERN_INFO "imrsim: SIGN - block is remember\n");
    }else{
        wa_penalty=0;
        rewriteSign=0;
        blockno = ((lba - (zlba + (trackno * (IMR_TOP_TRACK_SIZE + IMR_BOTTOM_TRACK_SIZE) <<
                IMR_BLOCK_SIZE_SHIFT))) >> IMR_BLOCK_SIZE_SHIFT) - IMR_TOP_TRACK_SIZE;
        trackrate = IMR_BOTTOM_TRACK_SIZE * 10000 / IMR_TOP_TRACK_SIZE;
        int wa_pba1=-1,wa_pba2=-1;
        imrsim_rmw_task.lba_num=0;
        if(trackno>=0 && zone_status[zone_idx].z_tracks[trackno].isUsedBlock[(__u32)(blockno*10000/trackrate)]==1){
            printk(KERN_INFO "imrsim: write amplification(zone_idx[%u]trackno), block: %u .\n",zone_idx, (__u32)(blockno*10000/trackrate));
            zone_state->stats.zone_stats[zone_idx].rewrite_total++;
            rewriteSign++;
            wa_penalty += (zone_state->config.dev_config.r_time_to_rmw_zone + 
                zone_state->config.dev_config.w_time_to_rmw_zone + 
                2*IMR_ROTATE_PENALTY);
            lba = zlba + (trackno * (IMR_TOP_TRACK_SIZE + IMR_BOTTOM_TRACK_SIZE) <<IMR_BLOCK_SIZE_SHIFT) 
                    + ((__u32)(blockno*10000/trackrate) <<IMR_BLOCK_SIZE_SHIFT);
            //imrsim_rmw_task.lba[imrsim_rmw_task.lba_num] = lba;
            //imrsim_rmw_task.lba_num++;
            wa_pba1=lba>>IMR_BLOCK_SIZE_SHIFT;
        }
        if(trackno+1<TOP_TRACK_NUM_TOTAL && zone_status[zone_idx].z_tracks[trackno+1].isUsedBlock[(__u32)(blockno*10000/trackrate)]==1){
            printk(KERN_INFO "imrsim: write amplification(trackno+1), block: %u .\n", (__u32)(blockno*10000/trackrate));
            zone_state->stats.zone_stats[zone_idx].rewrite_total++;
            rewriteSign++;
            wa_penalty += (zone_state->config.dev_config.r_time_to_rmw_zone + 
                zone_state->config.dev_config.w_time_to_rmw_zone + 
                2*IMR_ROTATE_PENALTY);
            lba = zlba + ((trackno+1) * (IMR_TOP_TRACK_SIZE + IMR_BOTTOM_TRACK_SIZE) <<IMR_BLOCK_SIZE_SHIFT) 
                    + ((__u32)(blockno*10000/trackrate) <<IMR_BLOCK_SIZE_SHIFT);
            //imrsim_rmw_task.lba[imrsim_rmw_task.lba_num] = lba;
            //imrsim_rmw_task.lba_num++;
            wa_pba2=lba>>IMR_BLOCK_SIZE_SHIFT;
        }
        // write zoom time penalty
        if(wa_penalty != 0){
            udelay(wa_penalty);
        }

        if(1 == rewriteSign){
            zone_state->stats.zone_stats[zone_idx].rewrite_single++;
        }
        if(1 <= rewriteSign){
            printk(KERN_INFO "imrsim: WA, wa_pba_1:%d,wa_pba_2:%d.\n", wa_pba1, wa_pba2);
        }
        //imrsim_rmw_process(bio);
    }
    return 0;
}

/* Device Read Rules */
int imrsim_read_rule_check(struct bio *bio, __u32 zone_idx, 
                           sector_t bio_sectors, int policy_flag)
{
    __u64 lba;
    __u64 zlba;
    __u64 elba;
    __u32 rv;
    __u8 ret;

    zlba = zone_idx_lba(zone_idx);

    #if LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0)
    lba = bio->bi_sector;
    __u32 block_offset = (lba-zlba)>>IMR_BLOCK_SIZE_SHIFT;
    ret = zone_status[zone_idx].z_pba_map[block_offset]!=-1?1:0;
    if(ret){
        bio->bi_sector = zlba 
            + (zone_status[zone_idx].z_pba_map[block_offset] << IMR_BLOCK_SIZE_SHIFT)
            + (lba-zlba)%(1<<IMR_BLOCK_SIZE_SHIFT);
        printk(KERN_INFO "imrsim: read_ops on zone %u - start LBA is %llu, PBA is %llu\n", zone_idx, lba>>IMR_BLOCK_SIZE_SHIFT, bio->bi_iter.bi_sector>>IMR_BLOCK_SIZE_SHIFT); 
        lba = bio->bi_sector;
    }else{
        rv++;
        //printk(KERN_ERR "imrsim: read none data\n"); 
    }
    #else
    lba = bio->bi_iter.bi_sector;
    __u32 block_offset = (lba-zlba)>>IMR_BLOCK_SIZE_SHIFT;
    ret = zone_status[zone_idx].z_pba_map[block_offset]!=-1?1:0;
    if(ret){
        bio->bi_iter.bi_sector = zlba 
            + (zone_status[zone_idx].z_pba_map[block_offset] << IMR_BLOCK_SIZE_SHIFT)
            + (lba-zlba)%(1<<IMR_BLOCK_SIZE_SHIFT);
        printk(KERN_INFO "imrsim: read_ops on zone %u - start LBA is %llu, PBA is %llu\n", zone_idx, lba>>IMR_BLOCK_SIZE_SHIFT, bio->bi_iter.bi_sector>>IMR_BLOCK_SIZE_SHIFT); 
        lba = bio->bi_iter.bi_sector;
    }else{
        rv++;
        //printk(KERN_ERR "imrsim: read none data\n"); 
    }
    #endif
    rv = 0;
    elba = lba + bio_sectors;
    
    if(elba > (zlba + num_sectors_zone())){
        printk(KERN_ERR "imrsim: error: read across zone: %u.%012llx.%08lx\n",
               zone_idx, lba, bio_sectors);
        rv++;
        zone_state->stats.zone_stats[zone_idx].out_of_policy_read_stats.span_zones_count++;
        imrsim_log_error(bio, IMR_ERR_READ_BORDER);
        if(!policy_flag){
            return IMR_ERR_READ_BORDER;
        }
        printk(KERN_ERR "imrsim:error: out of policy allowed pass\n");
    }
  
    if (imrsim_dbg_log_enabled && printk_ratelimit()) {
        printk(KERN_INFO "imrsim read PASS\n");
    }
    if (rv) {
        printk(KERN_ERR "imrsim: out of policy passed rule violation: %u\n", rv); 
        return IMR_ERR_OUT_OF_POLICY;
    }
    return 0;
}

static bool imrsim_ptask_queue_ok(__u32 idx)
{
   __u32 qidx;
   
    for (qidx = 0; qidx < imrsim_ptask.stu_zone_idx_cnt; qidx++) {
       if (abs(idx - (imrsim_ptask.stu_zone_idx[qidx]))
          <= IMR_PSTORE_PG_EDG) {
          return false;
       }
    }
    return true;
}

static bool imrsim_ptask_gap_ok(__u32 idx)
{
   __u32 qidx;
   
    for (qidx = 0; qidx < imrsim_ptask.stu_zone_idx_cnt; qidx++) {
       if (abs(idx - (imrsim_ptask.stu_zone_idx[qidx]))
          <= IMR_PSTORE_PG_GAP * IMR_PSTORE_PG_EDG) {
          return true;
       }
    }
    return false;
}

/* I/O mapping */
int imrsim_map(struct dm_target *ti, struct bio *bio)
{
    struct imrsim_c *c = ti->private;
    int cdir = bio_data_dir(bio);      // 通过宏来获得请求的读写方向

    sector_t bio_sectors = bio_sectors(bio);
    int policy_rflag = 0;
    int policy_wflag = 0;
    int ret = 0;
    unsigned int penalty;
    __u32 zone_idx;
    __u64 lba;

    mutex_lock(&imrsim_zone_lock);
    //printk(KERN_INFO "zone_lock.\n");
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0)
    zone_idx = bio->bi_sector >> IMR_BLOCK_SIZE_SHIFT >> IMR_ZONE_SIZE_SHIFT;
    lba = bio->bi_sector;
    #else
    zone_idx = bio->bi_iter.bi_sector >> IMR_BLOCK_SIZE_SHIFT >> IMR_ZONE_SIZE_SHIFT;
    lba = bio->bi_iter.bi_sector;
    #endif

    //printk(KERN_INFO "imrsim: map- lba is %llu\n", lba);

    imrsim_dev_idle_update();

    if(IMR_NUMZONES <= zone_idx){
        printk(KERN_ERR "imrsim: lba is out of range. zone_idx: %u\n", zone_idx);
        imrsim_log_error(bio, IMR_ERR_OUT_RANGE);
        goto nomap;
    }
    if(imrsim_dbg_log_enabled){
        printk(KERN_DEBUG "imrsim: %s bio_sectors=%llu\n", __FUNCTION__, 
                (unsigned long long)bio_sectors);
    }
    if((lba + bio_sectors) > (zone_idx_lba(zone_idx) + 2 * num_sectors_zone())){
        printk(KERN_ERR "imrsim: error: %s bio_sectors() is too large\n", __FUNCTION__);
        imrsim_log_error(bio, IMR_ERR_OUT_OF_POLICY);
        goto nomap;
    }
    if(zone_status[zone_idx].z_conds == Z_COND_OFFLINE){
        printk(KERN_ERR "imrsim: error: zone is offline. zone_idx:%u\n", zone_idx);
        imrsim_log_error(bio, IMR_ERR_ZONE_OFFLINE);
        goto nomap;
    }
    bio->bi_bdev = c->dev->bdev;
    policy_rflag = zone_state->config.dev_config.out_of_policy_read_flag;
    policy_wflag = zone_state->config.dev_config.out_of_policy_write_flag;
    
    // 判断是读请求还是写请求
    if(cdir == WRITE){
        if(imrsim_dbg_log_enabled){
            printk(KERN_DEBUG "imrsim: %s WRITE %u.%012llx:%08lx.\n", __FUNCTION__,
                zone_idx, lba, bio_sectors);
        }
        if ((zone_status[zone_idx].z_conds == Z_COND_RO) && !policy_wflag) {
            printk(KERN_ERR "imrsim:error: zone is read only. zone_idx: %u\n", zone_idx);  
            imrsim_log_error(bio, IMR_ERR_WRITE_RO);
            goto nomap;
        }
        if ((zone_status[zone_idx].z_conds == Z_COND_FULL) &&
            (lba != zone_idx_lba(zone_idx)) && !policy_wflag) {
            printk(KERN_ERR "imrsim:error: zone is full. zone_idx: %u\n", zone_idx);
            imrsim_log_error(bio, IMR_ERR_WRITE_FULL);
            goto nomap;
        }
        ret = imrsim_write_rule_check(bio, zone_idx, bio_sectors, policy_wflag);
        if(ret){
            if(policy_wflag == 1 && policy_rflag == 1){
                goto mapped;
            }
            penalty = 0;
            if(policy_wflag == 1){
                penalty = zone_state->config.dev_config.w_time_to_rmw_zone;
                printk(KERN_ERR "imrsim: %s: write error passed: out of policy write flagged on\n", __FUNCTION__);
                udelay(penalty);
            }else{
                goto nomap;
            }
        }
        imrsim_ptask.flag |= IMR_STATUS_CHANGE;
        if(imrsim_ptask.stu_zone_idx_cnt == IMR_PSTORE_QDEPTH){
            imrsim_ptask.stu_zone_idx_gap = IMR_PSTORE_PG_GAP;
        }else if(imrsim_ptask_queue_ok(zone_idx)){
            imrsim_ptask.stu_zone_idx[imrsim_ptask.stu_zone_idx_cnt] = zone_idx;
            imrsim_ptask.stu_zone_idx_cnt++;
            if(!imrsim_ptask_gap_ok(zone_idx)){
                imrsim_ptask.stu_zone_idx_gap++;
            }
        }
    }
    else if(cdir == READ){
        if (imrsim_dbg_log_enabled) {
            printk(KERN_DEBUG "imrsim: %s READ %u.%012llx:%08lx.\n", __FUNCTION__,
                    zone_idx, lba, bio_sectors);
        }
        ret = imrsim_read_rule_check(bio, zone_idx, bio_sectors, policy_rflag);
        if(ret){
            if(policy_wflag == 1 && policy_rflag == 1){
                printk(KERN_ERR "imrsim: out of policy read passthrough applied\n");
                goto mapped;
            }
            penalty = 0;
            if(policy_rflag == 1){
                penalty = zone_state->config.dev_config.r_time_to_rmw_zone;
                if(printk_ratelimit()){
                    printk(KERN_ERR "imrsim:%s: read error passed: out of policy read flagged on\n", 
                  __FUNCTION__);
                }
                udelay(penalty);
            }else{
                goto nomap;
            }
        }
    }
    mapped:
    if (bio_sectors(bio))
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0)
        bio->bi_sector =  c->start + dm_target_offset(ti, bio->bi_sector);
    #else
        bio->bi_iter.bi_sector =  c->start + dm_target_offset(ti, bio->bi_iter.bi_sector);
    #endif
    mutex_unlock(&imrsim_zone_lock);
    //printk(KERN_INFO "zone_unlock.\n");
    return DM_MAPIO_REMAPPED;

    nomap:
    imrsim_ptask.flag |= IMR_STATS_CHANGE;
    imrsim_ptask.sts_zone_idx = zone_idx;
    mutex_unlock(&imrsim_zone_lock);
    //printk(KERN_INFO "zone_unlock.\n");
    return IMR_DM_IO_ERR;
}

/* Device status query */
static void imrsim_status(struct dm_target* ti, 
                          status_type_t type,
                          unsigned status_flags, 
                          char* result,
                          unsigned maxlen)
{
   struct imrsim_c* c   = ti->private;

   switch(type)
   {
      case STATUSTYPE_INFO:
         result[0] = '\0';
         break;

      case STATUSTYPE_TABLE:
         snprintf(result, maxlen, "%s %llu", c->dev->name,
	    (unsigned long long)c->start);
         break;
   }
}

/* Present zone status information */
static void imrsim_list_zone_status(struct imrsim_zone_status *ptr, 
                                    __u32 num_zones, int criteria)
{
   __u32 i = 0;
   printk(KERN_DEBUG "\nQuery ceiteria: %d\n", criteria);
   printk(KERN_DEBUG "List zone status of %u zones:\n\n", num_zones);
   for (i = 0; i < num_zones; i++) {
       printk(KERN_DEBUG "zone index        : %lu\n", (long unsigned)ptr[i].z_start);
       printk(KERN_DEBUG "zone length       : %u\n",  ptr[i].z_length);
       printk(KERN_DEBUG "zone type         : 0x%x\n", ptr[i].z_type);
       printk(KERN_DEBUG "zone condition    : 0x%x\n", ptr[i].z_conds);
       printk(KERN_DEBUG "\n");
   }
}

/* Query zone status information and record the result in ptr */
int imrsim_query_zones(sector_t lba, int criteria,
                       __u32 *num_zones, struct imrsim_zone_status *ptr)
{
    int idx32;
    __u32 num32;
    __u32 zone_idx;

    if(!num_zones || !ptr){
        printk(KERN_ERR "imrsim: NULL pointer passed through.\n");
        return -EINVAL;
    }
    mutex_lock(&imrsim_zone_lock);
    zone_idx = lba >> IMR_BLOCK_SIZE_SHIFT >> IMR_ZONE_SIZE_SHIFT;
    if(0 == *num_zones || IMR_NUMZONES < (*num_zones + zone_idx)){
        mutex_unlock(&imrsim_zone_lock);
        printk(KERN_ERR "imrsim: number of zone out of range\n");
        return -EINVAL;
    }
    if (imrsim_dbg_log_enabled) {   
        imrsim_list_zone_status(zone_status, *num_zones, criteria);
    }
    if(criteria > 0){
        idx32 = 0; 
        for (num32 = 0; num32 < *num_zones; num32++) {
            memcpy((ptr + idx32), &zone_status[zone_idx + num32], 
                sizeof(struct imrsim_zone_status));
            idx32++;
        }
        *num_zones = idx32;
        mutex_unlock(&imrsim_zone_lock);
        return 0;  
    }
    switch(criteria){
        case ZONE_MATCH_ALL:
            memcpy(ptr, &zone_status[zone_idx], *num_zones * 
                sizeof(struct imrsim_zone_status));
            break;
        case ZONE_MATCH_FULL:
            idx32 = 0; 
            for (num32 = zone_idx; num32 < IMR_NUMZONES; num32++) {
                if (Z_COND_FULL == zone_status[num32].z_conds) {
                    memcpy((ptr + idx32), &zone_status[num32], 
                            sizeof(struct imrsim_zone_status));
                    idx32++;
                    if (idx32 == *num_zones) {
                        break;
                    }
                }
            }
            *num_zones = idx32;
            break;
        case ZONE_MATCH_NFULL:
            idx32 = 0;
            for (num32 = zone_idx; num32 < IMR_NUMZONES; num32++) {
                memcpy((ptr + idx32), &zone_status[num32], 
                        sizeof(struct imrsim_zone_status));
                idx32++;
                if (idx32 == *num_zones) {
                    break;
                }
            }
            *num_zones = idx32;
            break;
        case ZONE_MATCH_FREE:
            idx32 = 0;
            for (num32 = zone_idx; num32 < IMR_NUMZONES; num32++) {
                if ((Z_COND_EMPTY == zone_status[num32].z_conds)) {
                    memcpy((ptr + idx32), &zone_status[num32], 
                            sizeof(struct imrsim_zone_status));
                    idx32++;
                    if (idx32 == *num_zones) {
                        break;
                    }
                }
            }
            *num_zones = idx32;
            break;
        case ZONE_MATCH_RNLY:
            idx32 = 0;
            for (num32 = zone_idx; num32 < IMR_NUMZONES; num32++) {
                if (Z_COND_RO == zone_status[num32].z_conds) {
                memcpy((ptr + idx32), &zone_status[num32], 
                        sizeof(struct imrsim_zone_status));
                idx32++;
                if (idx32 == *num_zones) {
                    break;
                }
                }
            }
            *num_zones = idx32;
            break;
        case ZONE_MATCH_OFFL:
            idx32 = 0;
            for (num32 = zone_idx; num32 < IMR_NUMZONES; num32++) {
                if (Z_COND_OFFLINE == zone_status[num32].z_conds) {
                memcpy((ptr + idx32), &zone_status[num32], 
                        sizeof(struct imrsim_zone_status));
                idx32++;
                if (idx32 == *num_zones) {
                    break;
                }
                }
            }
            *num_zones = idx32;
            break;
        default:
            printk("imrsim: wrong query parameter\n");
    }
    mutex_unlock(&imrsim_zone_lock);
   return 0;
}
EXPORT_SYMBOL(imrsim_query_zones);

/* The ioctl interface method implements specific interface functions. */
int imrsim_ioctl(struct dm_target *ti,
                 unsigned int cmd,
                 unsigned long arg)
{
    imrsim_zbc_query          *zbc_query;
    struct imrsim_dev_config   pconf;
    //struct imrsim_zone_status  pstatus;
    struct imrsim_stats       *pstats;
    int                        ret = 0;
    __u32                      size  = 0;
    __u64                      num64;
    __u32                      param = IMR_NUMZONES;
    
    imrsim_dev_idle_update();
    mutex_lock(&imrsim_ioctl_lock);
    switch(cmd)
    {
        case IOCTL_IMRSIM_GET_LAST_RERROR:
            if(imrsim_get_last_rd_error(&param)){
                printk(KERN_ERR "imrsim: get last rd error failed\n");
                goto ioerr;
            }
            if((__u64)arg == 0){
                printk(KERN_ERR "imrsim: bad parameter\n");
                goto ioerr;
            }
            if(copy_to_user((__u32 *)arg, &param, sizeof(__u32) )){
                printk(KERN_ERR "imrsim: copy last rd error to user memory failed\n");
                goto ioerr;
            }
            break;
        case IOCTL_IMRSIM_GET_LAST_WERROR:
            if(imrsim_get_last_wd_error(&param)){
                printk(KERN_ERR "imrsim: get last wd error failed\n");
                goto ioerr;
            }
            if((__u64)arg == 0){
                printk(KERN_ERR "imrsim: bad parameter\n");
                goto ioerr;
            }
            if(copy_to_user((__u32 *)arg, &param, sizeof(__u32) )){
                printk(KERN_ERR "imrsim: copy last wd error to user memory failed\n");
                goto ioerr;
            }
            break;
        /* zone ioctl */
        case IOCTL_IMRSIM_SET_LOGENABLE:
            if(imrsim_set_log_enable(1)){
                printk(KERN_ERR "imrsim: enable log failed\n");
                goto ioerr;
            }
            break;
        case IOCTL_IMRSIM_SET_LOGDISABLE:
            if(imrsim_set_log_enable(0)){
                printk(KERN_ERR "imrsim: disable log failed\n");
                goto ioerr;
            }
            break;
        case IOCTL_IMRSIM_GET_NUMZONES:
            if(imrsim_get_num_zones(&param)){
                printk(KERN_ERR "imrsim: get number of zones failed\n");
                goto ioerr;
            }
            if((__u64)arg == 0){
                printk(KERN_ERR "imrsim: bad parameter\n");
                goto ioerr;
            }
            if(copy_to_user((__u32 *)arg, &param, sizeof(__u32) )){
                printk(KERN_ERR "imrsim: copy num of zones to user memory failed\n");
                goto ioerr;
            }
            break;
        case IOCTL_IMRSIM_GET_SIZZONEDEFAULT:
            if(imrsim_get_size_zone_default(&param)){
                printk(KERN_ERR "imrsim: get zone size failed\n");
                goto ioerr;
            }
            if((__u64)arg == 0){
                printk(KERN_ERR "imrsim: bad parameter\n");
                goto ioerr;
            }
            if(copy_to_user((__u32 *)arg, &param, sizeof(__u32) )){
                printk(KERN_ERR "imrsim: copy zone size to user memory failed\n");
                goto ioerr;
            }
            break;
        case IOCTL_IMRSIM_SET_SIZZONEDEFAULT:
            if((__u64)arg == 0){
                printk(KERN_ERR "imrsim: bad parameter\n");
                goto ioerr;
            }
            if(copy_from_user(&param, (__u32 *)arg, sizeof(__u32) )){
                printk(KERN_ERR "imrsim: set zone size copy from user failed\n");
                goto ioerr;
            }
            if(imrsim_set_size_zone_default(param)){
                printk(KERN_ERR "imrsim: set default zone size failed\n");
                goto ioerr;
            }
            imrsim_ptask.flag |= IMR_CONFIG_CHANGE;
            break;
        case IOCTL_IMRSIM_RESET_ZONE:
            if((__u64)arg == 0){
                printk(KERN_ERR "imrsim: bad parameter\n");
                goto ioerr;
            }
            if(copy_from_user(&num64, (__u64 *)arg, sizeof(__u64) )){
                printk(KERN_ERR "imrsim: reset zone write pointer copy from user memory failed\n");
                goto ioerr;
            }
            if(imrsim_blkdev_reset_zone_ptr(num64)){
                printk(KERN_ERR "imrsim: reset zone write pointer failed\n");
                goto ioerr;
            }
            imrsim_ptask.flag |= IMR_CONFIG_CHANGE;
            break;
        case IOCTL_IMRSIM_QUERY:
            zbc_query = kzalloc(sizeof(imrsim_zbc_query), GFP_KERNEL);
            if(!zbc_query){
                printk(KERN_ERR "imrsim: %s no enough memory for zbc query\n", __FUNCTION__);
                goto ioerr;
            }
            if((__u64)arg == 0){
                printk(KERN_ERR "imrsim: bad parameter\n");
                goto zfail;
            }
            ret = copy_from_user(zbc_query, (imrsim_zbc_query *)arg, sizeof(imrsim_zbc_query));
            if(ret){
                printk(KERN_ERR "imrsim: %s copy from user for zbc query failed\n", __FUNCTION__);
                goto zfail;
            }
            if (zbc_query->num_zones == 0 || zbc_query->num_zones > IMR_NUMZONES) {
                printk(KERN_ERR "imrsim: Wrong parameter for the number of zones\n");
                goto zfail;
            }
            size = sizeof(imrsim_zbc_query) + sizeof(struct imrsim_zone_status) *
                  (zbc_query->num_zones - 1);
            zbc_query = krealloc(zbc_query, size, GFP_KERNEL);
            if (!zbc_query) {
                printk(KERN_ERR "imrsim: %s no enough emeory for zbc query\n", __FUNCTION__);
                goto zfail;
            } 
            if (imrsim_query_zones(zbc_query->lba, zbc_query->criteria, 
                &zbc_query->num_zones, zbc_query->ptr)) {
                printk(KERN_ERR "imrsim: %s query zone status failed\n", __FUNCTION__);
                goto zfail;            
            }
            if(copy_to_user((__u32 *)arg, zbc_query, size)){
                    printk(KERN_ERR "imrsim: %s copy to user for zbc query failed\n", __FUNCTION__);
                    goto zfail;
            }
            kfree(zbc_query);
            break;
        zfail:
            kfree(zbc_query);
            break;
        /* IMRSIM stats IOCTLs */
        case IOCTL_IMRSIM_GET_STATS:
            size = imrsim_stats_size();
            pstats = (struct imrsim_stats *)kzalloc(size, GFP_ATOMIC);
            if(!pstats){
                printk(KERN_ERR "imrsim: no enough memory to hold stats\n");
                goto ioerr;
            }
            if(imrsim_get_stats(pstats)){
                printk(KERN_ERR "imrsim: get stats failed\n");
                kfree(pstats);
                goto sfail;
            }
            if(imrsim_dbg_log_enabled){
                imrsim_report_stats(pstats);
            }
            if((__u64)arg == 0){
                printk(KERN_ERR "imrsim: bad parameter\n");
                goto sfail;
            }
            if(copy_to_user((struct imrsim_stats *)arg, pstats, size)){
                printk(KERN_ERR "imrsim: get stats failed as insufficient user memory\n");
                kfree(pstats);
                goto sfail;
            }
            kfree(pstats);
            break;
        sfail:
            kfree(pstats);
            break;
        case IOCTL_IMRSIM_RESET_STATS:
            if(imrsim_reset_stats()){
                printk(KERN_ERR "imrsim: reset stats failed\n");
                goto ioerr;
            }
            imrsim_ptask.flag |= IMR_CONFIG_CHANGE;
            break;
        case IOCTL_IMRSIM_RESET_ZONESTATS:
            if((__u64)arg == 0){
                printk(KERN_ERR "imrsim: bad parameter\n");
                goto ioerr;
            }
            if(copy_from_user(&num64, (__u64 *)arg, sizeof(__u64) )){
                printk(KERN_ERR "imrsim: copy reset zone lba from user memory failed\n");
                goto ioerr;
            }
            if(imrsim_reset_zone_stats(num64)){
                printk(KERN_ERR "imrsim: reset zone stats on lba failed");
                goto ioerr;
            }
            imrsim_ptask.flag |= IMR_CONFIG_CHANGE;
            break;
        /* IMRSIM config IOCTLs */
        case IOCTL_IMRSIM_RESET_DEFAULTCONFIG:
            if(imrsim_reset_default_config()){
                goto ioerr;
            }
            imrsim_ptask.flag |= IMR_CONFIG_CHANGE;
            break;
        case IOCTL_IMRSIM_RESET_ZONECONFIG:
            if(imrsim_reset_default_zone_config()){
                goto ioerr;
            }
            imrsim_ptask.flag |= IMR_CONFIG_CHANGE;
            break;
        case IOCTL_IMRSIM_RESET_DEVCONFIG:
            if(imrsim_reset_default_device_config()){
                goto ioerr;
            }
            imrsim_ptask.flag |= IMR_CONFIG_CHANGE;
            break;
        case IOCTL_IMRSIM_GET_DEVCONFIG:
            if(imrsim_get_device_config(&pconf)){
                goto ioerr;
            }
            if((__u64)arg == 0){
                printk(KERN_ERR "imrsim: bad parameter\n");
                goto ioerr;
            }
            if(copy_to_user((struct imrsim_dev_config*)arg, &pconf, sizeof(struct imrsim_dev_config) )){
                goto ioerr;
            }
            break;
        case IOCTL_IMRSIM_SET_DEVRCONFIG_DELAY:
            if ((__u64)arg == 0) {
                printk(KERN_ERR "imrsim: bad parameter\n");
                goto ioerr; 
            }
            if(copy_from_user(&pconf, (struct imrsim_dev_config *)arg, sizeof(struct imrsim_dev_config) )){
                goto ioerr;
            }
            if(imrsim_set_device_rconfig_delay(&pconf)){
                goto ioerr;
            }
            imrsim_ptask.flag |= IMR_CONFIG_CHANGE;
            break;
        case IOCTL_IMRSIM_SET_DEVWCONFIG_DELAY:
            if ((__u64)arg == 0) {
                printk(KERN_ERR "imrsim: bad parameter\n");
                goto ioerr; 
            }
            if(copy_from_user(&pconf, (struct imrsim_dev_config *)arg, sizeof(struct imrsim_dev_config) )){
                goto ioerr;
            }
            if(imrsim_set_device_wconfig_delay(&pconf)){
                goto ioerr;
            }
            imrsim_ptask.flag |= IMR_CONFIG_CHANGE;
            break;
        default:
            break;
    }
    if(imrsim_ptask.flag & IMR_CONFIG_CHANGE){
        wake_up_process(imrsim_ptask.pstore_thread);
    }
    mutex_unlock(&imrsim_ioctl_lock);
    return 0;
    ioerr:
    mutex_unlock(&imrsim_ioctl_lock);
    return -EFAULT;
}

/* To merge requests. */
static int imrsim_merge(struct dm_target* ti, 
                        struct bvec_merge_data* bvm,
                        struct bio_vec* biovec, 
                        int max_size)
{
   struct imrsim_c*      c = ti->private;
   struct request_queue* q = bdev_get_queue(c->dev->bdev);

   if (!q->merge_bvec_fn)
      return max_size;

   bvm->bi_bdev   = c->dev->bdev;
   bvm->bi_sector = imrsim_map_sector(ti, bvm->bi_sector);

   return min(max_size, q->merge_bvec_fn(q, bvm, biovec));
}

/* iterate devices */
static int imrsim_iterate_devices(struct dm_target *ti,
                                  iterate_devices_callout_fn fn,
                                  void *data)
{
   struct imrsim_c* c = ti->private;

   return fn(ti, c->dev, c->start, ti->len, data);
}

/* Core structure - represents the target-driven plug-in, 
and the structure collects the function entry for the functions implemented by the driver plug-in */
static struct target_type imrsim_target = 
{
    .name            = "imrsim",
    .version         = {1, 0, 0},
    .module          = THIS_MODULE,
    .ctr             = imrsim_ctr,
    .dtr             = imrsim_dtr,
    .map             = imrsim_map,
    .status          = imrsim_status,
    .ioctl           = imrsim_ioctl,
    .merge           = imrsim_merge,
    .iterate_devices = imrsim_iterate_devices
};

/* init IMRSim module */
static int __init dm_imrsim_init(void)
{
    int ret = 0;

    printk(KERN_INFO "imrsim: %s called.\n", __FUNCTION__);
    ret = dm_register_target(&imrsim_target);
    if(ret < 0){
        printk(KERN_ERR "imrsim: register failed\n");
    }
    return ret;
}

/* kill IMRSim module */
static void dm_imrsim_exit(void)
{
    dm_unregister_target(&imrsim_target);
}

module_init(dm_imrsim_init);    
module_exit(dm_imrsim_exit);    

/* Module related signature information */
MODULE_DESCRIPTION(DM_NAME "IMR Simulator");
MODULE_AUTHOR("Zhimin Zeng <im_zzm@126.com>");
MODULE_LICENSE("GPL");