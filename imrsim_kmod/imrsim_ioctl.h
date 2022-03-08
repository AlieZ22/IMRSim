#ifndef _IMRSIM_IOCTL_H
#define _IMRSIM_IOCTL_H

/*
* IMRSim zone status IOCTLs
*/
#define IOCTL_IMRSIM_GET_NUMZONES           _IOR('z', 1, __u32 *)
#define IOCTL_IMRSIM_GET_SIZZONEDEFAULT     _IOR('z', 2, __u32 *)
#define IOCTL_IMRSIM_SET_SIZZONEDEFAULT     _IOW('z', 3, __u32 *)
#define IOCTL_IMRSIM_RESET_ZONE         _IOW('z', 4, __u64 *)
#define IOCTL_IMRSIM_QUERY              _IOWR('z', 5, imrsim_zbc_query *)

/*
* IMRSIM zone statistics IOCTLs
*/
#define IOCTL_IMRSIM_GET_STATS               _IOR('s', 1, struct imrsim_stats *)
#define IOCTL_IMRSIM_RESET_STATS             _IO('s', 2)
#define IOCTL_IMRSIM_RESET_ZONESTATS         _IOW('s', 3, __u64 *)

/*
* IMRSIM zone config IOCTLs
*/
#define IOCTL_IMRSIM_RESET_DEFAULTCONFIG     _IO('l', 1)
#define IOCTL_IMRSIM_RESET_ZONECONFIG        _IO('l', 2)
#define IOCTL_IMRSIM_RESET_DEVCONFIG         _IO('l', 3)
#define IOCTL_IMRSIM_GET_DEVCONFIG           _IOR('l', 4, struct imrsim_dev_config *)
#define IOCTL_IMRSIM_SET_DEVRCONFIG_DELAY    _IOW('l', 5, struct imrsim_dev_config *)
#define IOCTL_IMRSIM_SET_DEVWCONFIG_DELAY    _IOW('l', 6, struct imrsim_dev_config *)

/*
* IMRSIM debug error IOCTLs
*/ 
#define IOCTL_IMRSIM_GET_LAST_RERROR         _IOR('e', 1, __u32 *)
#define IOCTL_IMRSIM_GET_LAST_WERROR         _IOR('e', 2, __u32 *)
#define IOCTL_IMRSIM_SET_LOGENABLE           _IO('e', 3)
#define IOCTL_IMRSIM_SET_LOGDISABLE          _IO('e', 4)




#endif