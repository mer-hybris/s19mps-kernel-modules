#ifndef _REVO_ILITEK_H_
#define _REVO_ILITEK_H_
#include <linux/types.h>


struct file *revo_filp_open(const char *, int, umode_t);
ssize_t revo_kernel_read(struct file *, void *, size_t, loff_t *);
ssize_t revo_kernel_write(struct file *, void *, size_t, loff_t *);

#endif
