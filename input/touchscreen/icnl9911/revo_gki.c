#include "revo_gki.h"

struct file *revo_filp_open(const char *filename, int flags, umode_t mode)
{
	return NULL;
}

ssize_t revo_kernel_read(struct file *file, void *buf, size_t count, loff_t *pos)
{
	return 0;
}

ssize_t revo_kernel_write(struct file *file, void *buf, size_t count, loff_t *pos)
{
	return 0;
}
