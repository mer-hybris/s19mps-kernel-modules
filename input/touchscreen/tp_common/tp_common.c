/* ********************************************************************
  Revo Setting  GPIO_TOUCH_IRQ  GPIO_TOUCH_RESET
  (include tp_common.h)
********************************************************************** */
#include <linux/kernel.h>
#include <linux/dmi.h>
#include <linux/firmware.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/of.h>
//#include  "prj/prj_config.h"

static unsigned short lcd_width;
static unsigned short lcd_height;

static int ts_parse_lcd_size(char *str)
{
	char *c, buf[32] = { 0 };
	int length;
	if (str != NULL) {
		c = strchr(str, 'x');
		if (c != NULL) {
			/* height */
			length = c - str;
			strncpy(buf, str, length);
			if (kstrtou16(buf, 10, &lcd_height))
				lcd_height = 0;
			/* width */
			length = strlen(str) - (c - str) - 1;
			strncpy(buf, c + 1, length);
			buf[length] = '\0';
			if (kstrtou16(buf, 10, &lcd_width))
				lcd_width = 0;
		} else {
			lcd_width = lcd_height = 0;
		}
	}
	pr_info("lcd_height = %d, lcd_width = %d\n",lcd_height,lcd_width);
	return 1;
}

static int get_lcd_size(char *lcd_size)
{
    struct device_node *np;
    const char *cmd_line;
    char *s = NULL;

    int ret = 0;
    np = of_find_node_by_path("/chosen");

    if (!np) {
        pr_err("Can't get the /chosen\n");
        return -EIO;
    }

    ret = of_property_read_string(np, "bootargs", &cmd_line);
    if (ret < 0) {
        pr_err("Can't get the bootargs\n");
        return ret;
    }

    s = strstr(cmd_line, "lcd_size=");
    s += sizeof("lcd_size");
    while(*s != ' '){
        *lcd_size++ = *s++;
    }
    *lcd_size = '\0';

    return 0;
}

/* ****************************************************
     Revo:  Add  get TP size 
*******************************************************/
int revo_get_tp_size(unsigned short *tp_width,unsigned short *tp_height)
{   
    char lcd_size_str[10] = {0};
    get_lcd_size(lcd_size_str);
    ts_parse_lcd_size(lcd_size_str);

    if( lcd_width && lcd_height ) {
        *tp_width  = lcd_width ;
        *tp_height = lcd_height ;
        return  0 ;
    }
    return -1 ;
}
