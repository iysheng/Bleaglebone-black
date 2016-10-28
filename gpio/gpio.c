#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/slab.h>

#define GPIOGROUP_NUM 4
//#define GPIO1_BASE 0x4804c000
#define GPIO_OE_OFFSET 0x134
#define GPIO_DATAOUT_OFFET 0X13c

#define GPIO_MA 1
#define GPIO_MI 28
#define GPIO_NUM 0 

#define GPIOLED_NUM 1

#define NAME_LEN 16
#define GPIOLED_MAJOR 0

#define MKGPIONUM(ma,mi) (ma * 32 + mi)
#define MAGPIO(num) (num / 32)
#define MIGPIO(num) (num % 32)

static unsigned int gpiobase[GPIOGROUP_NUM] = {0x44E07000, 0x4804C000, 0x481AC000, 0x481AE000};

static unsigned int gpio_oe,gpio_dataout;
static char gpio_ma, gpio_mi;
static int major = GPIOLED_MAJOR;
static dev_t devnum;
static struct class *gpioled_class;


	
struct gpioled_dev{
	struct cdev cdev;
	char name[NAME_LEN];
	char gpio_ma;
	char gpio_mi;
	struct device *gpioled_device;
};

static struct gpioled_dev *gpioled_devp;


static int gpioled_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	unsigned int value_tmp;
	filp->private_data = container_of(inode->i_cdev, struct gpioled_dev, cdev);
	struct gpioled_dev *devp = filp->private_data;
	value_tmp = readl(gpio_dataout);
	printk("gpioled open and gpio%d_outdata:0x%lx.\n",MKGPIONUM(devp->gpio_ma, devp->gpio_mi),(unsigned long)value_tmp);
	return ret;	
}

static ssize_t gpioled_read(struct file *filp,char __user *buf,size_t count,loff_t *ppos)
{
	unsigned int value_tmp;
	struct gpioled_dev *devp = filp->private_data;
	value_tmp = readl(gpio_dataout);
	if(copy_to_user(buf,&value_tmp,sizeof(value_tmp)))
		printk(KERN_INFO "read failed!\n");
	else
	{
		*ppos+=sizeof(value_tmp);
		printk(KERN_INFO "%s open and outdata:0x%lx.\n",devp->name, (unsigned long)value_tmp);
	}
	return 0;
}

static ssize_t gpioled_write (struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
	char value;
	struct gpioled_dev *devp = filp->private_data;
	copy_from_user(&value, buf, sizeof(value));
	if(value == '1')
		writel((1<<devp->gpio_mi),gpio_dataout);
	else if(value == '0')
		writel((0<<devp->gpio_mi),gpio_dataout);
	else
		printk("no valid value value:%d.\n",value);
	
	return sizeof(value);
}

static int gpioled_release(struct inode *inode, struct file *filp)
{
	int ret = 0;
	
	return ret;
}

static struct file_operations gpioled_fops = {
	.owner = THIS_MODULE,
	.open = gpioled_open,
	.read = gpioled_read,
	.write = gpioled_write,
	.release =  gpioled_release,
};

static void gpioled_setup(struct gpioled_dev * devp, int index)
{
	int ret = 0;
	devnum = MKDEV(major,index);
	cdev_init(&devp->cdev,&gpioled_fops);
	devp->cdev.owner = THIS_MODULE;
	ret = cdev_add(&devp->cdev,devnum,1);
	if(ret)
	{
		printk(KERN_INFO "Error %d adding gpioled%d.\n",ret, index);
	}
	else
	{		
		devp->gpio_ma = gpio_ma;
		devp->gpio_mi = gpio_mi;
		sprintf(devp->name,"gpio%d_%d",devp->gpio_ma, devp->gpio_mi);
		devp->gpioled_device = device_create(gpioled_class, NULL, devnum,NULL,devp->name);
		printk(KERN_INFO "%s init.\n",devp->name);
	}	
}
static int __init gpioled_init(void)
{
	int i, ret = 0;
	unsigned int value_tmp;
	if(GPIO_NUM != 0)
	{
		gpio_ma = MAGPIO(GPIO_NUM);
		gpio_mi = MIGPIO(GPIO_NUM);
	}
	else
	{
		gpio_ma = GPIO_MA;
		gpio_mi = GPIO_MI;
	}
	if(request_mem_region(gpiobase[gpio_ma] | GPIO_OE_OFFSET,sizeof(int),"GPIO_OE"))
	{
		ret = -EBUSY;
		goto fail;
	}
	if(request_mem_region(gpiobase[gpio_ma] | GPIO_DATAOUT_OFFET,sizeof(int),"GPIO_DATAOUT"))
	{
		ret = -EBUSY;
		goto fail;
	}
	gpio_oe = ioremap(gpiobase[gpio_ma] | GPIO_OE_OFFSET,sizeof(int));
	if(gpio_oe == NULL)
	{
		release_mem_region(gpiobase[gpio_ma] | GPIO_OE_OFFSET,sizeof(int));
		ret = -ENOMEM;
		goto fail;
	}
	gpio_dataout = ioremap(gpiobase[gpio_ma] | GPIO_DATAOUT_OFFET,sizeof(int));
	if(gpio_dataout == NULL)
	{
		release_mem_region(gpiobase[gpio_ma] | GPIO_DATAOUT_OFFET,sizeof(int));
		ret = -ENOMEM;
		goto fail;
	}
	if(major)
	{
		devnum = MKDEV(major,0);
		ret = register_chrdev_region(devnum,GPIOLED_NUM,"gpioled");
	}
	else
	{
		ret = alloc_chrdev_region(&devnum,0,GPIOLED_NUM,"gpioled");
		major = MAJOR(devnum);
	}
	if(ret < 0)
	{
		goto fail;
	}
	gpioled_devp = kzalloc(sizeof(struct gpioled_dev) * GPIOLED_NUM,GFP_KERNEL);
	if(IS_ERR(gpioled_devp))
	{
		ret = -ENOMEM;
		goto fail;
	}
	gpioled_class = class_create(THIS_MODULE,"gpioled");
	for(i=0; i<GPIOLED_NUM; i++)
	{
		gpioled_setup(gpioled_devp + i, i);
	}
	writel(~(1<<gpio_mi),gpio_oe);
fail:
	return ret;
}

static void __exit gpioled_exit(void)
{
	int i;
	for(i=0; i<GPIOLED_NUM; i++)
	{
		devnum = MKDEV(major,i);
		device_destroy(gpioled_class,devnum);
		printk("%s exit.\n",(gpioled_devp+ i)->name);
		cdev_del(&(gpioled_devp+i)->cdev);
		unregister_chrdev_region(devnum,1);		
	}
	class_destroy(gpioled_class);
	kfree(gpioled_devp);
}

module_init(gpioled_init);
module_exit(gpioled_exit);

MODULE_AUTHOR("yang yongsheng<iysheng@163.com>");
MODULE_LICENSE("GPL v2");
