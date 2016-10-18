/**
 * @file   globalfifo.c
 * @author yang yongsheng
 * @date   2016.10.17
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>                 // Required for the GPIO functions
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/printk.h>


MODULE_LICENSE("GPL");
MODULE_AUTHOR("yang yongsheng<iysheng@163.com>");
MODULE_DESCRIPTION("A fifo test driver for the BBB");
MODULE_VERSION("0.1");

#define GLOBALFIFO_SIZE 64
#define NAME_SIZE 16
#define GLOBALFIFO_NUM 3
#define GLOBALFIFO_MAGIC 'f'
#define MEM_CLEAR _IO(GLOBALFIFO_MAGIC,0)

static int major = 0;
static dev_t devnum;

struct globalfifo_dev{
    struct cdev cdev;
	int cur_len;
	int r_times;
	char mem[GLOBALFIFO_SIZE];
	struct mutex mutex;
	struct device *globalfifo_device;
	wait_queue_head_t r_wait;
	wait_queue_head_t w_wait;
};

struct globalfifo_dev *globalfifo_devp;
static struct class *globalfifo_class;

//static struct device *globalfifo_device[GLOBALFIFO_NUM];

static int globalfifo_open(struct inode *inode, struct file *filp)
{
	struct globalfifo_dev *globalfifo_devp_tmp = container_of(inode->i_cdev, struct globalfifo_dev, cdev);
	filp->private_data = globalfifo_devp_tmp;
	return 0;
}
static ssize_t globalfifo_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
    int ret = 0;
	
	unsigned long p = *ppos;
	int size = count;    
	struct globalfifo_dev *devp_tmp = filp->private_data;
	devp_tmp->r_times ++;
	
	printk(KERN_INFO "BEFORE ppos is %lu,size is %d\nr_times is %d", p, size, devp_tmp->r_times);

	if(p >= GLOBALFIFO_SIZE)
	{
	    printk("buf longer than size\n");
		return 0;
	}
	else
	{
	    size = (size > GLOBALFIFO_SIZE - p) ? GLOBALFIFO_SIZE - p : count;
	    if(copy_to_user(buf, devp_tmp->mem, size))
		    ret = -EFAULT;
	    else
	    {
		    ret = size;
		    //printk(KERN_INFO "read %u bytes from %lu", size, p);
	    }
	        printk(KERN_INFO "AFTER ppos is %lu,size is %d\nr_times is %d", p, size, devp_tmp->r_times);
	}
	//copy_to_user(buf, devp_tmp->mem, count);
	return ret;
}
static ssize_t globalfifo_write (struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{   
    int ret = 0;
	unsigned long p = *ppos;
	unsigned int size = count;
	struct globalfifo_dev * devp_tmp = filp->private_data;
	if(p > GLOBALFIFO_SIZE)
	{
	    printk("buf longer than size.\n");
		return 0;
    }
	if(size > GLOBALFIFO_SIZE - p)
		size = GLOBALFIFO_SIZE - p;
	if(copy_from_user(devp_tmp->mem + p, buf, size))
		ret = -EFAULT;
	else
	{
	    *ppos += size;
		ret = size;
		printk(KERN_INFO "written %u bytes from %lu", size, p);
	}
	return ret;
}

static loff_t globalfifo_llseek (struct file *filp,loff_t offset,int orig)
{
    loff_t ret = 0;
	switch(orig)
	{
	case 0:
		if(offset < 0)
		{
		    ret = -EINVAL;
			break;
		}
		if((unsigned int)offset > GLOBALFIFO_SIZE)
		{
		    ret = -EINVAL;
			break;
		}
		filp->f_pos = (unsigned int)offset;
		ret = filp->f_pos;
		break;
	case 1:
		if((filp->f_pos + offset) > GLOBALFIFO_SIZE)
		{
		    ret = -EINVAL;
			break;
		}
		if((filp->f_pos + offset) > GLOBALFIFO_SIZE)
		{
		    ret = -EINVAL;
			break;
		}
		filp->f_pos += offset;
		ret = filp->f_pos;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static long globalfifo_ioctl(struct file *filp,unsigned int cmd,unsigned long arg)
{
    struct globalfifo_dev *devp_tmp = filp->private_data;
	switch(cmd)
	{
	    case MEM_CLEAR:
	    //case 0:
			memset(devp_tmp->mem, 0, GLOBALFIFO_SIZE);
			printk(KERN_INFO "globalfifo has been cleared.\n");
			break;
		default:
			printk(KERN_INFO "invalid cmd.\n");
			return -EINVAL;		
	}
	return 0;
}

static int globalfifo_release (struct inode *inode, struct file *filp)
{
    int ret = 0;
    return ret;
}

static struct file_operations globalfifo_fops = {
    .owner = THIS_MODULE,
	.open = globalfifo_open,
	.read = globalfifo_read,
	.write = globalfifo_write,
	.unlocked_ioctl = globalfifo_ioctl,
	.llseek = globalfifo_llseek,
	.release = globalfifo_release,
};

static void globalfifo_setup(struct globalfifo_dev *devp,int index)
{
    char name_buf[NAME_SIZE];
    int ret;
	devnum = MKDEV(major,index);	
	sprintf(name_buf,"globalfifo%d",index);
	cdev_init(&devp->cdev,&globalfifo_fops);
	devp->cdev.owner = THIS_MODULE;
	ret = cdev_add(&devp->cdev,devnum,1);
	if(ret)
	{
		printk(KERN_INFO "Error %d adding globalfifo%d", ret, index);
	}
	else
	//globalfifo_device[index] = 	
	devp->globalfifo_device = device_create(globalfifo_class,NULL,devnum,NULL,name_buf);	
}

static int __init globalfifo_init(void)
{
    int i,ret = 0;	
	
	if(major)
	{
	    devnum = MKDEV(major,0);
		ret = register_chrdev_region(devnum,GLOBALFIFO_NUM,"globalfifo");
	}
	else
	{
	    ret = alloc_chrdev_region(&devnum,0,GLOBALFIFO_NUM,"globalfifo");
		major = MAJOR(devnum);
    }
	if(ret < 0)
		goto fail;
	globalfifo_devp = kzalloc(sizeof(struct globalfifo_dev) * GLOBALFIFO_NUM,GFP_KERNEL);
	if(!globalfifo_devp)
	{
	    ret = ENOMEM;
	    goto fail;
    }
	globalfifo_class = class_create(THIS_MODULE,"globalfifo");
	for(i=0; i<GLOBALFIFO_NUM; i++)
	{
	    globalfifo_setup(globalfifo_devp + i, i);
	}
    printk(KERN_INFO "fifo test begin and the major is %d!\n",major);
    fail:
	return ret;
}

static void __exit globalfifo_exit(void)
{
    int i;
    for(i=0; i<GLOBALFIFO_NUM; i++)
    {
        device_destroy(globalfifo_class,devnum);
	    cdev_del(&(globalfifo_devp + i)->cdev);
		devnum = MKDEV(major,i);
	    unregister_chrdev_region(devnum,1);		
    }
	class_destroy(globalfifo_class);
    kfree(globalfifo_devp);
    printk(KERN_INFO "fifo test over!\n");
}


module_init(globalfifo_init);
module_exit(globalfifo_exit);

