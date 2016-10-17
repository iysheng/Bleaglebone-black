/**
 * @file   led.c
 * @author yang yongsheng
 * @date   2016.10.3
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


MODULE_LICENSE("GPL");
MODULE_AUTHOR("yang yongsheng<iysheng@163.com>");
MODULE_DESCRIPTION("A LED test driver for the BBB");
MODULE_VERSION("0.1");

#define STATE_SIZE 16
#define BUF_SIZE 32

static unsigned int gpioLED = 60;       ///< hard coding the LED gpio for this example to P9_12 (GPIO60)
static unsigned int gpioKEY = 65;       ///< hard coding the LED gpio for this example to P8_18 (GPIO65)

//static unsigned int numberPresses = 0;  ///< For information, store the number of button presses
static bool	    ledOn = 0;          ///< Is the LED on or off? Used to invert its state (off by default)

static unsigned  int major;
static struct class *led_class;
static struct device *led_drv;
static unsigned int keyirq;

static irq_handler_t KEY_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);

static struct led_dev{
	struct cdev cdev;
	struct mutex mutex;
	atomic_t numberLed;
	char state[STATE_SIZE];
};

static struct led_dev *led_devp;



static int led_open(struct inode *inode, struct file *file)
{
    file->private_data = led_devp;
	//atomic_set(&(file->private_data->numberLed),0);
	atomic_set(&(led_devp->numberLed),0);
	strcpy(led_devp->state,"opened");
	printk("led %s and numberLed is %d.\n",led_devp->state,atomic_read(&(led_devp->numberLed)));
    return 0;
}

static int led_release (struct inode *inode, struct file *file)
{
    file->private_data = led_devp;
	strcpy(led_devp->state,"released");
	printk("led %s.\n",led_devp->state);
    return 0;
}

static ssize_t led_write (struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int val;
	struct led_dev *dev = file->private_data;
	copy_from_user(&val, buf, count);

	if (val == 1)
		{
			strcpy(dev->state,"on");
		    gpio_set_value(gpioLED, true);
			atomic_add(1,&(dev->numberLed));
		}
    else if(val == 0)
    	{
    	    strcpy(dev->state,"off");
    	    gpio_set_value(gpioLED, false);
			atomic_sub(1,&(dev->numberLed));
    	}
	else
		{
		    strcpy(dev->state,"invalid");
		}
	printk("writing led %s and numberLed is %d.\n",dev->state,atomic_read(&(dev->numberLed)));
    return 0;
}

static ssize_t led_read (struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int val;
	char *kbuf[BUF_SIZE];
	struct led_dev *dev = file->private_data;
    val = gpio_get_value(gpioKEY);
	sprintf(kbuf,"state:%s numberLed:%d",dev->state,atomic_read(&(dev->numberLed)));
	copy_to_user(buf,kbuf,count);
	printk("reading led %s.\n",kbuf);
    return 0;
}

static struct file_operations led_fops = {
	.owner = THIS_MODULE,
	.open = led_open,
	.release = led_release,
	.write = led_write,	
	.read = led_read,
};

static int __init led_init(void){
   int result = 0;
   
   printk(KERN_INFO "GPIO_TEST: Initializing the GPIO_TEST LKM\n");
   // Is the GPIO a valid GPIO number (e.g., the BBB has 4x32 but not all available)
   if (!gpio_is_valid(gpioLED)){
      printk(KERN_INFO "invalid LED GPIO%d\n",gpioLED);
      return -ENODEV;
   }
   if (!gpio_is_valid(gpioKEY)){
      printk(KERN_INFO "invalid KEY GPIO%d\n",gpioKEY);
      return -ENODEV;
   }
   // Going to set up the LED. It is a GPIO in output mode and will be on by default
   ledOn = true;
   gpio_request(gpioLED, "sysfs");          // gpioLED is hardcoded to 60, request it
   gpio_direction_output(gpioLED, ledOn);   // Set the gpio to be in output mode and on
   gpio_set_value(gpioLED, ledOn);          // Not required as set by line above (here for reference)
   gpio_export(gpioLED, false);             // Causes gpio60 to appear in /sys/class/gpio

   gpio_request(gpioKEY, "sysfs");          // gpioLED is hardcoded to 60, request it
   gpio_direction_input(gpioKEY);   // Set the gpio to be in input mode and on
   gpio_set_debounce(gpioKEY,10); //去抖10ms
   gpio_export(gpioKEY, false);             // Causes gpio49 to appear in /sys/class/gpio
   printk("gpioKEY finished.\n");

   keyirq = gpio_to_irq(gpioKEY);
   result = request_irq(keyirq,(irq_handler_t)KEY_irq_handler,IRQF_TRIGGER_RISING,"gpio_KEY_handler",NULL);

   led_devp = kzalloc(sizeof(struct led_dev),GFP_KERNEL);

   major = register_chrdev(0,"bbb_led",&led_fops); //注册字符驱动

   led_class = class_create(THIS_MODULE,"bbb_led");
   led_drv = device_create(led_class,NULL,MKDEV(major,0),NULL,"bbb_led");

   
	if(!led_devp)
	{
	    gpio_unexport(gpioLED);                  // Unexport the LED GPIO
        gpio_free(gpioLED);                      // Free the LED GPIO
        free_irq(keyirq,NULL);
        unregister_chrdev(major,"bbb_led");	//卸载字符驱动
        printk(KERN_INFO "GPIO_TEST: Goodbye from the LKM!\n");
	    return -ENOMEM;
	}	

   return result;
}

static irq_handler_t KEY_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
   ledOn = !ledOn;                          // Invert the LED state on each button press
   gpio_set_value(gpioLED, ledOn);          // Set the physical LED accordingly
   printk(KERN_INFO "GPIO_TEST: Interrupt! (button state is %d)\n", gpio_get_value(gpioKEY));
   return (irq_handler_t) IRQ_HANDLED;      // Announce that the IRQ has been handled correctly
}

static void __exit led_exit(void){
   gpio_set_value(gpioLED, 0);              // Turn the LED off, makes it clear the device was unloaded
   gpio_unexport(gpioLED);                  // Unexport the LED GPIO
   gpio_free(gpioLED);                      // Free the LED GPIO
   free_irq(keyirq,NULL);
   kfree(led_devp);
   unregister_chrdev(major,"bbb_led");	//卸载字符驱动
   printk(KERN_INFO "GPIO_TEST: Goodbye from the LKM!\n");
}

module_init(led_init);
module_exit(led_exit);
