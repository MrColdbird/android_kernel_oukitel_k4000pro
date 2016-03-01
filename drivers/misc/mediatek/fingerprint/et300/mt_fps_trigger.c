//
// FILENAME.
//      fps_trigger.c - FringerPrint Sensor signal trigger routine
//
//      $PATH:
//
// FUNCTIONAL DESCRIPTION.
//      FingerPrint Sensor trigger routine
//
// MODIFICATION HISTORY.
//
// NOTICE.
//      Copyright (C) 2000-2014 EgisTec All Rights Reserved.
//

#include "mt_fps_trigger.h"
#include <linux/delay.h>
//
// interrupt description
//
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_spi.h>

#include <cust_eint.h>
#include <mach/irqs.h>
#include <mach/eint.h>

#include "cust_gpio_usage.h"

//
// FPS interrupt table 
//

static volatile char interrupt_values[] = {
	'0', '0', '0', '0', '0', '0', '0', '0'
};

static DECLARE_WAIT_QUEUE_HEAD(interrupt_waitq);

static volatile int ev_press = 0;
extern void et300_device_change();
struct delayed_work fingerprint_delay_work;


void fingerprint_eint_work_func(struct work_struct *work)
{
    et300_device_change();
//    mt_eint_set_sens(CUST_EINT_FIGERPRINT_INT_NUM, EINTF_TRIGGER_RISING);
    mt_eint_unmask(CUST_EINT_FIGERPRINT_INT_NUM);

}
//
// FUNCTION NAME.
//      fingerprint_interrupt
//
// FUNCTIONAL DESCRIPTION.
//      finger print interrupt callback routine 
//
//
// ENTRY PARAMETERS.
//      irq           
//      dev_id 
//
// EXIT PARAMETERS.
//      Function Return          – int, 
//

static void fingerprint_interrupt(void)
{
    
     printk("jay_FPS interrupt trigger");
    mt_eint_mask(CUST_EINT_FIGERPRINT_INT_NUM);
//    wake_up_interruptible(&interrupt_waitq); 
 
    schedule_delayed_work(&fingerprint_delay_work, msecs_to_jiffies(10));
}

//
// FUNCTION NAME.
//      Interrupt_Init
//
// FUNCTIONAL DESCRIPTION.
//      button initial routine
//
//
// ENTRY PARAMETERS.
//      gpio             - gpio address
//
// EXIT PARAMETERS.
//      Function Return          – int, 
//

int Interrupt_Init(void)
{
      int i;


      INIT_DELAYED_WORK(&fingerprint_delay_work, fingerprint_eint_work_func);

  // Setup Interrupt Pin
	mt_set_gpio_mode(GPIO_FIGERPRINT_INT, GPIO_FIGERPRINT_INT_M_EINT);
//      mt_set_gpio_mode(GPIO_FIGERPRINT_INT, 0);

 
	mt_set_gpio_dir(GPIO_FIGERPRINT_INT, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_FIGERPRINT_INT, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_FIGERPRINT_INT, GPIO_PULL_UP);

	//mt_eint_set_sens(CUST_EINT_TOUCH_FINGERPRINT_NUM, CUST_EINT_FIGERPRINT_INT_TYPE);
	mt_eint_set_hw_debounce(CUST_EINT_FIGERPRINT_INT_NUM, CUST_EINT_FIGERPRINT_INT_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_FIGERPRINT_INT_NUM, EINTF_TRIGGER_RISING, fingerprint_interrupt, 0);
	//mt_eint_registration(CUST_EINT_TOUCH_FINGERPRINT_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, tpd_eint_interrupt_handler, 1);
	mt_eint_unmask(CUST_EINT_FIGERPRINT_INT_NUM);
//	msleep(100);


  return 0;
}

//
// FUNCTION NAME.
//      Interrupt_Free
//
// FUNCTIONAL DESCRIPTION.
//      free all interrupt resource
//
//
// ENTRY PARAMETERS.
//      gpio             - gpio address
//
// EXIT PARAMETERS.
//      Function Return          – int, 
//

int Interrupt_Free(void)
{
  return 0;
}

//
// FUNCTION NAME.
//      fps_interrupt_read
//
// FUNCTIONAL DESCRIPTION.
//      FPS interrupt read status
//
//
// ENTRY PARAMETERS.
//      
//
// EXIT PARAMETERS.
//      Function Return          – int, 
//

int fps_interrupt_read(
  struct file *filp,
  char __user *buff,
  size_t count,
  loff_t *offp
)
{
  unsigned long err;

  if (filp->f_flags & O_NONBLOCK)
  {
    return -EAGAIN;
  }
  else
  {
    wait_event_interruptible(interrupt_waitq, ev_press);
  }

  printk(KERN_ERR "interrupt read condition %d\n",ev_press); 
  printk(KERN_ERR "interrupt value  %d\n",interrupt_values[0]); 

  err = copy_to_user((void *)buff, (const void *)(&interrupt_values),min(sizeof(interrupt_values), count));
  return err ? -EFAULT : min(sizeof(interrupt_values), count);
}

int Interrupt_Exit(void)
{
//  ev_press=1;
  return 0;
}

//
// FUNCTION NAME.
//      fps_interrupt_read
//
// FUNCTIONAL DESCRIPTION.
//      FPS interrupt read status
//
//
// ENTRY PARAMETERS.
//               
//      wait      poll table structure  
//
// EXIT PARAMETERS.
//      Function Return          – int, 
//

unsigned int fps_interrupt_poll( 
  struct file *file,
  struct poll_table_struct *wait
)
{
  unsigned int mask = 0;

  poll_wait(file, &interrupt_waitq, wait);
  if (ev_press)
  {
    mask |= POLLIN | POLLRDNORM;
  }

  return mask;
}


