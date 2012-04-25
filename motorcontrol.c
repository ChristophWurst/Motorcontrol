#include <linux/module.h>	
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/ioport.h>		//get I/O port access
#include <linux/uaccess.h>
#include <linux/delay.h>		//timing
#include <linux/sched.h>		//timing, jiffies
#include <asm/msr.h>
#include <linux/kthread.h>		//kernel thread
#include <asm/io.h>
#include <linux/interrupt.h>	//interrupt handling
#include <asm/irq.h>			//irq request

//module license
MODULE_LICENSE("GPL");

/************************************/
/* Motor Control                    */
/* - Creates PWM-Signal             */
/* - Reads RPM via Interrupts       */
/* Driver by Christoph Wurst        */
/* HTL Hollabrunn                   */
/************************************/

//IMPORTANT DEFINES! OTHERWISE YOU GET COMPILE ERRORS!!
//found at http://www.ip-phone-forum.de/showthread.php?t=172914
#define SA_INTERRUPT IRQF_DISABLED
#define SA_SAMPLE_RANDOM IRQF_SAMPLE_RANDOM
#define SA_SHIRQ IRQF_SHARED
#define SA_PROBEIRQ IRQF_PROBE_SHARED
#define SA_PERCPU IRQF_PERCPU

#define SA_TRIGGER_LOW IRQF_TRIGGER_LOW
#define SA_TRIGGER_HIGH IRQF_TRIGGER_HIGH
#define SA_TRIGGER_FALLING IRQF_TRIGGER_FALLING
#define SA_TRIGGER_RISING IRQF_TRIGGER_RISING
#define SA_TRIGGER_MASK IRQF_TRIGGER_MASK

//Prototypes
int motor_open(struct inode*, struct file*); 
int motor_release(struct inode*, struct file*); 
ssize_t motor_read(struct file *filp, char *buf, size_t count, loff_t *anything); 
ssize_t motor_write(struct file *filp,const char *buf,size_t count,loff_t *offset); 

//device major number and device name
const int device_major = 48;
#define DEVICE_NAME "motorcontrol"

//default I/O mapped
static int use_mem = 0;
//default use 8 ports
#define MOTOR_NR_PORTS	8

//probe irq
static int probe = 0;

//irq number
volatile int motor_irq = -1;

//default port base
unsigned long motor_base = 0x378;

//constant bit masks for pwm signal
const unsigned int PWM_HIGH = 0b01111111;
const unsigned int PWM_LOW = 0b00111111;

//default pwm duty cycle
unsigned int duty_cycle = 0;

//File Operarations
struct file_operations MotorFops = { open: motor_open, release: motor_release, read: motor_read, write: motor_write, }; 

//Databuffer
struct DataBuffer {
	char speed;
};

//initialize Databuffer
struct DataBuffer read_buffer = { '0' };

//thread started by default
int thread_started = 1;
//tast structure for thread
struct task_struct *ts;
//default pwm state
int pwm_state=0;

/*################THREAD#############################*/
int pwm_thread(void *data) {
	printk(KERN_INFO "motorcontrol: PWM thread started\n");
	while(!kthread_should_stop()) {
		if(pwm_state) {		//HIGH->LOW
			outb(PWM_LOW,motor_base);
			if(duty_cycle!=9) msleep((9-duty_cycle)*10);
			pwm_state=0;
		}
		else {				//LOW->HIGH
			outb(PWM_HIGH,motor_base);
			if(duty_cycle!=0) msleep(duty_cycle*10);
			pwm_state=1;
		}
	}
	printk(KERN_INFO "motorcontrol: PWM thread stopped\n");
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////

/*################INTERRUPT#############################*/

unsigned int interrupt_count = 0;
unsigned long time_old = 0;
unsigned long time_diff;

irqreturn_t motor_interrupt(int irq, void *dev_id, struct pt_regs *regs) {
	unsigned long time_new;
	
	//get time difference
	unsigned long time;
	rdtscl(time);
	time_diff = time - time_old;
	time_old = time;

	//increment interrupt counter
	interrupt_count++;
	
	return IRQ_HANDLED;
}

void motor_kernelprobe(void)
{
	int count = 0;
	do {
		unsigned long mask;

		mask = probe_irq_on();
		outb_p(0x10,motor_base+2); /* enable reporting */
		outb_p(0x00,motor_base);   /* clear the bit */
		outb_p(0xFF,motor_base);   /* set the bit: interrupt! */
		outb_p(0x00,motor_base+2); /* disable reporting */
		udelay(5);  /* give it some time */
		motor_irq = probe_irq_off(mask);

		if (motor_irq == 0) { /* none of them? */
			printk(KERN_INFO "motorcontrol: no irq reported by probe\n");
			motor_irq = -1;
		}
		/*
		 * if more than one line has been activated, the result is
		 * negative. We should service the interrupt (no need for lpt port)
		 * and loop over again. Loop at most five times, then give up
		 */
	} while (motor_irq < 0 && count++ < 5);
	if (motor_irq < 0)
		printk(KERN_ALERT "motorcontrol: probe failed %i times, giving up\n", count);
}

irqreturn_t motor_probing(int irq, void *dev_id, struct pt_regs *regs)
{
	if (motor_irq == 0) motor_irq = irq;	/* found */
	if (motor_irq != irq) motor_irq = -irq; /* ambiguous */
	return IRQ_HANDLED;
}

void motor_selfprobe(void)
{
	int trials[] = {3, 5, 7, 9, 0};
	int tried[]  = {0, 0, 0, 0, 0};
	int i, count = 0;

	/*
	 * install the probing handler for all possible lines. Remember
	 * the result (0 for success, or -EBUSY) in order to only free
	 * what has been acquired
      */
	for (i = 0; trials[i]; i++)
		tried[i] = request_irq(trials[i], motor_probing,
				SA_INTERRUPT, "motor probe", NULL);

	do {
		motor_irq = 0; /* none got, yet */
		outb_p(0x10,motor_base+2); /* enable */
		outb_p(0x00,motor_base);
		outb_p(0xFF,motor_base); /* toggle the bit */
		outb_p(0x00,motor_base+2); /* disable */
		udelay(5);  /* give it some time */

		/* the value has been set by the handler */
		if (motor_irq == 0) { /* none of them? */
			printk(KERN_INFO "motorcontrol: no irq reported by probe\n");
		}
		/*
		 * If more than one line has been activated, the result is
		 * negative. We should service the interrupt (but the lpt port
		 * doesn't need it) and loop over again. Do it at most 5 times
		 */
	} while (motor_irq <=0 && count++ < 5);

	/* end of loop, uninstall the handler */
	for (i = 0; trials[i]; i++)
		if (tried[i] == 0)
			free_irq(trials[i], NULL);

	if (motor_irq < 0)
		printk(KERN_INFO "motorcontrol: probe failed %i times, giving up\n", count);
}
//////////////////////////////////////////////////////////////////////////////////////////////

/*################INIT###############################*/
int init_motor(void) {
	int result;
	
	//get needed resources
	//check /proc/ioports
	if (!use_mem) {
		if (! request_region(motor_base, MOTOR_NR_PORTS, "motorcontrol")) {
			printk(KERN_ERR "motorcontrol: can't get I/O port address 0x%lx\n",
					motor_base);
			return -ENODEV;
		}
	} else {
		if (! request_mem_region(motor_base, MOTOR_NR_PORTS, "motorcontrol")) {
			printk(KERN_ERR "motorcontrol: can't get I/O mem address 0x%lx\n",
					motor_base);
			return -ENODEV;
		}

		/* also, ioremap it */
		motor_base = (unsigned long) ioremap(motor_base, MOTOR_NR_PORTS);
		/* Hmm... we should check the return value */
	}
	
	//register device
	if (register_chrdev(device_major, DEVICE_NAME, &MotorFops) < 0) 
		printk(KERN_ALERT "motorcontrol: cannot register motor\n"); 
	printk(KERN_INFO "motorcontrol: motorcontrol loaded\n");
	
	//initialize port
	outb(0x00, motor_base + 2);	//enable write
	printk(KERN_INFO "motorcontrol: port status register 0x%x", inb(motor_base + 1));
	printk(KERN_INFO "motorcontrol: port control register 0x%x", inb(motor_base + 2));
	
	//enable irq
	//check /proc/interrupts
	if (motor_irq < 0 && probe == 1)
		motor_kernelprobe();

	if (motor_irq < 0 && probe == 2)
		motor_selfprobe();

	if (motor_irq < 0) //set default irq
		switch(motor_base) {
		    case 0x378: motor_irq = 7; break;
		    case 0x278: motor_irq = 2; break;
		    case 0x3bc: motor_irq = 5; break;
		}
		if (motor_irq >= 0) {
		result = request_irq(motor_irq, motor_interrupt,
				0, "motorcontrol", NULL);
		if (result) {
			printk(KERN_ALERT "motorcontrol: can't get assigned irq %i\n",
					motor_irq);
			motor_irq = -1;
		}
		else { 
			outb(0x10,motor_base+2); //enable interrupts
		}
	}
	
	
	//start PWM thread
	ts=kthread_run(pwm_thread,NULL,"kthread");
	thread_started=1;
	
	return 0;
}
/*###############CLEANUP#############################*/
void cleanup_motor(void)
{
	//disable interrupt
	disable_irq(motor_irq);
	if (motor_irq >= 0) {
		outb(0x0, motor_base + 2);
		free_irq(motor_irq, NULL);
		printk(KERN_INFO "motorcontrol: irq freed\n");
	}
	//unregister device
	unregister_chrdev(device_major, DEVICE_NAME); 
	//stop thread if running
	if(thread_started) kthread_stop(ts);
	//release memory if used
	if (use_mem) {
		iounmap((void __iomem *)motor_base);
		release_mem_region(motor_base, MOTOR_NR_PORTS);
	} else {
		release_region(motor_base,MOTOR_NR_PORTS);
	}
	
	
	printk(KERN_INFO "motorcontrol: motorcontrol released\n");
}

//////////////////////////////////////////////////////////////////////////////////////////////

/*********************************************************************/ 
/* OPEN                                                              */
/*********************************************************************/ 
int motor_open(struct inode *inode, struct file *filp) { 
	//chef if module is in use
	if ( !try_module_get(THIS_MODULE) )
		return(-1);
	filp->private_data = &read_buffer; 
	return(0); 
}
/*********************************************************************/
/* RELEASE                                                           */
/*********************************************************************/
int motor_release(struct inode *inode, struct file *filp) { 
	//release module
	module_put(THIS_MODULE); 
	return(0); 
} 
/*********************************************************************/ 
/* READ                                                              */
/*********************************************************************/ 
char output_buffer[35];
unsigned int output_buffer_length = 0;
unsigned int read = 0;

ssize_t motor_read(struct file *filp, char *buf, size_t count, 
					loff_t *offset) { 
	int result;
	float buffer_f;
	unsigned int buffer_i;
	
	//check if already read byte -> return 0 -> all read
	if (read) {
		read = 0;
		return 0;
	}
	
	//calculate rpm
	
	//buffer_f = 1/(time_diff*1000000*60*30);
	//buffer_i = (long)buffer_f;
	
	//copy data to user
	output_buffer_length = sprintf(output_buffer, 
		"time_diff: %6luus \ninterrup_count: %8d\n", 
		time_diff/1000, interrupt_count);
	result = copy_to_user(buf, &output_buffer, output_buffer_length);
	if (result!=0) {
		printk(KERN_ALERT "motorcontrol: copy_to_user failed %d\n",result);
		return 0;
	}
	//set read -> return 0 next time
	read = 1;
	//returns number of read bytes
	return(output_buffer_length); 
} 
/*********************************************************************/
/* WRITE                                                             */
/*********************************************************************/ 
ssize_t motor_write(struct file *filp, const char *buf, size_t count, 
					loff_t *offset){ 
	struct DataBuffer *wb = filp->private_data;
	//copy data from user
	copy_from_user(&(wb->speed), buf, sizeof(char));
	
	//change PWM duty cycle
	if((wb->speed>=48)&&(wb->speed<=57))
		duty_cycle = wb->speed - 48;
		
	//returns number of written bytes
	return(sizeof(char)); 
} 
/*********************************************************************/ 
/* INIT / CLEANUP                                                    */ 
/*********************************************************************/ 

//kernel macros for init and release
module_init(init_motor);
module_exit(cleanup_motor);
