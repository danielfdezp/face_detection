#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

/************************** BASE ADDRESSES AND OFFSETS *************************/
#define XPAR_AXI_TIMER_0_BASEADDR   0x80020000
#define XPAR_AXI_TIMER_1_BASEADDR   0x80040000
#define TCSR0_OFFSET                0x00000000
#define TLR0_OFFSET                 0x00000004
#define TCSR1_OFFSET                0x00000010
#define TLR1_OFFSET                 0x00000014

/*********************************** FLAGS ************************************/
#define XTC_CSR_ENABLE_TMR_MASK		0x00000080 
#define XTC_CSR_LOAD_MASK			    0x00000020
#define XTC_CSR_ENABLE_PWM_MASK  	0x00000200 
#define XTC_CSR_EXT_GENERATE_MASK	0x00000004 
#define XTC_CSR_DOWN_COUNT_MASK 	0x00000002 
#define XTC_CSR_AUTO_RELOAD_MASK	0x00000010 
#define XTC_CSR_CASC_MASK			    0x00000800 
#define XTC_CSR_CAPTURE_MODE_MASK	0x00000001 
#define XTC_CSR_ENABLE_ALL_MASK		0x00000400 
#define XTC_CSR_INT_OCCURED_MASK	0x00000100 

/******************************* PWM VALUES ***********************************/
#define PWM_PERIOD					        0x00030d40 /* 200,000 -> 2ms (max position) */
#define MIN_POS					            0x000186a0 /* 100,000 -> 1ms (min position) */
#define NEUTRAL_POS                 0x000249f0 /* 150,000 -> 1.5ms  */
#define DEADBAND_WIDTH              0x000002bc /* 700     -> 7us    */

/**************************** GLOBAL VARIABLES ********************************/
uint32_t PageSize=4000;
uint32_t angleH = NEUTRAL_POS, angleV = NEUTRAL_POS;

/************************** FUNCTION PROTOTYPES *******************************/
int InitMappingsDevMem(uint32_t Axi_Timer_1_BaseAddr, uint32_t Axi_Timer_2_BaseAddr, void **Axi_Timer_1_Ptr, void **Axi_Timer_2_Ptr);
int WriteHw32(void *HwPtr, uint32_t Offset, uint32_t Value);
int ReadHw32(void *HwPtr, uint32_t Offset, uint32_t *Value);
void InitHw (void *Axi_Timer_1_Ptr, void *Axi_Timer_2_Ptr);
void PwmDisable (void *Axi_Timer_1_Ptr, void *Axi_Timer_2_Ptr); 
void PwmConfigure (void *Axi_Timer_Ptr, uint32_t HighTime);
void PwmEnable (void *Axi_Timer_Ptr);

/******************************************************************************/
/**
* Axi Timer addresses are mapped based on their Base Addresses.
*
* @param	Axi_Timer_1_BaseAddr is the base address of both timers.
*
* @param	Axi_Timer_1_Ptr is the mapped address
*
* @return	0 in case there was no issue, -1 if the mapping failed
*
* @note		none.
*
******************************************************************************/
int InitMappingsDevMem(uint32_t Axi_Timer_1_BaseAddr, uint32_t Axi_Timer_2_BaseAddr, void **Axi_Timer_1_Ptr, void **Axi_Timer_2_Ptr) {

	int File;

	// Open /dev/mem
	File = open ("/dev/mem", O_RDWR);
	if (File < 1) {
		printf("Failed to open /dev/mem \n\r");
		return -1;
	}

	if ((Axi_Timer_1_BaseAddr & (~(PageSize-1))) != Axi_Timer_1_BaseAddr) {
		printf("Address for Axi Timer 1 not a page boundary \n\r");
		return -1;
	}
	*Axi_Timer_1_Ptr = mmap(NULL, PageSize, PROT_READ|PROT_WRITE, MAP_SHARED, File, Axi_Timer_1_BaseAddr);

	if ((Axi_Timer_2_BaseAddr & (~(PageSize-1))) != Axi_Timer_2_BaseAddr) {
		printf("Address for Axi Timer 2 not a page boundary \n\r");
		return -1;
	}
	*Axi_Timer_2_Ptr = mmap(NULL, PageSize, PROT_READ|PROT_WRITE, MAP_SHARED, File, Axi_Timer_2_BaseAddr);


	return 0;
}

/*****************************************************************************/
/**
* Write a specified value to a register of a timer counter.
*
* @param	HwPtr is the base address of the timer counter device.
*
* @param	Offset contain the offset from the 1st register of the timer
*		counter to select the specific register of the timer counter.
* @param	Value is the 32 bit value to be written to the register.
*
* @return	0 in case there was no issue, -1 if the writing failed
*
* @note		none.
*
******************************************************************************/
int WriteHw32(void *HwPtr, uint32_t Offset, uint32_t Value) {

	if (Offset >= PageSize) {
		printf("Offset goes beyond page limit\n\r");
		return -1;
	}

	*((uint32_t *)(HwPtr + Offset)) = Value;        
	return 0;
}

/*****************************************************************************/
/**
* Read one of the timer counter registers.
*
* @param	HwPtr is the base address of the timer counter device.
*
* @param	Offset contains the offset from the 1st register of the timer
*		counter to select the specific register of the timer counter.
*
* @param	Value is the value read from the register, a 32 bit value.
*
* @return	0 in case there was no issue, -1 if the reading failed
*
* @note		none.
*
******************************************************************************/
int ReadHw32(void *HwPtr, uint32_t Offset, uint32_t *Value) {

	if (Offset >= PageSize) {
		printf("Offset goes beyond page limit\n\r");
		return -1;
	}

	*Value = *((uint32_t *)(HwPtr + Offset));

	return 0;
}

/*****************************************************************************/
/**
* (Re-)initialzes all timer counters which aren't started already.
*
* @param	Axi_Timer_1_Ptr contains the virtual address.
*
* @return	none.
*
* @note		none.
*
******************************************************************************/
void InitHw (void *Axi_Timer_1_Ptr, void *Axi_Timer_2_Ptr) 
{
	int i;
	uint32_t Axi_Timer[2] = {Axi_Timer_1_Ptr, Axi_Timer_2_Ptr};

	for (i=0; i<2; i++) {
		/* Set the compare register to 0 */
		WriteHw32(Axi_Timer[i], TCSR0_OFFSET, 0);
		WriteHw32(Axi_Timer[i], TCSR1_OFFSET, 0);

		/* Reset the timer and the interrupt */
		WriteHw32(Axi_Timer[i], TCSR0_OFFSET, XTC_CSR_INT_OCCURED_MASK | XTC_CSR_LOAD_MASK);
		WriteHw32(Axi_Timer[i], TCSR1_OFFSET, XTC_CSR_INT_OCCURED_MASK | XTC_CSR_LOAD_MASK);	

		/* Release the reset */
		WriteHw32(Axi_Timer[i], TCSR0_OFFSET, 0);
		WriteHw32(Axi_Timer[i], TCSR1_OFFSET, 0);
	}	
}

/*****************************************************************************/
/**
*
* Disables the PWM output.
*
* @param	Axi_Timer_1_Ptr contains the virtual address.
*
* @return	none.
*
* @note		Call to this function disables only the PWM output and do not
*		alter any configuration. PWM output can again be enabled by
*		calling PwmEnable without the need of re-configuration.
*
******************************************************************************/
void PwmDisable (void *Axi_Timer_1_Ptr, void *Axi_Timer_2_Ptr) 
{	
	int i;
	uint32_t Axi_Timer[2] = {Axi_Timer_1_Ptr, Axi_Timer_2_Ptr};
	uint32_t ControlStatusReg1, ControlStatusReg2; 
	uint32_t ResetCounterReg1, ResetCounterReg2;
	uint32_t CounterControlReg;

	for (i=0; i<2; i++) {
		/* --------------------STOP CODE-------------------------*/
		/* Read the current register contents */
		ReadHw32(Axi_Timer[i], TCSR0_OFFSET, &ControlStatusReg1);
		ReadHw32(Axi_Timer[i], TCSR1_OFFSET, &ControlStatusReg2); 

		/* Disable the timer counter such that it's not running */
		ControlStatusReg1 &= ~XTC_CSR_ENABLE_TMR_MASK; 
		ControlStatusReg2 &= ~XTC_CSR_ENABLE_TMR_MASK;

		/* Write out the updated value to the actual register */
		WriteHw32(Axi_Timer[i], TCSR0_OFFSET, ControlStatusReg1);
		WriteHw32(Axi_Timer[i], TCSR1_OFFSET, ControlStatusReg2);

		/* --------------------RESET CODE-------------------------*/
		/* Read current contents of the register so it won't be destroyed */
		ReadHw32(Axi_Timer[i], TCSR0_OFFSET, &ResetCounterReg1);
		ReadHw32(Axi_Timer[i], TCSR1_OFFSET, &ResetCounterReg2);

		/* Reset the timer by toggling the reset bit in the register */
		WriteHw32(Axi_Timer[i], TCSR0_OFFSET, ResetCounterReg1 | XTC_CSR_LOAD_MASK);
		WriteHw32(Axi_Timer[i], TCSR1_OFFSET, ResetCounterReg2 | XTC_CSR_LOAD_MASK);

		WriteHw32(Axi_Timer[i], TCSR0_OFFSET, ResetCounterReg1);
		WriteHw32(Axi_Timer[i], TCSR1_OFFSET, ResetCounterReg2);

		/* ---------------------DISABLE CODE-----------------------*/
		/* Disable PWM, Generate Out */
		ReadHw32(Axi_Timer[i], TCSR0_OFFSET, &CounterControlReg);
		CounterControlReg &= ~(XTC_CSR_ENABLE_PWM_MASK | XTC_CSR_EXT_GENERATE_MASK);
		WriteHw32(Axi_Timer[i], TCSR0_OFFSET, CounterControlReg);

		ReadHw32(Axi_Timer[i], TCSR1_OFFSET, &CounterControlReg);
		CounterControlReg &= ~(XTC_CSR_ENABLE_PWM_MASK | XTC_CSR_EXT_GENERATE_MASK);
		WriteHw32(Axi_Timer[i], TCSR1_OFFSET, CounterControlReg);
	}	
}

/*****************************************************************************/
/**
*
* Configures timers to generate PWM output.
*
* @param	Axi_Timer_Ptr contains the virtual address.
*
* @return	none.
*
* @note		This function needs to be called before enabling PWM otherwise
*		the output of PWM may be indeterminate. Here Down count mode of
*		timers are used for generating PWM output.
*
******************************************************************************/
void PwmConfigure (void *Axi_Timer_Ptr, uint32_t HighTime)
{
	uint32_t CounterControlReg, period, time;

	/* Configure timer modes to be used for PWM */
	ReadHw32(Axi_Timer_Ptr, TCSR0_OFFSET, &CounterControlReg);
	CounterControlReg |= (XTC_CSR_DOWN_COUNT_MASK | XTC_CSR_AUTO_RELOAD_MASK);
	CounterControlReg &= ~(XTC_CSR_CASC_MASK | XTC_CSR_EXT_GENERATE_MASK);
	WriteHw32(Axi_Timer_Ptr,	TCSR0_OFFSET, CounterControlReg);

	ReadHw32(Axi_Timer_Ptr, TCSR1_OFFSET, &CounterControlReg);
	CounterControlReg |= (XTC_CSR_DOWN_COUNT_MASK | XTC_CSR_AUTO_RELOAD_MASK);
	CounterControlReg &= ~(XTC_CSR_CASC_MASK | XTC_CSR_EXT_GENERATE_MASK);
	WriteHw32(Axi_Timer_Ptr, TCSR1_OFFSET, CounterControlReg);

	/* Set period and high time for PWM */
	WriteHw32(Axi_Timer_Ptr, TLR0_OFFSET, PWM_PERIOD);
	//ReadHw32(Axi_Timer_1_Ptr, TLR0_OFFSET, &period);
	//printf("Periodo: %x\n", period);
	WriteHw32(Axi_Timer_Ptr, TLR1_OFFSET, HighTime);
	//ReadHw32(Axi_Timer_1_Ptr, TLR1_OFFSET, &time);
	//printf("Time: %x\n", time);

	/* Configure timers in generate mode */
	ReadHw32(Axi_Timer_Ptr, TCSR0_OFFSET, &CounterControlReg);
	CounterControlReg &= ~(XTC_CSR_CAPTURE_MODE_MASK);
	WriteHw32(Axi_Timer_Ptr,	TCSR0_OFFSET, CounterControlReg);

	ReadHw32(Axi_Timer_Ptr, TCSR1_OFFSET, &CounterControlReg);
	CounterControlReg &= ~(XTC_CSR_CAPTURE_MODE_MASK);
	WriteHw32(Axi_Timer_Ptr, TCSR1_OFFSET, CounterControlReg);		
}

/*****************************************************************************/
/**
*
* Enables the PWM output as per configurations set by PwmConfigure.
*
* @param	Axi_Timer_Ptr contains the virtual address.
*
* @return	none.
*
* @note		none.
*
******************************************************************************/
void PwmEnable (void *Axi_Timer_Ptr)
{
	uint32_t CounterControlReg;
	uint32_t ResetCounterReg1, ResetCounterReg2;

	/* Enable PWM and Generate Out */
	ReadHw32(Axi_Timer_Ptr, TCSR0_OFFSET, &CounterControlReg);
	CounterControlReg |= (XTC_CSR_ENABLE_PWM_MASK | XTC_CSR_EXT_GENERATE_MASK);
	WriteHw32(Axi_Timer_Ptr, TCSR0_OFFSET, CounterControlReg);

	ReadHw32(Axi_Timer_Ptr, TCSR1_OFFSET, &CounterControlReg);
	CounterControlReg |= (XTC_CSR_ENABLE_PWM_MASK | XTC_CSR_EXT_GENERATE_MASK);
	WriteHw32(Axi_Timer_Ptr, TCSR1_OFFSET, CounterControlReg);

	/* --------------------RESET CODE-------------------------*/
	/* Read current contents of the register so it won't be destroyed */
	ReadHw32(Axi_Timer_Ptr, TCSR0_OFFSET, &ResetCounterReg1);
	ReadHw32(Axi_Timer_Ptr, TCSR1_OFFSET, &ResetCounterReg2);

	/* Reset the timer by toggling the reset bit in the register */
	WriteHw32(Axi_Timer_Ptr, TCSR0_OFFSET, ResetCounterReg1 | XTC_CSR_LOAD_MASK);
	WriteHw32(Axi_Timer_Ptr, TCSR1_OFFSET, ResetCounterReg2 | XTC_CSR_LOAD_MASK);

	WriteHw32(Axi_Timer_Ptr, TCSR0_OFFSET, ResetCounterReg1);
	WriteHw32(Axi_Timer_Ptr, TCSR1_OFFSET, ResetCounterReg2);
	/* -------------------------------------------------------*/

	/* Enable all timers */
	ReadHw32(Axi_Timer_Ptr, TCSR0_OFFSET, &CounterControlReg);
	CounterControlReg |= XTC_CSR_ENABLE_ALL_MASK;
	WriteHw32(Axi_Timer_Ptr,	TCSR0_OFFSET, CounterControlReg);
}

/*****************************************************************************/
/*
* Main function of the code
*/
/******************************************************************************/
cv::Mat process_result(cv::Mat &m1, const vitis::ai::FaceDetectResult &result, bool is_jpeg) {
    cv::Mat image;
    cv::resize(m1, image, cv::Size{result.width, result.height});
    cv::Point center;
    int x_pos, y_pos;
    void *Axi_Timer_1_Ptr = NULL, *Axi_Timer_2_Ptr = NULL;

    for (const auto &r : result.rects) {
        LOG_IF(INFO, is_jpeg) << " " << r.score << " "  //
                            << r.x << " "             //
                            << r.y << " "             //
                            << r.width << " "         //
                            << r.height;
        cv::rectangle(image,cv::Rect{cv::Point(r.x * image.cols, r.y * image.rows),cv::Size{(int)(r.width * image.cols),(int)(r.height * image.rows)}},0xff);

        cv::circle(image, cv::Point(r.x * image.cols + r.width*image.cols*0.5, r.y * image.rows + r.height*image.rows*0.5), 3, 0xff);
        center = cv::Point(r.x*image.cols + r.width*image.cols*0.5, r.y*image.rows+r.height*image.rows*0.5);
        x_pos = r.x*image.cols + r.width*image.cols*0.5;
        y_pos = r.y*image.rows + r.height*image.rows*0.5;
        std::cout << "Central point coordinates" << x_pos << "," << y_pos << "\n" << center << "\n" << image.cols << "x" << image.rows << std::endl;

        // Initialize physical memory mappings in order to access peripherals
        if (InitMappingsDevMem(XPAR_AXI_TIMER_0_BASEADDR, XPAR_AXI_TIMER_1_BASEADDR, &Axi_Timer_1_Ptr, &Axi_Timer_2_Ptr))
            printf("Failed to map Axi Timers physical addresses");

        else {
            printf("Physical addresses of Axi Timers have been correctly mapped\n\r");
            /* Initialize the timer counter so that it's ready to use */
            InitHw(Axi_Timer_1_Ptr, Axi_Timer_2_Ptr);

		    /* Disable PWM for reconfiguration */
		    PwmDisable(Axi_Timer_1_Ptr, Axi_Timer_2_Ptr);

            // Pan 
            if (x_pos < 30 && x_pos != 0) {              // Object moves to the right -> camera rotates to the left side
                angleH += DEADBAND_WIDTH;                // Camera rotates counterclockwise
                /* Configure PWM */
                PwmConfigure(Axi_Timer_1_Ptr, angleH);
                /* Enable PWM */
                PwmEnable(Axi_Timer_1_Ptr);
            }

            else if (x_pos > 615 && x_pos != 0) {        // Object moves to the left -> camera rotates to the right side
                angleH -= DEADBAND_WIDTH;                // Camera rotates clockwise
                /* Configure PWM */
                PwmConfigure(Axi_Timer_1_Ptr, angleH);
                /* Enable PWM */
                PwmEnable(Axi_Timer_1_Ptr);
            }

            // Tilt
            if (y_pos > 330 && y_pos != 0) {             // Object moves down -> camera looks down
                angleV += DEADBAND_WIDTH;                // Camera rotates counterclockwise
                /* Configure PWM */
                PwmConfigure(Axi_Timer_2_Ptr, angleV);
                /* Enable PWM */
                PwmEnable(Axi_Timer_2_Ptr);
            }

            else if (y_pos < 30 && y_pos != 0){          // Object moves up -> camera looks up
                angleV -= DEADBAND_WIDTH;                // Camera rotates clockwise
                /* Configure PWM */
                PwmConfigure(Axi_Timer_2_Ptr, angleV);
                /* Enable PWM */
                PwmEnable(Axi_Timer_2_Ptr);
            }
        }
    } 

  return image;
}
