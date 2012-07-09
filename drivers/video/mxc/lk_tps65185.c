

/*
 * purpose : TPS65185 driver
 *
 * author : Gallen Lin
 * versions :
 *
 */


#include <linux/kernel.h>
//#include <linux/config.h>

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/iomux-mx50.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/wait.h>


#include "ntx_hwconfig.h"

#define GDEBUG	0
#include <linux/gallen_dbg.h>

#include "lk_tps65185.h"


// command byte definitions ...

#define DRIVERNAME "TPS65185"
#define REG_UNKOWN_VAL	0xcc

#define TOTAL_CHIPS		1

//MODULE_LICENSE("GPL");



//////////////////////////////////////////////////////////////
// definitions gpio ...
//  GPIO output
#define GPIO_TPS65185_PWRUP		(2*32 + 30) /* GPIO_3_30 */
#define GPIO_TPS65185_WAKEUP	(2*32 + 29) /* GPIO_3_29 */
#define GPIO_TPS65185_VCOMCTRL	(3*32 + 21)	/* GPIO_4_21 */
#define GPIO_TPS65185_VIN		(0*32 + 27)	/* GPIO_1_27 */
//  GPIO input
#define GPIO_TPS65185_PWRGOOD	(2*32 + 28)	/* GPIO_3_28 */
#define GPIO_TPS65185_INT		(3*32 + 15)	/* GPIO_4_15 */

#define GPIO_TPS65185_SDA	(5*32+21) /* GPIO_6_21 */
#define GPIO_TPS65185_SDL	(5*32+20) /* GPIO_6_20 */

///////////////////////////////////////////////////////
// definitions for config_epd_timing() ...

typedef struct tagTPS65185_VERSIONS{
	unsigned char bMajor;
	unsigned char bMinor;
	unsigned char bVersion;
	unsigned char bRevID;
} TPS65185_VERSIONS;

typedef struct tagTPS65185_data {
	int iCurrent_temprature;
	unsigned short wTempratureData;
	unsigned long dwCurrent_mode;// active , sleep , standby .
	TPS65185_VERSIONS t65185_versions;
	int iCurrentPwrupState;
	int iCurrentWakeupState;
	int iIsInitPwrON;// is first power on or not , if yes ,you must to delay 1.8ms for i2c protocol initial .
	unsigned char bRegENABLE;
	//int iLast_temprature;
} TPS65185_data;


#define LKDRIVER_DATA_INIT(_iChipIdx)	\
{\
	gtTPS65185_DataA[_iChipIdx].iCurrent_temprature=-1;\
	gtTPS65185_DataA[_iChipIdx].wTempratureData=0;\
	gtTPS65185_DataA[_iChipIdx].dwCurrent_mode=TPS65185_MODE_UNKOWN;\
	gtTPS65185_DataA[_iChipIdx].iCurrentPwrupState=-1;\
	gtTPS65185_DataA[_iChipIdx].iCurrentWakeupState=-1;\
	gtTPS65185_DataA[_iChipIdx].iIsInitPwrON=1;\
}

static struct i2c_adapter *gpI2C_adapter = 0;
static struct i2c_client *gpI2C_clientA[TOTAL_CHIPS] = {0,};
volatile static int giIsTPS65185_inited=0;

static struct i2c_board_info gtTPS65185_BIA[TOTAL_CHIPS] = {
	{
	 .type = "tps65185-1",
	 .addr = 0x68,
	 .platform_data = NULL,
	 },
};


static TPS65185_data gtTPS65185_DataA[TOTAL_CHIPS] = {
	{-1,0x0000,TPS65185_MODE_UNKOWN,},
};



//////////////////////////////////////////////////////////////////////////
//
// internal hardware helper ...
//



// registers (write)....
#define TPS65185_REG_ENABLE_ACTIVE		0x80
#define TPS65185_REG_ENABLE_STANDBY		0x40
#define TPS65185_REG_ENABLE_V3P3_EN		0x20
#define TPS65185_REG_ENABLE_VCOM_EN		0x10
#define TPS65185_REG_ENABLE_VDDH_EN		0x08
#define TPS65185_REG_ENABLE_VPOS_EN		0x04
#define TPS65185_REG_ENABLE_VEE_EN		0x02
#define TPS65185_REG_ENABLE_VNEG_EN		0x01
#define TPS65185_REG_ENABLE_ALL			0xff
static volatile unsigned char gbTPS65185_REG_ENABLE=0; // default reset value is zero .
static const unsigned char gbTPS65185_REG_ENABLE_addr=0x01;

static volatile unsigned char gbTPS65185_REG_VADJ=0x23; // 15V .
static const unsigned char gbTPS65185_REG_VADJ_addr=0x02;


#define TPS65185_REG_VCOM1_ALL			0xff
static volatile unsigned char gbTPS65185_REG_VCOM1=0x7d; // .
static const unsigned char gbTPS65185_REG_VCOM1_addr=0x03;


#define TPS65185_REG_VCOM2_ACQ			0x80
#define TPS65185_REG_VCOM2_PROG			0x40
#define TPS65185_REG_VCOM2_HiZ			0x20
//#define TPS65185_REG_VCOM2_AVG			0x18
#define TPS65185_REG_VCOM2_VCOM8		0x01
#define TPS65185_REG_VCOM2_ALL			0xff
static volatile unsigned char gbTPS65185_REG_VCOM2=0x04; // .
static const unsigned char gbTPS65185_REG_VCOM2_addr=0x04;

//#define TPS65185_REG_INT_EN1_DTX_EN			0x80
#define TPS65185_REG_INT_EN1_TSD_EN					0x40
#define TPS65185_REG_INT_EN1_HOT_EN					0x20
#define TPS65185_REG_INT_EN1_TMST_HOT_EN			0x10
#define TPS65185_REG_INT_EN1_TMST_COLD_EN			0x08
#define TPS65185_REG_INT_EN1_UVLO_EN			0x04
#define TPS65185_REG_INT_EN1_ACQC_EN			0x02
#define TPS65185_REG_INT_EN1_PRGC_EN			0x01
#define TPS65185_REG_INT_EN1_ALL			0xff
static volatile unsigned char gbTPS65185_REG_INT_EN1=0x7f; // .
static const unsigned char gbTPS65185_REG_INT_EN1_addr=0x05;

#define TPS65185_REG_INT_EN2_VBUVEN				0x80
#define TPS65185_REG_INT_EN2_VDDHUVEN			0x40
#define TPS65185_REG_INT_EN2_VNUV_EN			0x20
#define TPS65185_REG_INT_EN2_VPOSUVEN			0x10
#define TPS65185_REG_INT_EN2_VEEUVEN			0x08
#define TPS65185_REG_INT_EN2_VCOMFEN			0x04
#define TPS65185_REG_INT_EN2_VNEGUVEN			0x02
#define TPS65185_REG_INT_EN2_EOCEN				0x01
#define TPS65185_REG_INT_EN2_ALL			0xff
static volatile unsigned char gbTPS65185_REG_INT_EN2=0xff; // .
static const unsigned char gbTPS65185_REG_INT_EN2_addr=0x06;

#define TPS65185_REG_INT1_ACQC			0x02
#define TPS65185_REG_INT1_PRGC			0x01
static volatile unsigned char gbTPS65185_REG_INT1=0x0; // .
static const unsigned char gbTPS65185_REG_INT1_addr=0x07;

static volatile unsigned char gbTPS65185_REG_INT2=0x0; // .
static const unsigned char gbTPS65185_REG_INT2_addr=0x08;



static volatile unsigned char gbTPS65185_REG_UPSEQ0=0xe4; // .
static const unsigned char gbTPS65185_REG_UPSEQ0_addr=0x09;

static volatile unsigned char gbTPS65185_REG_UPSEQ1=0x55; // .
static const unsigned char gbTPS65185_REG_UPSEQ1_addr=0x0a;


static volatile unsigned char gbTPS65185_REG_DWNSEQ0=0x1e; // .
static const unsigned char gbTPS65185_REG_DWNSEQ0_addr=0x0b;

static volatile unsigned char gbTPS65185_REG_DWNSEQ1=0xe0; // .
static const unsigned char gbTPS65185_REG_DWNSEQ1_addr=0x0c;

#define TPS65185_REG_TMST1_READ_THERM			0x80
#define TPS65185_REG_TMST1_CONV_END				0x20
static volatile unsigned char gbTPS65185_REG_TMST1=0x20; // .
static const unsigned char gbTPS65185_REG_TMST1_addr=0x0d;

//static unsigned char gbTPS65185_REG_TMST2=0x78; // .
//static const unsigned char gbTPS65185_REG_TMST2_addr=0x0e;

// registers (read)....
static volatile unsigned char gbTPS65185_REG_TMST_VALUE=0; //
static const unsigned char gbTPS65185_REG_TMST_VALUE_addr=0x00;

static volatile unsigned char gbTPS65185_REG_PG=0;
static const unsigned char gbTPS65185_REG_PG_addr=0x0f;

static volatile unsigned char gbTPS65185_REG_REVID=0x45; // default is TPS65185 1p0 .
static const unsigned char gbTPS65185_REG_REVID_addr=0x10;



static int tps65185_set_reg(unsigned char bRegAddr,unsigned char bRegSetVal)
{
	int iRet=TPS65185_RET_SUCCESS;
	int iChk;
	unsigned char bA[2] ;

#if 0
	ASSERT(gpI2C_adapter);
	ASSERT(gpI2C_clientA[0]);
#else
	if(!gpI2C_adapter) {
		WARNING_MSG("%s gpI2C_adapter null \n",__FUNCTION__);
		return (TPS65185_RET_PARAMERR);
	}
	if(!gpI2C_clientA[0]) { 
		WARNING_MSG("%s gpI2C_clientA[0] null \n",__FUNCTION__);
		return (TPS65185_RET_PARAMERR);
	}
#endif

	bA[0]=bRegAddr;
	bA[1]=bRegSetVal;
	iChk = i2c_master_send(gpI2C_clientA[0], (const char *)bA, sizeof(bA));
	if (iChk < 0) {
		ERR_MSG("%s(%d):%d=%s(),regAddr=0x%x,regVal=0x%x fail !\n",__FILE__,__LINE__,\
			iChk,"i2c_master_send",bRegAddr,bRegSetVal);
		return TPS65185_RET_I2CTRANS_ERR;
	}

	return iRet;
}

static int tps65185_get_reg(unsigned char bRegAddr,unsigned char  *O_pbRegVal)
{
	int iRet=TPS65185_RET_SUCCESS;
	int iChk;
	unsigned char bA[1] ;

	ASSERT(gpI2C_adapter);
	ASSERT(gpI2C_clientA[0]);

	ASSERT(O_pbRegVal);

	bA[0]=bRegAddr;
	
	
	iChk = i2c_master_send(gpI2C_clientA[0], (const char *)bA, 1);
	if (iChk < 0) {
		ERR_MSG("%s(%d):%s i2c_master_send fail !\n",__FILE__,__LINE__,__FUNCTION__);
		return TPS65185_RET_I2CTRANS_ERR;
	}
	

	iChk = i2c_master_recv(gpI2C_clientA[0], bA, 1);
	if (iChk < 0) {
		ERR_MSG("%s(%d):%s i2c_master_recv fail !\n",__FILE__,__LINE__,__FUNCTION__);
		return TPS65185_RET_I2CTRANS_ERR;
	}

	*O_pbRegVal = bA[0];

	return iRet;
}


#define TPS65185_REG_SET(_regName,_bFieldName,_bSetVal)		\
({\
	int _iRet=TPS65185_RET_SUCCESS;\
	int _iChk;\
	unsigned char _bNewReg,_bFieldMask;\
	\
	_bFieldMask=(unsigned char)TPS65185_REG_##_regName##_##_bFieldName;\
	if(0xff==_bFieldMask) {\
		_bNewReg = _bSetVal;\
	}\
	else {\
		_bNewReg=gbTPS65185_REG_##_regName;\
		if(_bSetVal) {\
			_bNewReg |= _bFieldMask ;\
		}\
		else {\
			_bNewReg &= ~_bFieldMask;\
		}\
	}\
	\
	_iChk = tps65185_set_reg(gbTPS65185_REG_##_regName##_##addr,_bNewReg);\
	if(_iChk<0) {\
		_iRet = _iChk;\
	}\
	else {\
		DBG_MSG("%s() : tps65185 write reg%s(%02Xh) 0x%02x->0x%02x\n",__FUNCTION__,\
		#_regName,gbTPS65185_REG_##_regName##_##addr,gbTPS65185_REG_##_regName,_bNewReg);\
		gbTPS65185_REG_##_regName = _bNewReg;\
	}\
	_iRet;\
})

#define TPS65185_REG_GET(_regName)		\
({\
	int _iChk;\
	unsigned char bReadReg=0;\
	unsigned short _wRet=0;\
	\
	_iChk = tps65185_get_reg(gbTPS65185_REG_##_regName##_##addr,&bReadReg);\
	if(_iChk<0) {\
		_wRet = (unsigned short)(-1);\
	}\
	else {\
		_wRet = bReadReg;\
		gbTPS65185_REG_##_regName = bReadReg;\
		DBG_MSG("%s() : tps65185 read reg%s(%02Xh)=0x%02x\n",__FUNCTION__,\
			#_regName,gbTPS65185_REG_##_regName##_##addr,bReadReg);\
	}\
	_wRet;\
})

#define TPS65185_REG(_regName)	gbTPS65185_REG_##_regName

DECLARE_WAIT_QUEUE_HEAD(tps65185_ACQC_WQ);
DECLARE_WAIT_QUEUE_HEAD(tps65185_PRGC_WQ);


static struct work_struct tps65185_int_work;
static struct workqueue_struct *tps65185_int_workqueue;

static void tps65185_int_func(struct work_struct *work)
{
	unsigned char bRegINT1,bRegINT2;
	unsigned short wReg;

	
	wReg = TPS65185_REG_GET(INT1);
	if(((unsigned short)(-1))==wReg) {
		ERR_MSG("%s(%d):%s regINT1 read fail !\n",__FILE__,__LINE__,__FUNCTION__);
	}
	bRegINT1=(unsigned char)wReg;
	
	wReg = TPS65185_REG_GET(INT2);
	if(((unsigned short)(-1))==wReg) {
		ERR_MSG("%s(%d):%s regINT2 read fail !\n",__FILE__,__LINE__,__FUNCTION__);
	}
	bRegINT2=(unsigned char)wReg;
	
	if(bRegINT1&TPS65185_REG_INT1_ACQC) {
		wake_up_all(&tps65185_ACQC_WQ);
	}
	
	if(bRegINT1&TPS65185_REG_INT1_PRGC) {
		wake_up_all(&tps65185_PRGC_WQ);
	}
	
	//DBG0_MSG("%s() : INT1=0x%x,INT2=0x%x\n",__FUNCTION__,bRegINT1,bRegINT2);
}


static irqreturn_t tps65185_int(int irq, void *dev_id)
{
	DBG_MSG("[%s-%d] tps65185 interrupt triggered !!!\n",__func__,__LINE__);
	queue_work(tps65185_int_workqueue,&tps65185_int_work);

	return 0;
}


//DECLARE_COMPLETION(tps65185_pwrgood_completion);
DECLARE_WAIT_QUEUE_HEAD(tps65185_pwron_wq);
DECLARE_WAIT_QUEUE_HEAD(tps65185_pwroff_wq);
static struct work_struct tps65185_pwrgood_work;
static struct workqueue_struct *tps65185_pwrgood_workqueue;

static void tps65185_pwrgood_func(struct work_struct *work)
{
	unsigned char bRegPG;
	unsigned short wReg;
	int iIsPwrOn;
	
	wReg = TPS65185_REG_GET(PG);
	if(((unsigned short)(-1))==wReg) {
		ERR_MSG("%s(%d):%s regPG read fail !\n",__FILE__,__LINE__,__FUNCTION__);
	}
	bRegPG=(unsigned char)wReg;
	
	iIsPwrOn=gpio_get_value(GPIO_TPS65185_PWRGOOD);
	DBG_MSG("%s() : powergood signal=%d , PG=0x%x\n",
		__FUNCTION__,iIsPwrOn,bRegPG);
	
	if(iIsPwrOn) {
		wake_up_all(&tps65185_pwron_wq);
	}
	else{
		wake_up_all(&tps65185_pwroff_wq);
	}
	//complete_all(&tps65185_pwrgood_completion);
}


static irqreturn_t tps65185_pwrgood_inthandler(int irq, void *dev_id)
{
	DBG_MSG("[%s-%d] tps65185 pwrgood interrupt triggered !!!\n",__func__,__LINE__);
	queue_work(tps65185_pwrgood_workqueue,&tps65185_pwrgood_work);

	return 0;
}


//
static int tps65185_gpio_init(void)
{
	int iRet=TPS65185_RET_SUCCESS;
	int iChk;
	int irq;

	// inputs
	mxc_iomux_v3_setup_pad(MX50_PAD_EPDC_PWRSTAT__GPIO_3_28);
	//mxc_iomux_v3_setup_pad(MX50_PAD_EPDC_PWRSTAT__GPIO_3_28_INT);
	gpio_request(GPIO_TPS65185_PWRGOOD, "tps65185_PWRGOOD");
	gpio_direction_input(GPIO_TPS65185_PWRGOOD);
    
	tps65185_pwrgood_workqueue=create_rt_workqueue("tps65185_PWRGOOD");
	INIT_WORK(&tps65185_pwrgood_work, tps65185_pwrgood_func);

	irq = gpio_to_irq(GPIO_TPS65185_PWRGOOD);
	set_irq_type(irq, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING);//


	
	mxc_iomux_v3_setup_pad(MX50_PAD_ECSPI1_SS0__GPIO_4_15);
	gpio_request(GPIO_TPS65185_INT, "tps65185_INT");
	gpio_direction_input (GPIO_TPS65185_INT);
	
	tps65185_int_workqueue=create_rt_workqueue("tps65185_INT");
	INIT_WORK(&tps65185_int_work, tps65185_int_func);

	irq = gpio_to_irq(GPIO_TPS65185_INT);
	set_irq_type(irq, IRQF_TRIGGER_FALLING);//IRQF_TRIGGER_RISING|


	// outputs
	mxc_iomux_v3_setup_pad(MX50_PAD_EIM_CRE__GPIO_1_27);
	gpio_request(GPIO_TPS65185_VIN, "tps65185_VIN");
	gpio_direction_output(GPIO_TPS65185_VIN, 1);

	mxc_iomux_v3_setup_pad(MX50_PAD_EPDC_PWRCTRL1__GPIO_3_30);
	gpio_request(GPIO_TPS65185_PWRUP, "tps65185_PWRUP");
	gpio_direction_output(GPIO_TPS65185_PWRUP, 0);

	mxc_iomux_v3_setup_pad(MX50_PAD_EPDC_PWRCTRL0__GPIO_3_29);
	gpio_request(GPIO_TPS65185_WAKEUP, "tps65185_WAKEUP");
	gpio_direction_output(GPIO_TPS65185_WAKEUP, 0);

	mxc_iomux_v3_setup_pad(MX50_PAD_EPDC_VCOM0__GPIO_4_21);
	gpio_request(GPIO_TPS65185_VCOMCTRL, "tps65185_VCOMCTRL");
	gpio_direction_output(GPIO_TPS65185_VCOMCTRL, 1);


	return iRet;
}


static int tps65185_gpio_release(void)
{
	int iRet=TPS65185_RET_SUCCESS;
	int irq;
	

	// release gpios ...
	
	// release interrupt ...
	irq = gpio_to_irq(GPIO_TPS65185_PWRGOOD);
	free_irq(irq,0);
	flush_workqueue(tps65185_pwrgood_workqueue);
	destroy_workqueue(tps65185_pwrgood_workqueue);

	irq = gpio_to_irq(GPIO_TPS65185_INT);
	free_irq(irq,0);
	flush_workqueue(tps65185_int_workqueue);
	destroy_workqueue(tps65185_int_workqueue);
	return iRet;
}







//////////////////////////////////////////////////////////////////////////
//
// internal helper ...
//



static int tps65185_chk_PG(unsigned char I_bChkMask)
{
	int iRet;
	unsigned short wReg;
	unsigned char bReg;


	wReg = TPS65185_REG_GET(PG);

	if(((unsigned short)(-1))==wReg) {
		return TPS65185_RET_REGREADFAIL;
	}

	bReg=(unsigned char)wReg;
	//bChkMask=0xfa;

	bReg &= ~I_bChkMask;
	if(I_bChkMask==bReg) {
		iRet = TPS65185_RET_ALLPOWERGOOD;
	}
	else {
		iRet = TPS65185_RET_POWERNOTGOOD;
	}

	return iRet;
}



static int tps65185_get_versions(TPS65185_VERSIONS *O_pt65185ver)
{
	int iRet=TPS65185_RET_SUCCESS;
	int iChk;
	unsigned short wReg;
	unsigned char bReg;

	ASSERT(gpI2C_adapter);
	ASSERT(O_pt65185ver);

	wReg = TPS65185_REG_GET(REVID);

	if(((unsigned short)(-1))==wReg) {
		return TPS65185_RET_REGREADFAIL;
	}

	bReg=(unsigned char)wReg;
	O_pt65185ver->bMajor = (bReg>>6)&0x3;
	O_pt65185ver->bMinor  = (bReg>>4)&0x3;
	O_pt65185ver->bVersion = (bReg)&0xf;
	O_pt65185ver->bRevID = bReg;

	return iRet;
}



static int tps65185_config_epd_timing(int iEPDTimingType)
{
	int iRet=TPS65185_RET_SUCCESS;


	return iRet;
}

static int tps65185_reg_init(void)
{
	int iRet=TPS65185_RET_SUCCESS;
	unsigned char bRegVal;

	GALLEN_DBGLOCAL_BEGIN();

	bRegVal = TPS65185_REG_ENABLE_V3P3_EN|TPS65185_REG_ENABLE_VCOM_EN|TPS65185_REG_ENABLE_VDDH_EN|\
			TPS65185_REG_ENABLE_VPOS_EN|TPS65185_REG_ENABLE_VEE_EN|TPS65185_REG_ENABLE_VNEG_EN;
	iRet = TPS65185_REG_SET(ENABLE,ALL,bRegVal);
	if(iRet<0) goto error;


	//bRegVal = 0;
	bRegVal = 0x7f;
	iRet = TPS65185_REG_SET(INT_EN1,ALL,bRegVal);
	if(iRet<0) goto error;

	bRegVal = 0xff;
	iRet = TPS65185_REG_SET(INT_EN2,ALL,bRegVal);
	if(iRet<0) goto error;

error:
	GALLEN_DBGLOCAL_END();
	return iRet;
}

// auto detect tps65185 .
// parameters :
// 	iPort : i2c channel in system (from 1~3) .
//  iEPDTimingType :
int tps65185_init(int iPort,int iEPDTimingType)
{
	int iRet = TPS65185_RET_SUCCESS;
	int iChipIdx;
	unsigned long dw65185mode;
	int iChk;
	int irq;
	
	GALLEN_DBGLOCAL_BEGIN();
	
	//printk ("%s(%d) \n",__func__,iPort);

	if(0!=giIsTPS65185_inited) {
		WARNING_MSG("%s(%d):skip ps65185 init twice !\n",__FILE__,__LINE__);
		return TPS65185_RET_SUCCESS;
	}

	iChk = tps65185_gpio_init();
	if(iChk<0) {
		ERR_MSG("[Error] %s : gpio init fail !\n",__FUNCTION__);
		GALLEN_DBGLOCAL_ESC();
		return iChk;
	}

	gpI2C_adapter = i2c_get_adapter(iPort-1);//
	if( NULL == gpI2C_adapter )
	{
		ERR_MSG ("[Error] %s : TPS65185_RET_I2CCHN_NOTFOUND\n",__FUNCTION__);
		GALLEN_DBGLOCAL_ESC();
		return TPS65185_RET_I2CCHN_NOTFOUND;
	}
	
	for(iChipIdx=0;iChipIdx<TOTAL_CHIPS;iChipIdx++) {
		gpI2C_clientA[iChipIdx] = i2c_new_device(gpI2C_adapter, &gtTPS65185_BIA[iChipIdx]);
		if(NULL == gpI2C_clientA[iChipIdx]) {
			tps65185_release();
			GALLEN_DBGLOCAL_ESC();
			ERR_MSG("[Error] %s : TPS65185_RET_NEWDEVFAIL\n",__FUNCTION__);
			return TPS65185_RET_NEWDEVFAIL;
		}
		printk("client%d ,addr=0x%x,name=%s\n",iChipIdx,
			gpI2C_clientA[iChipIdx]->addr,gpI2C_clientA[iChipIdx]->name);

		LKDRIVER_DATA_INIT(iChipIdx);
	}
	
	giIsTPS65185_inited=1;

	// change TPS65185 to standby mode .
	dw65185mode = TPS65185_MODE_STANDBY;
	iChk = tps65185_chg_mode(&dw65185mode);
	if(iChk<0) {
		ERR_MSG("[Error] %s : change to standby mode fail !\n",__FUNCTION__);
		tps65185_release();
		GALLEN_DBGLOCAL_ESC();
		return iChk;
	}

	iChk = tps65185_reg_init();
	if(iChk<0) {
		ERR_MSG("[Error] %s : tps65185 regs init fail !\n",__FUNCTION__);
		tps65185_release();
		GALLEN_DBGLOCAL_ESC();
		return iChk;
	}



	tps65185_get_versions(&gtTPS65185_DataA[0].t65185_versions);
	printk("TPS65185 versions : major=0x%x,minor=0x%x,version=0x%x,RevID=0x%x\n",
			gtTPS65185_DataA[0].t65185_versions.bMajor,
			gtTPS65185_DataA[0].t65185_versions.bMinor,
			gtTPS65185_DataA[0].t65185_versions.bVersion,
			gtTPS65185_DataA[0].t65185_versions.bRevID);

	//tps65185_get_temperature(0,0);// test .
	
	
	//
	// config registers of TPS65185 ...
	//
	iChk = tps65185_config_epd_timing(iEPDTimingType);
	if(iChk<0) {
		ERR_MSG("[Error] %s : config EPD timing fail !\n",__FUNCTION__);
		tps65185_release();
		GALLEN_DBGLOCAL_ESC();
		return iChk;
	}

#if 1
	irq = gpio_to_irq(GPIO_TPS65185_PWRGOOD);
	iChk = request_irq(irq, tps65185_pwrgood_inthandler, 0, "tps65185_PWRGOOD", 0);
	if (iChk) {
		pr_info("register TPS65185 pwrgood interrupt failed\n");
	}
	else {
		//enable_irq_wake(irq);
		//disable_irq_wake(irq);
		//disable_irq(irq);
	}
#endif
#if 1
	irq = gpio_to_irq(GPIO_TPS65185_INT);
	iChk = request_irq(irq, tps65185_int, 0, "tps65185_INT", 0);
	if (iChk) {
		pr_info("register TPS65185 interrupt failed\n");
	}
	else {
		//enable_irq_wake(irq);
		//disable_irq(irq);
	}
#endif
	

	/*
	dw65185mode = TPS65185_MODE_ACTIVE;
	iChk = tps65185_chg_mode(&dw65185mode);
	if(iChk<0) {
		ERR_MSG("[Error] %s : change to active mode fail !\n",__FUNCTION__);
		tps65185_release();
		GALLEN_DBGLOCAL_ESC();
		return iChk;
	}
	*/
	GALLEN_DBGLOCAL_END();
	return iRet;
}





int tps65185_release(void)
{
	int iRet = TPS65185_RET_SUCCESS;
	int iChipIdx ;
	int iChk;
	unsigned long dw65185mode ;
	
	GALLEN_DBGLOCAL_BEGIN();
	//printk("%s(%d):%s()\n",__FILE__,__LINE__,__FUNCTION__);
	for(iChipIdx=0;iChipIdx<TOTAL_CHIPS;iChipIdx++) {
		if(gpI2C_clientA[iChipIdx]) {
			i2c_unregister_device(gpI2C_clientA[iChipIdx]);
			gpI2C_clientA[iChipIdx] = NULL;
		}
		gtTPS65185_DataA[iChipIdx].iCurrent_temprature = -1;
		//gtTPS65185_DataA[iChipIdx].iLast_temprature = -1;
	}
	
	dw65185mode = TPS65185_MODE_SLEEP;
	iChk = tps65185_chg_mode(&dw65185mode);
	if(iChk<0) {
		ERR_MSG("[Error] %s : change to power down mode fail !\n",__FUNCTION__);
		tps65185_release();
		GALLEN_DBGLOCAL_ESC();
		return iChk;
	}


	iChk = tps65185_gpio_release();
	if(iChk<0) {
		WARNING_MSG("[Warnig] %s : gpio release fail !\n",__FUNCTION__);
	}

	gpI2C_adapter = NULL;
	
	GALLEN_DBGLOCAL_END();
	return iRet;
}





int tps65185_get_temperature(int *O_piTemperature)
{
	int iRet = TPS65185_RET_SUCCESS;
	int iChk;
	
	unsigned short wTemp,wReg;
	
	unsigned char bTemp,bReg;
	int iTemp;
	
	int iChipIdx = 0;

	unsigned long ulTimeoutTick;
	unsigned long ulCurTick;

	GALLEN_DBGLOCAL_BEGIN();

	//printk("%s()\n",__FUNCTION__);
	iChk = TPS65185_REG_SET(TMST1,READ_THERM,1);

	//udelay(1);
	ulTimeoutTick = jiffies + 50;
	do {

		wReg = TPS65185_REG_GET(TMST1);
		if(((unsigned short)(-1))==wReg) {
			GALLEN_DBGLOCAL_ESC();
			return TPS65185_RET_REGREADFAIL;
		}
		if(wReg&TPS65185_REG_TMST1_CONV_END) {
			break;
		}

		schedule_timeout(1);

		ulCurTick = jiffies;
		if(ulCurTick>ulTimeoutTick) {
			ERR_MSG("%s(%d):wait TMST1 ADC timeout \n",__FILE__,__LINE__);
			GALLEN_DBGLOCAL_ESC();
			return TPS65185_RET_TIMEOUT;
		}
	} while(1);
	
	wReg = TPS65185_REG_GET(TMST_VALUE);

	if(((unsigned short)(-1))==wReg) {
		GALLEN_DBGLOCAL_ESC();
		return TPS65185_RET_REGREADFAIL;
	}
	
	bReg = (unsigned char)wReg;
	gtTPS65185_DataA[iChipIdx].wTempratureData = wReg;
	if(bReg&0x80) {
		GALLEN_DBGLOCAL_RUNLOG(0);
		// negative .
		bTemp=(~bReg)+1;
		iTemp = bTemp;
		iTemp = (~iTemp)+1;
	}
	else {
		GALLEN_DBGLOCAL_RUNLOG(1);
		// positive .
		iTemp = (int)(bReg);
	}
	gtTPS65185_DataA[iChipIdx].iCurrent_temprature = iTemp;
	printk("%s temprature data = 0x%x,%d\n",DRIVERNAME,wReg,gtTPS65185_DataA[iChipIdx].iCurrent_temprature);
	
	//gtTPS65185_DataA[iChipIdx].iCurrent_temprature = bA[0];
	if(O_piTemperature) {
		GALLEN_DBGLOCAL_RUNLOG(2);
		*O_piTemperature = gtTPS65185_DataA[iChipIdx].iCurrent_temprature;
	}
	
	GALLEN_DBGLOCAL_END();
	return iRet;
}



int tps65185_is_panel_poweron(void)
{
	int iRet;
	#if 0
	int iChk;
	
	iChk = tps65185_chk_PG(0xfa);
	if(TPS65185_RET_ALLPOWERGOOD==iChk) {
		iRet = 1;
	}
	else {
		iRet = 0;
	}
	#else
	iRet = gpio_get_value(GPIO_TPS65185_PWRGOOD)?1:0;
	#endif
	
	return iRet;
}

int tps65185_wait_panel_poweron(void)
{
	int iRet = TPS65185_RET_SUCCESS;
	int iChk;
	#if 0
	if(in_interrupt()) {
		printk("%s(%d):[warning] call %s in interrupt !!\n",
			__FILE__,__LINE__,__FUNCTION__);
	}
	else 
	#endif 
	{
		//wait_for_completion(&tps65185_pwrgood_completion);
		iChk = wait_event_timeout(tps65185_pwron_wq,tps65185_is_panel_poweron(),50);
		if(!tps65185_is_panel_poweron()) {
			iRet = TPS65185_RET_TIMEOUT;
			ERR_MSG("%s(%d):wait power on timeout !\n",__FILE__,__LINE__);
		}
	}
	return iRet;
}

int tps65185_wait_panel_poweroff(void)
{
	int iRet = TPS65185_RET_SUCCESS;
	int iChk;
	#if 0
	if(in_interrupt()) {
		printk("%s(%d):[warning] call %s in interrupt !!\n",
			__FILE__,__LINE__,__FUNCTION__);
	}
	else 
	#endif
	{
		iChk = wait_event_timeout(tps65185_pwroff_wq,!tps65185_is_panel_poweron(),50);
		if(tps65185_is_panel_poweron()) {
			iRet = TPS65185_RET_TIMEOUT;
			ERR_MSG("%s(%d):wait power off timeout !\n",__FILE__,__LINE__);
		}
	}
	return iRet;
}


int tps65185_chg_mode(unsigned long *IO_pdwMode)
{
	int iRet=TPS65185_RET_SUCCESS;
	unsigned long dwCurrent_mode = gtTPS65185_DataA[0].dwCurrent_mode;
	unsigned long dwNewMode;


	int iCurrentWakeupState,iCurrentPwrupState,iNewWakeupState,iNewPwrupState;

	GALLEN_DBGLOCAL_BEGIN();

	if(0==giIsTPS65185_inited) {
		WARNING_MSG("[Error] %s : tps65185 must be initialized first !\n",__FUNCTION__);
		GALLEN_DBGLOCAL_ESC();
		return TPS65185_RET_INITNOTYET;
	}

	if(!IO_pdwMode) {
		WARNING_MSG("[Warning] %s : parameter error !\n",__FUNCTION__);
		GALLEN_DBGLOCAL_ESC();
		return TPS65185_RET_PARAMERR;
	}

	dwNewMode = *IO_pdwMode;
	if(dwCurrent_mode == dwNewMode) {
		DBG_MSG("%s : skip same mode \n",__FUNCTION__);
		GALLEN_DBGLOCAL_ESC();
		return TPS65185_RET_SUCCESS;
	}


	// change tps65185 work mode ...
	iCurrentWakeupState = gpio_get_value(GPIO_TPS65185_WAKEUP);
	iCurrentPwrupState = gpio_get_value(GPIO_TPS65185_PWRUP);

	switch(dwNewMode) {
	case TPS65185_MODE_ACTIVE:GALLEN_DBGLOCAL_RUNLOG(0);
		iNewWakeupState = 1;
		iNewPwrupState = 1;
		if(iNewWakeupState != iCurrentWakeupState) {
			GALLEN_DBGLOCAL_RUNLOG(1);
			gpio_set_value(GPIO_TPS65185_WAKEUP, iNewWakeupState);
			if(gtTPS65185_DataA[0].iIsInitPwrON) {
				GALLEN_DBGLOCAL_RUNLOG(2);
				gtTPS65185_DataA[0].iIsInitPwrON = 0;
				mdelay(2);// follow spec to delay 1.8ms .
			}
		}
		if(TPS65185_MODE_SLEEP==dwCurrent_mode) {
			GALLEN_DBGLOCAL_RUNLOG(3);
			gpio_set_value(GPIO_TPS65185_PWRUP, iNewPwrupState);
		}
		else {
			GALLEN_DBGLOCAL_RUNLOG(4);
			iRet = TPS65185_REG_SET(ENABLE,ACTIVE,1);
		}
		break;

	case TPS65185_MODE_SLEEP:GALLEN_DBGLOCAL_RUNLOG(5);
		iNewWakeupState = 0;
		iNewPwrupState = iCurrentPwrupState;
		gpio_set_value(GPIO_TPS65185_WAKEUP, iNewWakeupState);
		break;

	case TPS65185_MODE_STANDBY:GALLEN_DBGLOCAL_RUNLOG(6);
		iNewWakeupState = 1;
		iNewPwrupState = 0;
		if(iNewWakeupState != iCurrentWakeupState) {
			GALLEN_DBGLOCAL_RUNLOG(7);
			gpio_set_value(GPIO_TPS65185_WAKEUP, iNewWakeupState);
			if(gtTPS65185_DataA[0].iIsInitPwrON) {
				GALLEN_DBGLOCAL_RUNLOG(8);
				gtTPS65185_DataA[0].iIsInitPwrON = 0;
				mdelay(2);// follow spec to delay 1.8ms .
			}
		}
		if(TPS65185_MODE_SLEEP==dwCurrent_mode) {
			GALLEN_DBGLOCAL_RUNLOG(9);
			gpio_set_value(GPIO_TPS65185_PWRUP, iNewPwrupState);
			msleep(1);// to avoid system hands up ,while suspend to resume .
		}
		else {
			GALLEN_DBGLOCAL_RUNLOG(10);
			iRet = TPS65185_REG_SET(ENABLE,STANDBY,1);
		}

		break;

	default:
		GALLEN_DBGLOCAL_RUNLOG(11);
		WARNING_MSG("%s : mode unsupported (0x%x) !!\n",__FUNCTION__,
			(unsigned int)dwNewMode);
		GALLEN_DBGLOCAL_ESC();
		return TPS65185_RET_PARAMERR;
	}

	DBG_MSG("%s :[mode] 0x%x->0x%x [wakeup] %d->%d, [pwrup] %d->%d \n",__FUNCTION__,\
			(unsigned int)dwCurrent_mode,(unsigned int)dwNewMode,\
			iCurrentWakeupState,iNewWakeupState,\
			iCurrentPwrupState,iNewPwrupState);

	gtTPS65185_DataA[0].dwCurrent_mode = dwNewMode;
	gtTPS65185_DataA[0].iCurrentPwrupState = iCurrentPwrupState;
	gtTPS65185_DataA[0].iCurrentWakeupState = iCurrentWakeupState;

	*IO_pdwMode = dwCurrent_mode;

	GALLEN_DBGLOCAL_END();
	return iRet;
}



int tps65185_vcom_set(int I_iVCOM_mv,int iIsWriteToFlash)
{
	int iRet = TPS65185_RET_SUCCESS;
	int iChk;
	unsigned short wVCOM_val;
	unsigned long dwTPS65185_mode;
	int iChkVCOM;
	//
	
	GALLEN_DBGLOCAL_BEGIN();
	
	// TPS65185 should be in Standby or Active mode .
	dwTPS65185_mode = TPS65185_MODE_STANDBY;
	iChk = tps65185_chg_mode(&dwTPS65185_mode);
	
	// set VCOM @ VCOM1/VCOM2 register ...
	if(I_iVCOM_mv>0) {
		GALLEN_DBGLOCAL_RUNLOG(0);
		I_iVCOM_mv = 0;
		wVCOM_val = 0;
	}
	else {
		GALLEN_DBGLOCAL_RUNLOG(1);
		wVCOM_val = (unsigned short)((-I_iVCOM_mv)/10);
	}
	
	if(wVCOM_val&0x100) {
		GALLEN_DBGLOCAL_RUNLOG(2);
		TPS65185_REG_SET(VCOM2,VCOM8,1);
	}
	TPS65185_REG_SET(VCOM1,ALL,(unsigned char)wVCOM_val);
	
	if(iIsWriteToFlash) {
#if 1//[

		GALLEN_DBGLOCAL_RUNLOG(3);
		TPS65185_REG_SET(INT_EN1,PRGC_EN,1);
		TPS65185_REG_SET(VCOM2,PROG,1);
		iChk = wait_event_timeout(tps65185_PRGC_WQ,
			TPS65185_REG(INT1)&TPS65185_REG_INT1_PRGC,100);
		if(!(TPS65185_REG(INT1)&TPS65185_REG_INT1_PRGC)) {
			iRet = TPS65185_RET_TIMEOUT;
			ERR_MSG("%s(%d):wait TPS65185 PRGC timeout !\n",__FILE__,__LINE__);
		}
#else//!][
		{
			unsigned long ulTimeoutTick,ulCurTick;
			unsigned short wReg;

			GALLEN_DBGLOCAL_RUNLOG(4);
			//printk("%s()\n",__FUNCTION__);
			iChk = TPS65185_REG_SET(INT_EN1,PRGC_EN,1);
			iChk = TPS65185_REG_SET(VCOM2,PROG,1);

			//udelay(1);
			ulTimeoutTick = jiffies + 100;
			do {

				wReg = TPS65185_REG(INT1);
				if(((unsigned short)(-1))==wReg) {
					GALLEN_DBGLOCAL_ESC();
					return TPS65185_RET_REGREADFAIL;
				}
				if(wReg&TPS65185_REG_INT1_PRGC) {
					GALLEN_DBGLOCAL_RUNLOG(5);
					break;
				}


				GALLEN_DBGLOCAL_RUNLOG(6);
				schedule_timeout(1);

				ulCurTick = jiffies;
				if(ulCurTick>ulTimeoutTick) {

					ERR_MSG("%s(%d):wait TMST1 ADC timeout ,wReg=0x%x\n",
							__FILE__,__LINE__,wReg);

					GALLEN_DBGLOCAL_ESC();
					return TPS65185_RET_TIMEOUT;
				}

			} while(1);
		}
#endif//]

#if 1
		dwTPS65185_mode = TPS65185_MODE_SLEEP;
		iChk = tps65185_chg_mode(&dwTPS65185_mode);
	
		dwTPS65185_mode = TPS65185_MODE_STANDBY;
		iChk = tps65185_chg_mode(&dwTPS65185_mode);

		iChk = tps65185_vcom_get(&iChkVCOM);
		//printk("%s:iChkVCOM = %d mV\n",__FUNCTION__,iChkVCOM);
		if(iChkVCOM!=(I_iVCOM_mv/10*10)) {

			ERR_MSG("%s(%d):VCOM check fail (%d!=%d)!\n",
					__FILE__,__LINE__,iChk,I_iVCOM_mv);

			GALLEN_DBGLOCAL_ESC();
			return TPS65185_RET_VCOMWRFAIL;
		}
#endif

	}
	GALLEN_DBGLOCAL_END();

	return iRet;
}

int tps65185_vcom_get(int *O_piVCOM_mv)
{
	int iRet = TPS65185_RET_SUCCESS;
	
	unsigned char bRegVCOM1,bRegVCOM2;
	unsigned short wTemp;
	int iTemp;
	
	if(O_piVCOM_mv) {
		bRegVCOM2 = TPS65185_REG_GET(VCOM2);
		bRegVCOM1 = TPS65185_REG_GET(VCOM1);
		wTemp = (bRegVCOM1|bRegVCOM2<<8)&0x1ff;
		iTemp = -(wTemp*10);
		*O_piVCOM_mv = iTemp;
	}
	else {
		iRet = TPS65185_RET_PARAMERR;
	}
	
	return iRet;
}

int tps65185_vcom_kickback_measurement(int *O_piVCOM_mv)
{
	int iRet = TPS65185_RET_SUCCESS;
	int iChk;

	GALLEN_DBGLOCAL_BEGIN();
	
	if(O_piVCOM_mv) {
		unsigned long dwTPS65185_mode;
	
		
		// Pull the WAKEUP pin and the PWRUP pin to enable all output rails .
		dwTPS65185_mode = TPS65185_MODE_ACTIVE;
		iChk = tps65185_chg_mode(&dwTPS65185_mode);
		// Set the HiZ bit in the VCOM2 register. This puts the VCOM pin in high-impedance state .
		TPS65185_REG_SET(VCOM2,HiZ,1);
		
		// Drive the panel with the Null waveform. 
		
		
		// Set ACQ bt in the VCOM2 register to 1. This starts the mesurement routine .
		TPS65185_REG_SET(VCOM2,ACQ,1);
		
		// When the measurement is complete, the ACQC (Acquisition Complete) 
		//   bit in the INT1 register is set and the nINT pin is pulled low .
		
		iChk = wait_event_timeout(tps65185_ACQC_WQ,
			TPS65185_REG(INT1)&TPS65185_REG_INT1_ACQC,50);
		if(!(TPS65185_REG(INT1)&TPS65185_REG_INT1_ACQC)) {
			iRet = TPS65185_RET_TIMEOUT;
			ERR_MSG("%s(%d):wait TPS65185 ACQC timeout !\n",__FILE__,__LINE__);
		}
		
		// The measurement result is stored in the VCOM[8:0] bits of the VCOM1 and VCOM2 register .
		iRet = tps65185_vcom_get(O_piVCOM_mv);
		
	}
	else {
		iRet = TPS65185_RET_PARAMERR;
	}
	
	GALLEN_DBGLOCAL_END();
	return iRet;
}


#define TPS65185_SUSPEND		1	

void tps65185_suspend(void)
{
#ifdef TPS65185_SUSPEND //[
	unsigned long dwTPS65185_mode;
	unsigned char bVal;
	int irq;
	
	dbgENTER();

	flush_workqueue(tps65185_pwrgood_workqueue);
	flush_workqueue(tps65185_int_workqueue);

	tps65185_wait_panel_poweroff();

	irq = gpio_to_irq(GPIO_TPS65185_INT);
	disable_irq(irq);
	irq = gpio_to_irq(GPIO_TPS65185_PWRGOOD);
	disable_irq(irq);

	bVal=0;
	TPS65185_REG_SET(INT_EN1,ALL,bVal);
	TPS65185_REG_SET(INT_EN2,ALL,bVal);
	TPS65185_REG_SET(ENABLE,ALL,bVal);


	dwTPS65185_mode = TPS65185_MODE_SLEEP;
	tps65185_chg_mode(&dwTPS65185_mode);


#if 1

	udelay(50);
	mxc_iomux_v3_setup_pad(MX50_PAD_I2C2_SCL__GPIO_6_20);
	gpio_request(GPIO_TPS65185_SDL, "tps65185_i2c_sdl");
	gpio_direction_input (GPIO_TPS65185_SDL);
	//gpio_direction_output(GPIO_TPS65185_SDL, 0);

	mxc_iomux_v3_setup_pad(MX50_PAD_I2C2_SDA__GPIO_6_21);
	gpio_request(GPIO_TPS65185_SDA, "tps65185_i2c_sda");
	gpio_direction_input (GPIO_TPS65185_SDL);
	//gpio_direction_output(GPIO_TPS65185_SDA, 0);


	gpio_direction_output(GPIO_TPS65185_PWRUP, 0);
	gpio_direction_output(GPIO_TPS65185_WAKEUP, 0);

	//gpio_direction_output(GPIO_TPS65185_PWRGOOD, 0);
	//gpio_direction_output(GPIO_TPS65185_INT, 0);

	gpio_direction_output(GPIO_TPS65185_VCOMCTRL, 0);

	//gpio_direction_output(GPIO_TPS65185_VIN, 0);
#endif

	dbgLEAVE();
#endif //]TPS65185_SUSPEND
}

void tps65185_resume(void)
{
#ifdef TPS65185_SUSPEND //[
	unsigned long dwTPS65185_mode;
	int irq;

	dbgENTER();
	
#if 1

	gpio_free(GPIO_TPS65185_SDL);
	gpio_free(GPIO_TPS65185_SDA);

	mxc_iomux_v3_setup_pad(MX50_PAD_I2C2_SCL__I2C2_SCL);
	mxc_iomux_v3_setup_pad(MX50_PAD_I2C2_SDA__I2C2_SDA);

	//gpio_direction_input(GPIO_TPS65185_PWRGOOD);
	//gpio_direction_input(GPIO_TPS65185_INT);
	

	gpio_direction_output(GPIO_TPS65185_VCOMCTRL, 1);

	//gpio_direction_output(GPIO_TPS65185_VIN, 1);
	
#endif

	dwTPS65185_mode = TPS65185_MODE_STANDBY;
	tps65185_chg_mode(&dwTPS65185_mode);

	tps65185_reg_init();

	irq = gpio_to_irq(GPIO_TPS65185_INT);
	enable_irq(irq);
	irq = gpio_to_irq(GPIO_TPS65185_PWRGOOD);
	enable_irq(irq);

	dbgLEAVE();
#endif //]TPS65185_SUSPEND
}

//EXPORT_SYMBOL(tps65185_init);
//EXPORT_SYMBOL(tps65185_release);
//EXPORT_SYMBOL(tps65185_get_temperature);

