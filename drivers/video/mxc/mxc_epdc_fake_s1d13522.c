
// this file should be included by epdc driver from manufacturer .


#include "fake_s1d13522.h"
#include "lk_lm75.h"

#include <linux/completion.h>


#define FW_IN_RAM	1


#define _SCR_H	600
#define _SCR_W	800


#define WF_INIT	0
#define WF_DU	1
#define WF_GC16	2
#define WF_GC4	3
//
static EPDFB_DC *gptDC;

// global mxc update data ....
static struct mxcfb_update_data g_mxc_upd_data;
extern check_hardware_name(void);


static int giIsInited = 0;
DECLARE_COMPLETION(mxc_epdc_fake13522_inited);

//
// private help functions prototype ...
//


//////////////////////////////////////////////////////
//
// driver extention helper functions ...
//
int mxc_epdc_fb_check_update_complete(u32 update_marker, struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info:g_fb_data;
	struct update_marker_data *next_marker;
	struct update_marker_data *temp;
	unsigned long flags;
	bool marker_found = false;
	int ret = 0;
	
	//GALLEN_DBGLOCAL_BEGIN();

	/* 0 is an invalid update_marker value */
	if (update_marker == 0) {
		//GALLEN_DBGLOCAL_ESC();
		return -EINVAL;
	}

	/*
	 * Find completion associated with update_marker requested.
	 * Note: If update completed already, marker will have been
	 * cleared, it won't be found, and function will just return.
	 */

	/* Grab queue lock to protect access to marker list */
	spin_lock_irqsave(&fb_data->queue_lock, flags);

	list_for_each_entry_safe(next_marker, temp,
		&fb_data->full_marker_list, full_list) {
		//GALLEN_DBGLOCAL_RUNLOG(0);
		if (next_marker->update_marker == update_marker) {
			//GALLEN_DBGLOCAL_RUNLOG(1);
			dev_dbg(fb_data->dev, "Waiting for marker %d\n",
				update_marker);
			next_marker->waiting = true;
			marker_found = true;
			break;
		}
	}

	spin_unlock_irqrestore(&fb_data->queue_lock, flags);

	/*
	 * If marker not found, it has either been signalled already
	 * or the update request failed.  In either case, just return.
	 */
	if (!marker_found) {
		//GALLEN_DBGLOCAL_ESC();
		return ret;
	}

	ret = completion_done(&next_marker->update_completion)?1:0;


	//GALLEN_DBGLOCAL_END();
	return ret;
}
//EXPORT_SYMBOL(mxc_epdc_fb_check_update_complete);



//////////////////////////////////////////////////////
//
// fake_s1d13522 HAL interface .
//


static void k_fake_s1d13522_progress_start(void)
{
	if(0==giIsInited) {
		printk("[%s]:skip before init .",__FUNCTION__);
		return ;
	}
	
	//fake_s1d13522_progress_start(gptDC);
}

static int32_t k_fake_s1d13522_ioctl(unsigned int cmd,unsigned long arg)
{
	if(0==giIsInited) {
		if(in_interrupt()) {
			printk("[%s]:skip before init (interrupt).",__FUNCTION__);
		}
		else {
			printk("[%s]:wait init .",__FUNCTION__);
			wait_for_completion(&mxc_epdc_fake13522_inited);
		}
	}
	
	return fake_s1d13522_ioctl(cmd,arg,gptDC);
}


static void k_vcom_enable(int iIsEnable)
{
	if(iIsEnable) {
		//vcom enable .
	}
	else {
		//vcom disable .
	}
}

static int k_get_wfbpp(void)
{
	int i_wf_bpp=4;
	
	return i_wf_bpp;
}

static void k_set_partial(int iIsSetPartial)
{
	u32 temp;
	if(0==giIsInited) {
		printk("[%s]:skip before init .",__FUNCTION__);
		return ;
	}
	
	if(iIsSetPartial) {
		g_mxc_upd_data.update_mode = UPDATE_MODE_PARTIAL;
	}
	else {
		g_mxc_upd_data.update_mode = UPDATE_MODE_FULL;
	}
}

static unsigned char *k_get_realfbEx(unsigned long *O_pdwFBSize)
{
	unsigned char *pbRet ;
	if(0==giIsInited) {
		printk("[%s]:skip before init .",__FUNCTION__);
		return 0;
	}
	
	//pbRet = (unsigned char *)g_fb_data->working_buffer_virt;
	pbRet = (unsigned char *)g_fb_data->info.screen_base;
	if(O_pdwFBSize) {
		*O_pdwFBSize = g_fb_data->info.screen_size;
	}
	
	return pbRet;
}

static void k_display_start(int iIsStart)
{
	int iChk;
	if(0==giIsInited) {
		printk("[%s]:skip before init .",__FUNCTION__);
		return ;
	}
	
	
	if(iIsStart) {
		#if 0
		printk("%s(%d):==============================>\n",__FUNCTION__,__LINE__) ;
		printk("\t%d bits/pixel\n",g_fb_data->info.var.bits_per_pixel) ;
		printk("\t grayscale=%d \n",g_fb_data->info.var.grayscale) ;
		printk("\t yoffset=%d \n",g_fb_data->info.var.yoffset) ;
		printk("\t rotate=%d \n",g_fb_data->info.var.rotate) ;
		printk("\t activate=%d \n",g_fb_data->info.var.activate) ;
		printk("<======================================\n") ;
		#endif

		DBG_MSG("%s() (x,y)=(%u,%u),(w,h)(%u,%u)\n",__FUNCTION__,
			g_mxc_upd_data.update_region.top,g_mxc_upd_data.update_region.left,
			g_mxc_upd_data.update_region.height,g_mxc_upd_data.update_region.width);

		
		iChk = mxc_epdc_fb_send_update(&g_mxc_upd_data,&g_fb_data->info);
		if(iChk<0) {
			printk(KERN_WARNING"%s(%d):mxc_epdc_fb_send_update fail !\n",
				__FUNCTION__,__LINE__);
		}

	}
	else {
	}
}


static int k_get_wfmode(void)
{
	if(0==giIsInited) {
		printk("[%s]:skip before init .",__FUNCTION__);
		return 0;
	}
	
	int i_wf_mode = g_mxc_upd_data.waveform_mode;
	return i_wf_mode;
}

static void k_set_wfmode(int iWaveform)
{
	if(0==giIsInited) {
		printk("[%s]:skip before init .",__FUNCTION__);
		return ;
	}
	
	g_mxc_upd_data.waveform_mode = iWaveform;
}


static int k_is_updating(void)
{
	int iRet;
	int iChk = 0;

	if(0==giIsInited) {
		printk("[%s]:skip before init .",__FUNCTION__);
		return 0;
	}

	//if(epdc_is_working_buffer_busy()) 
	iChk = mxc_epdc_fb_check_update_complete(g_mxc_upd_data.update_marker,&g_fb_data->info);
	
	if(1==iChk)
	{
		// updating ...
		iRet = 1;
	}
	else {
		// update done 
		iRet = 0;
	}
	return iRet;
}

static int k_wait_update_complete(void) 
{
	int iRet;
	if(0==giIsInited) {
		printk("[%s]:skip before init .",__FUNCTION__);
		return 0;
	}
	

	if(1==g_mxc_upd_data.waveform_mode||4==g_mxc_upd_data.waveform_mode) {
		// skip wait update complete at DOC mode and A2 mode .
		iRet = 0;
	}
	else if(k_is_updating()){
		unsigned long dwJiffiesStart,dwJiffiesEnd;
		dwJiffiesStart = jiffies ;
		iRet = mxc_epdc_fb_wait_update_complete(g_mxc_upd_data.update_marker++,&g_fb_data->info);
		dwJiffiesEnd = jiffies;
		printk("[%s]waitupdate ret=%d,%u->%u\n",__FUNCTION__,iRet,dwJiffiesStart,dwJiffiesEnd);
	}
	return iRet;
}

// calling by real epdc driver .
static int k_set_temperature(struct fb_info *info)
{
	int iRet;
	int iChk;
	
	static int giLastTemprature = DEFAULT_TEMP;
	static unsigned long gdwLastUpdateJiffies = 0;
	
	//printk("%s()\n",__FUNCTION__);
	if(0==gdwLastUpdateJiffies||time_after(jiffies,gdwLastUpdateJiffies+6000)) {
		iChk = lm75_get_temperature(0,&giLastTemprature);
		if(iChk>=0) {
			iChk = mxc_epdc_fb_set_temperature(giLastTemprature,info);
			gdwLastUpdateJiffies = jiffies;
		}
	}
	return giLastTemprature;
}

static int k_set_update_rect(unsigned short wX,unsigned short wY,
	unsigned short wW,unsigned short wH)
{
	int iRet = 0;
	if(0==giIsInited) {
		printk("[%s]:skip before init .",__FUNCTION__);
		return 0;
	}
	
	
	#if 1
	DBG_MSG("%s() x=%u,y=%u,w=%u,h=%u\n",__FUNCTION__,wX,wY,wW,wH);
	g_mxc_upd_data.update_region.top = wY;
	g_mxc_upd_data.update_region.left = wX;
	g_mxc_upd_data.update_region.height = wH;
	g_mxc_upd_data.update_region.width = wW;	
	#else
	g_mxc_upd_data.update_region.top = 0;
	g_mxc_upd_data.update_region.left = 0;
	g_mxc_upd_data.update_region.height = _SCR_H;
	g_mxc_upd_data.update_region.width = _SCR_W;
	#endif
	
	return iRet;
}

static int k_fake_s1d13522_init(unsigned char *pbInitDCbuf)
{
	//gptDC = fake_s1d13522_initEx(4,0);
	gptDC = fake_s1d13522_initEx(default_bpp,g_fb_data->info.screen_base);
	if(gptDC) {
		gptDC->pfnGetWaveformBpp = k_get_wfbpp;
		gptDC->pfnVcomEnable = k_vcom_enable;
		gptDC->pfnSetPartialUpdate = k_set_partial;
		//gptDC->pfnGetRealFrameBuf = k_get_realfb;
		gptDC->pfnGetRealFrameBufEx = k_get_realfbEx;
		gptDC->pfnDispStart = k_display_start;
		gptDC->pfnGetWaveformMode = k_get_wfmode;
		gptDC->pfnSetWaveformMode = k_set_wfmode;
		gptDC->pfnIsUpdating = k_is_updating;
		gptDC->pfnWaitUpdateComplete = k_wait_update_complete;
		gptDC->pfnSetUpdateRect = k_set_update_rect;
		
		//gptDC->dwFlags |= EPDFB_DC_FLAG_OFB_RGB565;
		gptDC->dwFlags |= EPDFB_DC_FLAG_FLASHDIRTY;
		
		// 
		g_mxc_upd_data.update_region.top = 0;
		g_mxc_upd_data.update_region.left = 0;
		g_mxc_upd_data.update_region.height = _SCR_H;
		g_mxc_upd_data.update_region.width = _SCR_W;
		
		//g_mxc_upd_data.waveform_mode = g_fb_data->wv_modes.mode_gc16;
		g_mxc_upd_data.waveform_mode = WF_GC16;
		
		g_mxc_upd_data.update_mode = UPDATE_MODE_FULL;
		g_mxc_upd_data.update_marker = 0;
		g_mxc_upd_data.temp = TEMP_USE_AMBIENT;
		g_mxc_upd_data.flags = 0;
		//g_mxc_upd_data.alt_buffer_data = ;
		
		if (4 == check_hardware_name())
			lm75_init (3);
		else
			lm75_init (2);
		
		giIsInited = 1;
		complete_all(&mxc_epdc_fake13522_inited);
		
		//while(k_is_updating()) {
			//DBG0_MSG("%s(%d):wait for update done .\n");
			//schedule();
		//}
		if(pbInitDCbuf) {
			#if 1
			if(k_get_wfbpp() == 4) 
			{
				k_set_wfmode(WF_GC16); // fill LUT with default waveform, for 4bit, use mode 2
			}
			else {
				k_set_wfmode(WF_GC4); // fill LUT with default waveform, for 3bit, use mode 3 (GC)
			}	
			#endif	
			
			fake_s1d13522_display_img(0,0,gptDC->dwWidth,gptDC->dwHeight,
				pbInitDCbuf,gptDC,4,0);
				
			
		}
		fake_s1d13522_progress_start(gptDC);
			
		return 0;
	}
	else {
		printk("%s(%d): init fail !!\n",__FUNCTION__,__LINE__);
		return -1;
	}
}

