#include "jadard_sorting.h"
extern struct jadard_module_fp g_module_fp;
extern struct jadard_ts_data *pjadard_ts_data;
extern struct jadard_ic_data *pjadard_ic_data;

void jadard_sorting_init(void);

static uint8_t LCD_Status = JD_LCD_STATUS_DISPLAY_ON;
static bool Fail_flag;
static int ModeEnter_Flag;
static bool Switch_Mode_Timeout_Flag;
static int PST_GetFrameOfRawData_Number;
static bool Get_Data_Finished_Flag;
static bool Collect_Data_Timeout_Flag;

static int jd_test_data_pop_out(char *rslt_buf, char *filepath)
{
#if 0
	struct file *raw_file = NULL;
	mm_segment_t fs;
	loff_t pos = 0;
	int ret_val = JD_SORTING_OK;

	JD_D("%s: Entering!\n", __func__);
	JD_I("data size=0x%04X\n", (uint32_t)strlen(rslt_buf));
	JD_I("Ouput result to %s\n", filepath);

	if (raw_file == NULL)
		raw_file = filp_open(filepath, O_TRUNC|O_CREAT|O_RDWR, 0660);

	raw_file = filp_open(filepath, O_TRUNC|O_CREAT|O_RDWR, 0660);
	if (IS_ERR(raw_file)) {
		JD_E("%s open file failed = %ld\n", __func__, PTR_ERR(raw_file));
		ret_val = -EIO;
		goto SAVE_DATA_ERR;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	kernel_write(raw_file, rslt_buf, JD_OUTPUT_BUFFER_SIZE * JD_SORTING_ACTIVE_ITEM * sizeof(char), &pos);
	if (raw_file != NULL) {
		filp_close(raw_file, NULL);
	}
	set_fs(fs);

SAVE_DATA_ERR:
	JD_D("%s: End!\n", __func__);

	return ret_val;
#else
	return 0;
#endif
}

static void jadard_sorting_data_deinit(void)
{
	int i = 0;

	if (jd_g_sorting_threshold) {
		for (i = 0; i < JD_READ_THRESHOLD_SIZE; i++) {
			kfree(jd_g_sorting_threshold[i]);
		}
		kfree(jd_g_sorting_threshold);
		jd_g_sorting_threshold = NULL;
		JD_I("Free the jd_g_sorting_threshold\n");
	}

	if (jd_g_file_path) {
		kfree(jd_g_file_path);
		jd_g_file_path = NULL;
	}

	if (jd_g_rslt_data) {
		kfree(jd_g_rslt_data);
		jd_g_rslt_data = NULL;
	}

	if (GET_RAWDATA) {
		kfree(GET_RAWDATA);
		GET_RAWDATA = NULL;
	}
}

static void jadard_pst_enter_normal_mode(void)
{
	uint8_t os_test_mode_password[JD_TWO_SIZE];

	os_test_mode_password[0] = JD_MPAP_PW_NORMAL_ACTIVE_hbyte;
	os_test_mode_password[1] = JD_MPAP_PW_NORMAL_ACTIVE_lbyte;
	g_module_fp.fp_APP_SetSortingMode(os_test_mode_password, sizeof(os_test_mode_password));
}

static void jadard_pst_enter_slpin_mode(void)
{
	JD_I("Start to Enter Sleep In\n");

	jadard_pst_enter_normal_mode();
	g_module_fp.fp_APP_GetLcdSleep(&LCD_Status);

	if (LCD_Status != JD_LCD_STATUS_SLEEP_IN) {
		g_module_fp.fp_dd_register_write(0x28, NULL, 0);
		g_module_fp.fp_dd_register_write(0x10, NULL, 0);
	}

	mdelay(100);
	g_module_fp.fp_APP_GetLcdSleep(&LCD_Status);

	if (LCD_Status != JD_LCD_STATUS_SLEEP_IN) {
		JD_I("Enter Sleep In Fail\n");
	} else {
		JD_I("Enter Sleep In Finished\n");
	}
}

static void jadard_pst_enter_slpout_mode(void)
{
	JD_I("Start to Enter Sleep Out\n");

	jadard_pst_enter_normal_mode();
	g_module_fp.fp_APP_GetLcdSleep(&LCD_Status);

	if (LCD_Status != JD_LCD_STATUS_SLEEP_OUT) {
		g_module_fp.fp_dd_register_write(0x11, NULL, 0);
		g_module_fp.fp_dd_register_write(0x29, NULL, 0);
	}

	mdelay(100);
	g_module_fp.fp_APP_GetLcdSleep(&LCD_Status);

	if (LCD_Status != JD_LCD_STATUS_SLEEP_OUT) {
		JD_I("Enter Sleep Out Fail\n");
	} else {
		JD_I("Enter Sleep Out Finished\n");
	}
}

static void jadard_PST_Enter_LpwugIdle_Diff_mode(void)
{
	uint8_t os_test_mode_password[JD_TWO_SIZE];

	os_test_mode_password[0] = JD_MPAP_PW_MP_LPWUG_IDLE_DIFF_START_hbyte;
	os_test_mode_password[1] = JD_MPAP_PW_MP_LPWUG_IDLE_DIFF_START_lbyte;
	g_module_fp.fp_APP_SetSortingMode(os_test_mode_password, sizeof(os_test_mode_password));
}

static void jadard_PST_Enter_LpwugIdle_mode(void)
{
	uint8_t os_test_mode_password[JD_TWO_SIZE];

	os_test_mode_password[0] = JD_MPAP_PW_MP_LPWUG_IDLE_START_hbyte;
	os_test_mode_password[1] = JD_MPAP_PW_MP_LPWUG_IDLE_START_lbyte;
	g_module_fp.fp_APP_SetSortingMode(os_test_mode_password, sizeof(os_test_mode_password));
}

static void jadard_PST_Enter_LpwugActive_Diff_mode(void)
{
	uint8_t os_test_mode_password[JD_TWO_SIZE];

	os_test_mode_password[0] = JD_MPAP_PW_MP_LPWUG_ACTIVE_DIFF_START_hbyte;
	os_test_mode_password[1] = JD_MPAP_PW_MP_LPWUG_ACTIVE_DIFF_START_lbyte;
	g_module_fp.fp_APP_SetSortingMode(os_test_mode_password, sizeof(os_test_mode_password));
}

static void jadard_PST_Enter_LpwugActive_mode(void)
{
	uint8_t os_test_mode_password[JD_TWO_SIZE];

	os_test_mode_password[0] = JD_MPAP_PW_MP_LPWUG_ACTIVE_START_hbyte;
	os_test_mode_password[1] = JD_MPAP_PW_MP_LPWUG_ACTIVE_START_lbyte;
	g_module_fp.fp_APP_SetSortingMode(os_test_mode_password, sizeof(os_test_mode_password));
}

static void jadard_PST_Enter_NormalIdle_Diff_mode(void)
{
	uint8_t os_test_mode_password[JD_TWO_SIZE];

	os_test_mode_password[0] = JD_MPAP_PW_MP_NORMAL_IDLE_DIFF_START_hbyte;
	os_test_mode_password[1] = JD_MPAP_PW_MP_NORMAL_IDLE_DIFF_START_lbyte;
	g_module_fp.fp_APP_SetSortingMode(os_test_mode_password, sizeof(os_test_mode_password));
}

static void jadard_PST_Enter_NormalIdle_mode(void)
{
	uint8_t os_test_mode_password[JD_TWO_SIZE];

	os_test_mode_password[0] = JD_MPAP_PW_MP_NORMAL_IDLE_START_hbyte;
	os_test_mode_password[1] = JD_MPAP_PW_MP_NORMAL_IDLE_START_lbyte;
	g_module_fp.fp_APP_SetSortingMode(os_test_mode_password, sizeof(os_test_mode_password));
}

static void jadard_PST_Enter_NormalActive_Diff_mode(void)
{
	uint8_t os_test_mode_password[JD_TWO_SIZE];

	os_test_mode_password[0] = JD_MPAP_MP_MP_NORMAL_ACTIVE_DIFF_START_hbyte;
	os_test_mode_password[1] = JD_MPAP_MP_MP_NORMAL_ACTIVE_DIFF_START_lbyte;
	g_module_fp.fp_APP_SetSortingMode(os_test_mode_password, sizeof(os_test_mode_password));
}

static void jadard_PST_Enter_NormalActive_mode(void)
{
	uint8_t os_test_mode_password[JD_TWO_SIZE];

	os_test_mode_password[0] = JD_MPAP_PW_MP_NORMAL_ACTIVE_START_hbyte;
	os_test_mode_password[1] = JD_MPAP_PW_MP_NORMAL_ACTIVE_START_lbyte;
	g_module_fp.fp_APP_SetSortingMode(os_test_mode_password, sizeof(os_test_mode_password));
}

static void jadard_pst_enter_short_mode(void)
{
	uint8_t os_test_mode_password[JD_TWO_SIZE];

	os_test_mode_password[0] = JD_MPAP_PW_MP_SHORT_START_hbyte;
	os_test_mode_password[1] = JD_MPAP_PW_MP_SHORT_START_lbyte;
	g_module_fp.fp_APP_SetSortingMode(os_test_mode_password, sizeof(os_test_mode_password));
}

static void jadard_pst_enter_open_mode(void)
{
	uint8_t os_test_mode_password[JD_TWO_SIZE];

	os_test_mode_password[0] = JD_MPAP_PW_MP_OPEN_START_hbyte;
	os_test_mode_password[1] = JD_MPAP_PW_MP_OPEN_START_lbyte;
	g_module_fp.fp_APP_SetSortingMode(os_test_mode_password, sizeof(os_test_mode_password));
}

static void jadard_ReadSortingMode(int *PST_MP_Mode_Index)
{
	uint8_t OS_TEST_MODE_PASSWORD[JD_TWO_SIZE];

	*PST_MP_Mode_Index = -1;
	OS_TEST_MODE_PASSWORD[0] = 0x00;
	OS_TEST_MODE_PASSWORD[1] = 0x00;

	g_module_fp.fp_APP_ReadSortingMode(OS_TEST_MODE_PASSWORD, sizeof(OS_TEST_MODE_PASSWORD));

	if ((OS_TEST_MODE_PASSWORD[0] == JD_MPAP_PW_MP_SORTING_FINISH_hbyte) && (OS_TEST_MODE_PASSWORD[1] == JD_MPAP_PW_MP_SORTING_FINISH_lbyte)) {
		*PST_MP_Mode_Index = JD_PST_MP_Mode_Sorting;
	}
	else if ((OS_TEST_MODE_PASSWORD[0] == JD_MPAP_PW_MP_SHORT_FINISH_hbyte) && (OS_TEST_MODE_PASSWORD[1] == JD_MPAP_PW_MP_SHORT_FINISH_lbyte)) {
		*PST_MP_Mode_Index = JD_PST_MP_Mode_Short;
	}
	else if ((OS_TEST_MODE_PASSWORD[0] == JD_MPAP_PW_MP_OPEN_FINISH_hbyte) && (OS_TEST_MODE_PASSWORD[1] == JD_MPAP_PW_MP_OPEN_FINISH_lbyte)) {
		*PST_MP_Mode_Index = JD_PST_MP_Mode_Open;
	}
	else if ((OS_TEST_MODE_PASSWORD[0] == JD_MPAP_PW_MP_NORMAL_ACTIVE_FINISH_hbyte) && (OS_TEST_MODE_PASSWORD[1] == JD_MPAP_PW_MP_NORMAL_ACTIVE_FINISH_lbyte)) {
		*PST_MP_Mode_Index = JD_PST_MP_Mode_MP_Normal_Active;
	}
	else if ((OS_TEST_MODE_PASSWORD[0] == JD_MPAP_PW_MP_NORMAL_ACTIVE_DIFF_FINISH_hbyte) && (OS_TEST_MODE_PASSWORD[1] == JD_MPAP_PW_MP_NORMAL_ACTIVE_DIFF_FINISH_lbyte)) {
		*PST_MP_Mode_Index = JD_PST_MP_Mode_MP_Normal_Active_Diff;
	}
	else if ((OS_TEST_MODE_PASSWORD[0] == JD_MPAP_MP_NORMAL_IDLE_FINISH_hbyte) && (OS_TEST_MODE_PASSWORD[1] == JD_MPAP_MP_NORMAL_IDLE_FINISH_lbyte)) {
		*PST_MP_Mode_Index = JD_PST_MP_Mode_MP_Normal_Idle;
	}
	else if ((OS_TEST_MODE_PASSWORD[0] == JD_MPAP_PW_MP_NORMAL_IDLE_DIFF_FINISH_hbyte) && (OS_TEST_MODE_PASSWORD[1] == JD_MPAP_PW_MP_NORMAL_IDLE_DIFF_FINISH_lbyte)) {
		*PST_MP_Mode_Index = JD_PST_MP_Mode_MP_Normal_Idle_Diff;
	}
	else if ((OS_TEST_MODE_PASSWORD[0] == JD_MPAP_PW_MP_LPWUG_ACTIVE_FINISH_hbyte) && (OS_TEST_MODE_PASSWORD[1] == JD_MPAP_PW_MP_LPWUG_ACTIVE_FINISH_lbyte)) {
		*PST_MP_Mode_Index = JD_PST_MP_Mode_MP_LPWUG_Active;
	}
	else if ((OS_TEST_MODE_PASSWORD[0] == JD_MPAP_PW_MP_LPWUG_ACTIVE_DIFF_FINISH_hbyte) && (OS_TEST_MODE_PASSWORD[1] == JD_MPAP_PW_MP_LPWUG_ACTIVE_DIFF_FINISH_lbyte)) {
		*PST_MP_Mode_Index = JD_PST_MP_Mode_MP_LPWUG_Active_Diff;
	}
	else if ((OS_TEST_MODE_PASSWORD[0] == JD_MPAP_PW_MP_LPWUG_IDLE_FINISH_hbyte) && (OS_TEST_MODE_PASSWORD[1] == JD_MPAP_PW_MP_LPWUG_IDLE_FINISH_lbyte)) {
		*PST_MP_Mode_Index = JD_PST_MP_Mode_MP_LPWUG_Idle;
	}
	else if ((OS_TEST_MODE_PASSWORD[0] == JD_MPAP_PW_MP_LPWUG_IDLE_DIFF_FINISH_hbyte) && (OS_TEST_MODE_PASSWORD[1] == JD_MPAP_PW_MP_LPWUG_IDLE_DIFF_FINISH_lbyte)) {
		*PST_MP_Mode_Index = JD_PST_MP_Mode_MP_LPWUG_Idle_Diff;
	}
}

static void jadard_PST_SwitchSortMode_Check(int *enter_flag, int PST_MP_Mode_Index, bool *switch_mode_timeout_flag)
{
	int counter = 0;
	*enter_flag = -1;
	*switch_mode_timeout_flag = false;

	while (*enter_flag != PST_MP_Mode_Index) {
		mdelay(50);
		jadard_ReadSortingMode(enter_flag);

		if (counter++ > 100) {
			*switch_mode_timeout_flag = true;
			*enter_flag = JD_PST_MP_Mode_Null;
			JD_E("Switch_SortMode_Fail\n");
			return;
		}
	}
}

static void jadard_PST_LpwugIdle_Diff_Mode(void)
{
	JD_I("Start to Enter LPWUG Idle Diff Mode\n");
	g_module_fp.fp_APP_SetSortingKeepFrame(0);
	g_module_fp.fp_APP_SetSortingSkipFrame((uint8_t)JD_SkipDiffData_Frame_Number);

	jadard_PST_Enter_LpwugIdle_Diff_mode();
	jadard_PST_SwitchSortMode_Check(&ModeEnter_Flag, JD_PST_MP_Mode_MP_LPWUG_Idle_Diff, &Switch_Mode_Timeout_Flag);

	if ((ModeEnter_Flag == JD_PST_MP_Mode_Null) && (Switch_Mode_Timeout_Flag == true)) {
		JD_E("Enter LPWUG Idle Diff Mode Fail\n");
		Fail_flag = true;
	} else {
		JD_I("Enter LPWUG Idle Diff Mode Finished\n");
	}
}

static void jadard_PST_LpwugIdle_Mode(void)
{
	JD_I("Start to Enter LPWUG Idle Mode\n");

	g_module_fp.fp_APP_SetSortingSkipFrame((uint8_t)JD_SkipRawData_Frame_Number);
	jadard_PST_Enter_LpwugIdle_mode();
	jadard_PST_SwitchSortMode_Check(&ModeEnter_Flag, JD_PST_MP_Mode_MP_LPWUG_Idle, &Switch_Mode_Timeout_Flag);

	if ((ModeEnter_Flag == JD_PST_MP_Mode_Null) && (Switch_Mode_Timeout_Flag == true)) {
		JD_E("Enter LPWUG Idle Mode Fail\n");
		Fail_flag = true;
	} else {
		JD_I("Enter LPWUG Idle Mode Finished\n");
	}
}

static void jadard_PST_LpwugActive_Diff_Mode(void)
{
	JD_I("Start to LPWUG Active Diff Mode\n");
	g_module_fp.fp_APP_SetSortingSkipFrame(0);
	g_module_fp.fp_APP_SetSortingSkipFrame((uint8_t)JD_SkipDiffData_Frame_Number);

	jadard_PST_Enter_LpwugActive_Diff_mode();
	jadard_PST_SwitchSortMode_Check(&ModeEnter_Flag, JD_PST_MP_Mode_MP_LPWUG_Active_Diff, &Switch_Mode_Timeout_Flag);

	if ((ModeEnter_Flag == JD_PST_MP_Mode_Null) && (Switch_Mode_Timeout_Flag == true)) {
		JD_E("Enter LPWUG Active Diff Mode Fail\n");
		Fail_flag = true;
	} else {
		JD_I("Enter LPWUG Active Diff Mode Finished\n");
	}
}

static void jadard_PST_LpwugActive_Mode(void)
{
	JD_I("Start to LPWUG Active Mode\n");

	g_module_fp.fp_APP_SetSortingSkipFrame((uint8_t)JD_SkipRawData_Frame_Number);
	jadard_PST_Enter_LpwugActive_mode();
	jadard_PST_SwitchSortMode_Check(&ModeEnter_Flag, JD_PST_MP_Mode_MP_LPWUG_Active, &Switch_Mode_Timeout_Flag);

	if ((ModeEnter_Flag == JD_PST_MP_Mode_Null) && (Switch_Mode_Timeout_Flag == true)) {
		JD_E("Enter LPWUG Active Mode Fail\n");
		Fail_flag = true;
	} else {
		JD_I("Enter LPWUG Active Mode Finished\n");
	}
}

static void jadard_PST_NormalIdle_Diff_Mode(void)
{
	JD_I("Start to Enter Normal Idle Diff Mode\n");
	g_module_fp.fp_APP_SetSortingSkipFrame(0);
	g_module_fp.fp_APP_SetSortingSkipFrame((uint8_t)JD_SkipRawData_Frame_Number);

	jadard_PST_Enter_NormalIdle_Diff_mode();
	jadard_PST_SwitchSortMode_Check(&ModeEnter_Flag, JD_PST_MP_Mode_MP_Normal_Idle_Diff, &Switch_Mode_Timeout_Flag);

	if ((ModeEnter_Flag == JD_PST_MP_Mode_Null) && (Switch_Mode_Timeout_Flag == true)) {
		JD_E("Enter Normal Idle Diff Mode Fail\n");
		Fail_flag = true;
	} else {
		JD_I("Enter Normal Idle Diff Mode Finished\n");
	}
}

static void jadard_PST_NormalIdle_Mode(void)
{
	JD_I("Start to Enter Normal Idle Mode\n");

	g_module_fp.fp_APP_SetSortingSkipFrame((uint8_t)JD_SkipDiffData_Frame_Number);
	jadard_PST_Enter_NormalIdle_mode();
	jadard_PST_SwitchSortMode_Check(&ModeEnter_Flag, JD_PST_MP_Mode_MP_Normal_Idle, &Switch_Mode_Timeout_Flag);

	if ((ModeEnter_Flag == JD_PST_MP_Mode_Null) && (Switch_Mode_Timeout_Flag == true)) {
		JD_E("Enter Normal Idle Mode Fail\n");
		Fail_flag = true;
	} else {
		JD_I("Enter Normal Idle Mode Finished\n");
	}
}

static void jadard_PST_NormalActive_Diff_Mode(void)
{
	JD_I("Start to Enter Normal Active Diff Mode\n");

	g_module_fp.fp_APP_SetSortingKeepFrame(0);
	g_module_fp.fp_APP_SetSortingSkipFrame((uint8_t)JD_SkipDiffData_Frame_Number);
	jadard_PST_Enter_NormalActive_Diff_mode();
	jadard_PST_SwitchSortMode_Check(&ModeEnter_Flag, JD_PST_MP_Mode_MP_Normal_Active_Diff, &Switch_Mode_Timeout_Flag);

	if ((ModeEnter_Flag == JD_PST_MP_Mode_Null) && (Switch_Mode_Timeout_Flag == true)) {
		JD_E("Enter Normal Active Diff Mode Fail\n");
		Fail_flag = true;
	} else {
		JD_I("Enter Normal Active Diff Mode Finished\n");
	}
}

static void jadard_PST_NormalActive_Mode(void)
{
	JD_I("Start to Enter Normal Active Mode\n");

	g_module_fp.fp_APP_SetSortingSkipFrame((uint8_t)JD_SkipRawData_Frame_Number);
	jadard_PST_Enter_NormalActive_mode();
	jadard_PST_SwitchSortMode_Check(&ModeEnter_Flag, JD_PST_MP_Mode_MP_Normal_Active, &Switch_Mode_Timeout_Flag);

	if ((ModeEnter_Flag == JD_PST_MP_Mode_Null) && (Switch_Mode_Timeout_Flag == true)) {
		JD_E("Enter Normal Active Mode Fail\n");
		Fail_flag = true;
	} else {
		JD_I("Enter Normal Active Mode Finished\n");
	}
}

static void jadard_PST_Short_Mode(void)
{
	JD_I("Start to Enter Short Mode\n");

	g_module_fp.fp_APP_SetSortingSkipFrame((uint8_t)JD_SkipRawData_Frame_Number);
	jadard_pst_enter_short_mode();
	jadard_PST_SwitchSortMode_Check(&ModeEnter_Flag, JD_PST_MP_Mode_Short, &Switch_Mode_Timeout_Flag);

	if ((ModeEnter_Flag == JD_PST_MP_Mode_Null) && (Switch_Mode_Timeout_Flag == true)) {
		JD_E("Enter Short Mode Fail\n");
		Fail_flag = true;
	} else {
		JD_I("Enter Short Mode Finished\n");
	}
}

static void jadard_PST_Open_Mode(void)
{
	JD_I("Start to Enter Open Mode\n");

	g_module_fp.fp_APP_SetSortingSkipFrame((uint8_t)JD_SkipRawData_Frame_Number);
	jadard_pst_enter_open_mode();
	jadard_PST_SwitchSortMode_Check(&ModeEnter_Flag, JD_PST_MP_Mode_Open, &Switch_Mode_Timeout_Flag);

	if ((ModeEnter_Flag == JD_PST_MP_Mode_Null) && (Switch_Mode_Timeout_Flag == true)) {
		JD_E("Enter Open Mode Fail\n");
		Fail_flag = true;
	} else {
		JD_I("Enter Open Mode Finished\n");
	}
}

static void jadard_PST_Enter_Diff_Type_Mode(void)
{
	JD_I("Start to Enter Diff Type Mode\n");
	g_module_fp.fp_mutual_data_set(JD_DATA_TYPE_Difference);
	mdelay(200);
	JD_I("Enter Diff Type Finished\n");
}

static void jadard_PST_Enter_RawData_Type_Mode(void)
{
	JD_I("Start to Enter RawData Type Mode\n");
	//jadard_pst_enter_normal_mode();
	g_module_fp.fp_mutual_data_set(JD_DATA_TYPE_RawData);
	mdelay(200);
	JD_I("Enter RawData Type Finished\n");
}

static void jadard_CPST_GetDataRun(void)
{
	uint16_t rdata_size = pjadard_ic_data->JD_X_NUM * pjadard_ic_data->JD_Y_NUM * sizeof(uint16_t);

	if (g_module_fp.fp_get_mutual_data(JD_DATA_TYPE_RawData, GET_RAWDATA, rdata_size) < 0) {
		Fail_flag = true;
	}
}

static void jadard_Sorting_BusyStatus_Check(void)
{
	int counter = 0;

	JD_I("Start Sorting_BusyStatus_Check\n");
	Collect_Data_Timeout_Flag = false;

	while (g_module_fp.fp_APP_ReadSortingBusyStatus(JD_MPAP_HANDSHAKE_FINISH) == false) {
		mdelay(50);

		if (counter++ > 200) {
			Collect_Data_Timeout_Flag = true;
			Fail_flag = true;
			JD_E("MPAP_HandShake_Fail\n");
			return;
		} else {
			Fail_flag = false;
		}
	}

	JD_I("Sorting_BusyStatus_Check Pass\n");
}

/* static void jadard_DataByteToDec(uint8_t *rdata, uint16_t rdata_len, int tx, int rx, int *maxdata, int *mindata)
{
	uint16_t value = 0;
	int tx_cnt = 0;
	int rx_cnt = 0;
	int i;

	for (i = 0; i < rdata_len / 2; i++) {
		if (i % 2 != 0) {
			if (rx_cnt == rx) {
				rx_cnt = 0;
				tx_cnt++;
			}

			value = (uint16_t)((rdata[i] << 8) + rdata[i - 1]);
			maxdata[tx_cnt, rx_cnt] = value;
			rx_cnt++;
		}
	}

	tx_cnt = 0;
	rx_cnt = 0;
	for (i = rdata_len / 2; i < rdata_len; i++) {
		if (i % 2 != 0) {
			if (rx_cnt == rx) {
				rx_cnt = 0;
				tx_cnt++;
			}
			value = (uint16_t)((rdata[i] << 8) + rdata[i - 1]);
			mindata[tx_cnt, rx_cnt] = value;
			rx_cnt++;
		}
	}
} */

static void jadard_Start_to_Get_DiffData_Function(void)
{
	JD_I("Start to Get Diff Data\n");
	memset(GET_RAWDATA, 0x00, pjadard_ic_data->JD_X_NUM * pjadard_ic_data->JD_Y_NUM * sizeof(uint16_t));

	PST_GetFrameOfRawData_Number = JD_GetDiffData_Frame_Number;
	if (PST_GetFrameOfRawData_Number < 3)
		PST_GetFrameOfRawData_Number = 3;

	g_module_fp.fp_APP_SetSortingKeepFrame(PST_GetFrameOfRawData_Number);

	jadard_Sorting_BusyStatus_Check();
	if (Fail_flag == true) {
		JD_E("Get Diff Data Fail\n");
	} else {
		g_module_fp.fp_GetSortingDiffData(GET_RAWDATA, pjadard_ic_data->JD_X_NUM * pjadard_ic_data->JD_Y_NUM * sizeof(uint16_t));
		//jadard_DataByteToDec(rdata, sizeof(rdata), pjadard_ic_data->JD_X_NUM, pjadard_ic_data->JD_Y_NUM, Diff_Max, Diff_Min);
		JD_I("Get Diff Data Finished\n");
	}
}

static void jadard_Start_to_Get_RawData_Function(void)
{
	JD_I("Start to Get Raw Data\n");
	memset(GET_RAWDATA, 0x00, pjadard_ic_data->JD_X_NUM * pjadard_ic_data->JD_Y_NUM * sizeof(uint16_t));

	PST_GetFrameOfRawData_Number = JD_GetRawData_Frame_Number;
	if (PST_GetFrameOfRawData_Number < 3) {
		PST_GetFrameOfRawData_Number = 3;
	}

	Get_Data_Finished_Flag = false;
	jadard_CPST_GetDataRun();

	if (Fail_flag == true) {
		JD_E("Get Raw Data Fail\n");
	} else {
		JD_I("Get Raw Data Finished\n");
	}
}

static void jd_test_data_get(uint8_t *RAW, char *start_log, char *result, int now_item)
{
	uint32_t i;
	int raw_data;
	uint32_t index = 0;
	ssize_t len = 0;
	char *testdata = NULL;
	uint32_t SZ_SIZE = JD_OUTPUT_BUFFER_SIZE;

	JD_D("%s: Entering, Now type=%s!\n", __func__, jd_action_item_name[now_item]);

	testdata = kzalloc(sizeof(char) * SZ_SIZE, GFP_KERNEL);

	len += snprintf((testdata + len), SZ_SIZE - len, "%s", start_log);
	for (i = 0; i < pjadard_ic_data->JD_Y_NUM*pjadard_ic_data->JD_X_NUM; i++) {
		if (pjadard_ts_data->rawdata_little_endian) {
			raw_data = (((int8_t)RAW[index + 1] << 8) | RAW[index]);
		} else {
			raw_data = (((int8_t)RAW[index] << 8) | RAW[index + 1]);
		}

		if (i > 1 && ((i + 1) % pjadard_ic_data->JD_Y_NUM) == 0) {
			len += snprintf((testdata + len), SZ_SIZE - len, "%5d,\n", raw_data);
		}	else {
			len += snprintf((testdata + len), SZ_SIZE - len, "%5d,", raw_data);
		}

		index += 2;
	}
	len += snprintf((testdata + len), SZ_SIZE - len, "\n%s", result);

	memcpy(&jd_g_rslt_data[now_item * SZ_SIZE], testdata, SZ_SIZE);

	/* dbg */
	/*for(i = 0; i < SZ_SIZE; i++)
	{
		I("0x%04X, ", jd_g_rslt_data[i + (now_item * SZ_SIZE)]);
		if(i > 0 && (i % 16 == 15))
			printk("\n");
	}*/

	kfree(testdata);
	JD_D("%s: End!\n", __func__);
}

static uint32_t jadard_Item_Test_Function(uint8_t checktype)
{
	int i;
	int raw_data;
	uint32_t index = 0;
	char *rslt_log = NULL;
	char *start_log = NULL;
	int ret_val = JD_SORTING_OK;

	rslt_log = kzalloc(256 * sizeof(char), GFP_KERNEL);
	start_log = kzalloc(256 * sizeof(char), GFP_KERNEL);

	if (checktype >= JD_SORTING_OPEN_CHECK) {
		sprintf(start_log, "\n%s%s\n", jd_action_item_name[checktype - JD_SORTING_OPEN_CHECK], ": data as follow!\n");
	}

	switch (checktype) {
	case JD_SORTING_OPEN_CHECK:
		for (i = 0; i < (pjadard_ic_data->JD_Y_NUM * pjadard_ic_data->JD_X_NUM); i++) {
			if (pjadard_ts_data->rawdata_little_endian) {
				raw_data = (((int8_t)GET_RAWDATA[index + 1] << 8) | GET_RAWDATA[index]);
			} else {
				raw_data = (((int8_t)GET_RAWDATA[index] << 8) | GET_RAWDATA[index + 1]);
			}

			if ((raw_data < jd_g_sorting_threshold[0][i]) || (raw_data > jd_g_sorting_threshold[1][i])) {
				JD_E("%s: open check FAIL\n", __func__);
				ret_val = JD_SORTING_OPEN_CHECK_ERR;
				goto FAIL_END;
			}

			index += 2;
		}
		JD_I("%s: open check PASS\n", __func__);
		break;

	case JD_SORTING_SHORT_CHECK:
		for (i = 0; i < (pjadard_ic_data->JD_Y_NUM * pjadard_ic_data->JD_X_NUM); i++) {
			if (pjadard_ts_data->rawdata_little_endian) {
				raw_data = (((int8_t)GET_RAWDATA[index + 1] << 8) | GET_RAWDATA[index]);
			} else {
				raw_data = (((int8_t)GET_RAWDATA[index] << 8) | GET_RAWDATA[index + 1]);
			}

			if ((raw_data < jd_g_sorting_threshold[2][i]) || (raw_data > jd_g_sorting_threshold[3][i])) {
				JD_E("%s: short check FAIL\n", __func__);
				ret_val = JD_SORTING_SHORT_CHECK_ERR;
				goto FAIL_END;
			}

			index += 2;
		}
		JD_I("%s: short check PASS\n", __func__);
		break;

	case JD_SORTING_NORMALACTIVE_SB_DEV:
		for (i = 0; i < (pjadard_ic_data->JD_Y_NUM * pjadard_ic_data->JD_X_NUM); i++) {
			if (pjadard_ts_data->rawdata_little_endian) {
				raw_data = (((int8_t)GET_RAWDATA[index + 1] << 8) | GET_RAWDATA[index]);
			} else {
				raw_data = (((int8_t)GET_RAWDATA[index] << 8) | GET_RAWDATA[index + 1]);
			}

			if ((raw_data < jd_g_sorting_threshold[4][i]) || (raw_data > jd_g_sorting_threshold[5][i])) {
				JD_E("%s: Normal Active SB_DEV check FAIL\n", __func__);
				ret_val = JD_SORTING_NORMALACTIVE_SB_DEV_ERR;
				goto FAIL_END;
			}

			index += 2;
		}
		JD_I("%s: Normal Active SB_DEV check PASS\n", __func__);
		break;

	case JD_SORTING_NORMALACTIVE_NOISE:
		for (i = 0; i < (pjadard_ic_data->JD_Y_NUM * pjadard_ic_data->JD_X_NUM); i++) {
			if (pjadard_ts_data->rawdata_little_endian) {
				raw_data = (((int8_t)GET_RAWDATA[index + 1] << 8) | GET_RAWDATA[index]);
			} else {
				raw_data = (((int8_t)GET_RAWDATA[index] << 8) | GET_RAWDATA[index + 1]);
			}

			if ((raw_data < jd_g_sorting_threshold[6][i]) || (raw_data > jd_g_sorting_threshold[7][i])) {
				JD_E("%s: Normal Active noise check FAIL\n", __func__);
				ret_val = JD_SORTING_NORMALACTIVE_NOISE_ERR;
				goto FAIL_END;
			}

			index += 2;
		}
		JD_I("%s: Normal Active noise check PASS\n", __func__);
		break;

	case JD_SORTING_NORMALIDLE_RAWDATA_CHECK:
		for (i = 0; i < (pjadard_ic_data->JD_Y_NUM * pjadard_ic_data->JD_X_NUM); i++) {
			if (pjadard_ts_data->rawdata_little_endian) {
				raw_data = (((int8_t)GET_RAWDATA[index + 1] << 8) | GET_RAWDATA[index]);
			} else {
				raw_data = (((int8_t)GET_RAWDATA[index] << 8) | GET_RAWDATA[index + 1]);
			}

			if ((raw_data < jd_g_sorting_threshold[8][i]) || (raw_data > jd_g_sorting_threshold[9][i])) {
				JD_E("%s: Normal Idle Rawdata check FAIL\n", __func__);
				ret_val = JD_SORTING_NORMALIDLE_RAWDATA_CHECK_ERR;
				goto FAIL_END;
			}

			index += 2;
		}
		JD_I("%s: Normal Idle Rawdata check PASS\n", __func__);
		break;

	case JD_SORTING_NORMALIDLE_NOISE:
		for (i = 0; i < (pjadard_ic_data->JD_Y_NUM * pjadard_ic_data->JD_X_NUM); i++) {
			if (pjadard_ts_data->rawdata_little_endian) {
				raw_data = (((int8_t)GET_RAWDATA[index + 1] << 8) | GET_RAWDATA[index]);
			} else {
				raw_data = (((int8_t)GET_RAWDATA[index] << 8) | GET_RAWDATA[index + 1]);
			}

			if ((raw_data < jd_g_sorting_threshold[10][i]) || (raw_data > jd_g_sorting_threshold[11][i])) {
				JD_E("%s: Normal Idle noise check FAIL\n", __func__);
				ret_val = JD_SORTING_NORMALIDLE_NOISE_ERR;
				goto FAIL_END;
			}

			index += 2;
		}
		JD_I("%s: Normal Idle noise check PASS\n", __func__);
		break;

	case JD_SORTING_LPWUGACTIVE_SB_DEV:
		for (i = 0; i < (pjadard_ic_data->JD_Y_NUM * pjadard_ic_data->JD_X_NUM); i++) {
			if (pjadard_ts_data->rawdata_little_endian) {
				raw_data = (((int8_t)GET_RAWDATA[index + 1] << 8) | GET_RAWDATA[index]);
			} else {
				raw_data = (((int8_t)GET_RAWDATA[index] << 8) | GET_RAWDATA[index + 1]);
			}

			if ((raw_data < jd_g_sorting_threshold[12][i]) || (raw_data > jd_g_sorting_threshold[13][i])) {
				JD_E("%s: LPWUG Active SB_DEV check FAIL\n", __func__);
				ret_val = JD_SORTING_LPWUGACTIVE_SB_DEV_ERR;
				goto FAIL_END;
			}

			index += 2;
		}
		JD_I("%s: LPWUG Active SB_DEV check PASS\n", __func__);
		break;

	case JD_SORTING_LPWUGACTIVE_NOISE:
		for (i = 0; i < (pjadard_ic_data->JD_Y_NUM * pjadard_ic_data->JD_X_NUM); i++) {
			if (pjadard_ts_data->rawdata_little_endian) {
				raw_data = (((int8_t)GET_RAWDATA[index + 1] << 8) | GET_RAWDATA[index]);
			} else {
				raw_data = (((int8_t)GET_RAWDATA[index] << 8) | GET_RAWDATA[index + 1]);
			}

			if ((raw_data < jd_g_sorting_threshold[14][i]) || (raw_data > jd_g_sorting_threshold[15][i])) {
				JD_E("%s: LPWUG Active noise check FAIL\n", __func__);
				ret_val = JD_SORTING_LPWUGACTIVE_NOISE_ERR;
				goto FAIL_END;
			}

			index += 2;
		}
		JD_I("%s: LPWUG Active noise check PASS\n", __func__);
		break;

	case JD_SORTING_LPWUGIDLE_RAWDATA_CHECK:
		for (i = 0; i < (pjadard_ic_data->JD_Y_NUM * pjadard_ic_data->JD_X_NUM); i++) {
			if (pjadard_ts_data->rawdata_little_endian) {
				raw_data = (((int8_t)GET_RAWDATA[index + 1] << 8) | GET_RAWDATA[index]);
			} else {
				raw_data = (((int8_t)GET_RAWDATA[index] << 8) | GET_RAWDATA[index + 1]);
			}

			if ((raw_data < jd_g_sorting_threshold[16][i]) || (raw_data > jd_g_sorting_threshold[17][i])) {
				JD_E("%s: LPWUG Idle Rawdata check FAIL\n", __func__);
				ret_val = JD_SORTING_LPWUGIDLE_RAWDATA_CHECK_ERR;
				goto FAIL_END;
			}

			index += 2;
		}
		JD_I("%s: LPWUG Idle Rawdata check PASS\n", __func__);
		break;

	case JD_SORTING_LPWUGIDLE_NOISE:
		for (i = 0; i < (pjadard_ic_data->JD_Y_NUM * pjadard_ic_data->JD_X_NUM); i++) {
			if (pjadard_ts_data->rawdata_little_endian) {
				raw_data = (((int8_t)GET_RAWDATA[index + 1] << 8) | GET_RAWDATA[index]);
			} else {
				raw_data = (((int8_t)GET_RAWDATA[index] << 8) | GET_RAWDATA[index + 1]);
			}

			if ((raw_data < jd_g_sorting_threshold[18][i]) || (raw_data > jd_g_sorting_threshold[19][i])) {
				JD_E("%s: LPWUG Idle noise check FAIL\n", __func__);
				ret_val = JD_SORTING_LPWUGIDLE_NOISE_ERR;
				goto FAIL_END;
			}

			index += 2;
		}
		JD_I("%s: LPWUG Idle noise check PASS\n", __func__);
		break;

	default:
		JD_E("Wrong type=%d\n", checktype);
		break;
	}

	sprintf(rslt_log, "\n%s%s\n", jd_action_item_name[checktype - JD_SORTING_OPEN_CHECK], " Test Pass!\n");
	JD_I("%s Test Pass!\n", jd_action_item_name[checktype - JD_SORTING_OPEN_CHECK]);
	goto END_FUNC;

FAIL_END:
	sprintf(rslt_log, "\n%s%s\n", jd_action_item_name[checktype - JD_SORTING_OPEN_CHECK], " Test Fail!\n");
	JD_I("%s Test Fail!\n", jd_action_item_name[checktype - JD_SORTING_OPEN_CHECK]);

END_FUNC:
	jd_test_data_get(GET_RAWDATA, start_log, rslt_log, checktype - JD_SORTING_OPEN_CHECK);

	if (start_log) {
		kfree(start_log);
		start_log = NULL;
	}

	if (rslt_log) {
		kfree(rslt_log);
		rslt_log = NULL;
	}

	return ret_val;
}

static uint32_t jadard_TestItem_Select(uint8_t sorting_item)
{
	Fail_flag = false;

	switch (sorting_item)
	{
		case JD_SORTING_SLEEP_IN:
			jadard_pst_enter_slpin_mode();
			mdelay(300);
			break;

		case JD_SORTING_SLEEP_OUT:
			jadard_pst_enter_slpout_mode();
			mdelay(300);
			break;

		case JD_SORTING_OPEN_CHECK:
			jadard_PST_Open_Mode();
			if (Fail_flag == true)
				return JD_SORTING_OPEN_CHECK_ERR;
			jadard_PST_Enter_RawData_Type_Mode();
			if (Fail_flag == true)
				return JD_SORTING_OPEN_CHECK_ERR;
			jadard_Start_to_Get_RawData_Function();
			if (Fail_flag == true)
				return JD_SORTING_OPEN_CHECK_ERR;
			if (jadard_Item_Test_Function(JD_SORTING_OPEN_CHECK) != JD_SORTING_OK)
				return JD_SORTING_OPEN_CHECK_ERR;
			break;

		case JD_SORTING_SHORT_CHECK:
			jadard_PST_Short_Mode();
			if (Fail_flag == true)
				return JD_SORTING_SHORT_CHECK_ERR;
			jadard_PST_Enter_RawData_Type_Mode();
			if (Fail_flag == true)
				return JD_SORTING_SHORT_CHECK_ERR;
			jadard_Start_to_Get_RawData_Function();
			if (Fail_flag == true)
				return JD_SORTING_SHORT_CHECK_ERR;
			if (jadard_Item_Test_Function(JD_SORTING_SHORT_CHECK) != JD_SORTING_OK)
				return JD_SORTING_SHORT_CHECK_ERR;
			break;

		case JD_SORTING_NORMALACTIVE_SB_DEV:
			jadard_PST_NormalActive_Mode();
			if (Fail_flag == true)
				return JD_SORTING_NORMALACTIVE_SB_DEV_ERR;
			jadard_PST_Enter_RawData_Type_Mode();
			if (Fail_flag == true)
				return JD_SORTING_NORMALACTIVE_SB_DEV_ERR;
			jadard_Start_to_Get_RawData_Function();
			if (Fail_flag == true)
				return JD_SORTING_NORMALACTIVE_SB_DEV_ERR;
			if (jadard_Item_Test_Function(JD_SORTING_NORMALACTIVE_SB_DEV) != JD_SORTING_OK)
				return JD_SORTING_NORMALACTIVE_SB_DEV_ERR;
			break;

		case JD_SORTING_NORMALACTIVE_NOISE:
			jadard_PST_NormalActive_Diff_Mode();
			if (Fail_flag == true)
				return JD_SORTING_NORMALACTIVE_NOISE_ERR;
			jadard_PST_Enter_Diff_Type_Mode();
			if (Fail_flag == true)
				return JD_SORTING_NORMALACTIVE_NOISE_ERR;
			jadard_Start_to_Get_DiffData_Function();
			if (Fail_flag == true)
				return JD_SORTING_NORMALACTIVE_NOISE_ERR;
			if (jadard_Item_Test_Function(JD_SORTING_NORMALACTIVE_NOISE) != JD_SORTING_OK)
				return JD_SORTING_NORMALACTIVE_NOISE_ERR;
			break;

		case JD_SORTING_NORMALIDLE_RAWDATA_CHECK:
			jadard_PST_NormalIdle_Mode();
			if (Fail_flag == true)
				return JD_SORTING_NORMALIDLE_RAWDATA_CHECK_ERR;
			jadard_PST_Enter_RawData_Type_Mode();
			if (Fail_flag == true)
				return JD_SORTING_NORMALIDLE_RAWDATA_CHECK_ERR;
			jadard_Start_to_Get_RawData_Function();
			if (Fail_flag == true)
				return JD_SORTING_NORMALIDLE_RAWDATA_CHECK_ERR;
			if (jadard_Item_Test_Function(JD_SORTING_NORMALIDLE_RAWDATA_CHECK) != JD_SORTING_OK)
				return JD_SORTING_NORMALIDLE_RAWDATA_CHECK_ERR;
			break;

		case JD_SORTING_NORMALIDLE_NOISE:
			jadard_PST_NormalIdle_Diff_Mode();
			if (Fail_flag == true)
				return JD_SORTING_NORMALIDLE_NOISE_ERR;
			jadard_PST_Enter_Diff_Type_Mode();
			if (Fail_flag == true)
				return JD_SORTING_NORMALIDLE_NOISE_ERR;
			jadard_Start_to_Get_DiffData_Function();
			if (Fail_flag == true)
				return JD_SORTING_NORMALIDLE_NOISE_ERR;
			if (jadard_Item_Test_Function(JD_SORTING_NORMALIDLE_NOISE) != JD_SORTING_OK)
				return JD_SORTING_NORMALIDLE_NOISE_ERR;
			break;

		case JD_SORTING_LPWUGACTIVE_SB_DEV:
			jadard_PST_LpwugActive_Mode();
			if (Fail_flag == true)
				return JD_SORTING_LPWUGACTIVE_SB_DEV_ERR;
			jadard_PST_Enter_RawData_Type_Mode();
			if (Fail_flag == true)
				return JD_SORTING_LPWUGACTIVE_SB_DEV_ERR;
			jadard_Start_to_Get_RawData_Function();
			if (Fail_flag == true)
				return JD_SORTING_LPWUGACTIVE_SB_DEV_ERR;
			if (jadard_Item_Test_Function(JD_SORTING_LPWUGACTIVE_SB_DEV) != JD_SORTING_OK)
				return JD_SORTING_LPWUGACTIVE_SB_DEV_ERR;
			break;

		case JD_SORTING_LPWUGACTIVE_NOISE:
			jadard_PST_LpwugActive_Diff_Mode();
			if (Fail_flag == true)
				return JD_SORTING_LPWUGACTIVE_NOISE_ERR;
			jadard_PST_Enter_Diff_Type_Mode();
			if (Fail_flag == true)
				return JD_SORTING_LPWUGACTIVE_NOISE_ERR;
			jadard_Start_to_Get_DiffData_Function();
			if (Fail_flag == true)
				return JD_SORTING_LPWUGACTIVE_NOISE_ERR;
			if (jadard_Item_Test_Function(JD_SORTING_LPWUGACTIVE_NOISE) != JD_SORTING_OK)
				return JD_SORTING_LPWUGACTIVE_NOISE_ERR;
			break;

		case JD_SORTING_LPWUGIDLE_RAWDATA_CHECK:
			jadard_PST_LpwugIdle_Mode();
			if (Fail_flag == true)
				return JD_SORTING_LPWUGIDLE_RAWDATA_CHECK_ERR;
			jadard_PST_Enter_RawData_Type_Mode();
			if (Fail_flag == true)
				return JD_SORTING_LPWUGIDLE_RAWDATA_CHECK_ERR;
			jadard_Start_to_Get_RawData_Function();
			if (Fail_flag == true)
				return JD_SORTING_LPWUGIDLE_RAWDATA_CHECK_ERR;
			if (jadard_Item_Test_Function(JD_SORTING_LPWUGIDLE_RAWDATA_CHECK) != JD_SORTING_OK)
				return JD_SORTING_LPWUGIDLE_RAWDATA_CHECK_ERR;
			break;

		case JD_SORTING_LPWUGIDLE_NOISE:
			jadard_PST_LpwugIdle_Diff_Mode();
			if (Fail_flag == true)
				return JD_SORTING_LPWUGIDLE_NOISE_ERR;
			jadard_PST_Enter_Diff_Type_Mode();
			if (Fail_flag == true)
				return JD_SORTING_LPWUGIDLE_NOISE_ERR;
			jadard_Start_to_Get_DiffData_Function();
			if (Fail_flag == true)
				return JD_SORTING_LPWUGIDLE_NOISE_ERR;
			if (jadard_Item_Test_Function(JD_SORTING_LPWUGIDLE_NOISE) != JD_SORTING_OK)
				return JD_SORTING_LPWUGIDLE_NOISE_ERR;
			break;

		default:
			break;
	}

	return JD_SORTING_OK;
}

static int jadard_power_cal(int pow, int number)
{
	int i = 0;
	int result = 1;

	for (i = 0; i < pow; i++)
		result *= 10;
	result = result * number;

	return result;
}

static int jadard_str2int(char *str)
{
	int i = 0;
	int temp_cal = 0;
	int result = 0;
	int str_len = strlen(str);
	int negtive_flag = 0;
	for (i = 0; i < strlen(str); i++) {
		if (str[i] == '-') {
			negtive_flag = 1;
			str_len--;
			continue;
		}
		temp_cal = str[i] - '0';
		result += jadard_power_cal(str_len - i - 1, temp_cal);
	}

	if (negtive_flag == 1) {
		result = 0 - result;
	}

	return result;
}

static int jadard_string_search(char *str1, char *str2)
{
	const char* cur;
	const char* last;
	int str1_len = strlen(str1);
	int str2_len = strlen(str2) - 1;
	
	last =  str2 + str2_len - str1_len;

	for (cur = str2; cur <= last; ++cur) {
		if (cur[0] == str1[0] && memcmp(cur, str1, str1_len) == 0) {
			JD_I("%s: match item %s\n", __func__, str1);
			return 0;
		}
	}

	return -1;
}

static int jadard_set_global_variable(char *str, int *val_parsed)
{
	int i, j, item_num, var_val;
	char value[5];
	bool item_found = false;
	
	for (item_num = 0; item_num < JD_GLOBAL_VARIABLE_END; item_num++) {
		if (val_parsed[item_num])
			continue;
		
		if (jadard_string_search(jd_global_variable_name[item_num], str) == 0) {
			i = 0;
			j = 0;
			while (str[i] != ASCII_LF) {
				if (str[i] == ACSII_COLON) {
					item_found = true;
					memset(value, 0x00, sizeof(value));
				} else if (item_found) {
					/* Get variable value */
					value[j++] = str[i];
				}
				i++;
			}
			if (item_found) {
				var_val = jadard_str2int(value);
				val_parsed[item_num] = 1;
				switch (item_num)
				{
					case JD_GLOBAL_GETRAWDATA_FRAME_NUMBER:
						JD_GetRawData_Frame_Number = var_val;
						break;
					case JD_GLOBAL_GETDIFFDATA_FRAME_NUMBER:
						JD_GetDiffData_Frame_Number = var_val;
						break;
					case JD_GLOBAL_SKIPRAWDATA_FRAME_NUMBER:
						JD_SkipRawData_Frame_Number = var_val;
						break;
					case JD_GLOBAL_SKIPDIFFDATA_FRAME_NUMBER:
						JD_SkipDiffData_Frame_Number = var_val;
						break;
					default:
						break;
				}
				JD_I("%s: Set variable %s: %d\n", __func__, jd_global_variable_name[item_num], var_val);
			}
			break;
		}
	}
	
	return 0;
}

static int jadard_set_sorting_threshold(char **result, int data_lines)
{
	int i, j, k, item_num, *th_parsed;
	int count_type = -1;
	int count_data = 0;
	char data[5];
	bool item_found = false;
	int count_size = pjadard_ic_data->JD_X_NUM * pjadard_ic_data->JD_Y_NUM;
	
	th_parsed = kzalloc(sizeof(int) * JD_SORTING_ACTIVE_ITEM, GFP_KERNEL);

	for (i = 0; i < data_lines; i++) {
		if (!item_found) {
			/* Sorting action name */
			for (item_num = 0; item_num < JD_SORTING_ACTIVE_ITEM; item_num++) {
				if (th_parsed[item_num])
					continue;
				
				if (jadard_string_search(jd_action_item_name[item_num], result[i]) == 0) {
					count_type = item_num;
					count_data = 0;
					item_found = true;
					th_parsed[item_num] = 1;
					break;
				}
			}
		} else {
			/* Sorting threshold */
			j = 0;
			k = 0;
			memset(data, 0x00, sizeof(data));

			while (result[i][j] != ASCII_LF) {
				if (result[i][j] == ACSII_COLON) {
					/* Sorting min threshold */
					jd_g_sorting_threshold[count_type * 2][count_data] = jadard_str2int(data);
					k = 0;
					memset(data, 0x00, sizeof(data));
				} else if (result[i][j] == ASCII_COMMA) {
					/* Sorting max threshold */
					jd_g_sorting_threshold[count_type * 2 + 1][count_data] = jadard_str2int(data);
					k = 0;
					count_data++;
					memset(data, 0x00, sizeof(data));
				} else {
					/* Get threshold data */
					data[k++] = result[i][j];
				}
				j++;
			}
			
			if (count_data == count_size)
				item_found = false;
		}
	}
	
	for (item_num = 0; item_num < JD_SORTING_ACTIVE_ITEM; item_num++) {
		if (!th_parsed[item_num])
			JD_E("Sorting threshold not found for item %s\n", jd_action_item_name[item_num]);
	}
	kfree(th_parsed);

	return JD_SORTING_OK;
}

static int jadard_remove_space_cr(const struct firmware *file_entry, char **result, int data_lines, int line_max_len)
{
	int count = 0;
	int str_count = 0;
	int char_count = 0;
	int item_num, *val_parsed;
	bool threshold_found = false;
	bool variable_found = false;
	char *temp_str = NULL;
	char *check_str = "Threshold";

	val_parsed = kzalloc(sizeof(int) * (JD_GLOBAL_VARIABLE_END - 1), GFP_KERNEL);
	do {
		if (!temp_str)
			temp_str = kzalloc(line_max_len * sizeof(char), GFP_KERNEL);
		
		switch (file_entry->data[count]) {
		case ASCII_CR:
		case ACSII_SPACE:
			count++;
			break;
		case ASCII_LF:
			if (char_count != 0) {
				if (threshold_found) {
					result[str_count][char_count] = ASCII_LF;
					str_count++;
				} else {
					temp_str[char_count] = ASCII_LF;
					if (jadard_string_search(check_str, temp_str) == 0) {
						variable_found = true;
						for (item_num = 0; item_num < JD_GLOBAL_VARIABLE_END; item_num++) {
							if (!val_parsed[item_num]) {
								variable_found = false;
								JD_E("Gloable variable not found for item %s\n", jd_global_variable_name[item_num]);
							}
						}
						kfree(val_parsed);
						val_parsed = NULL;
						if (variable_found)
							threshold_found = true;
						else
							return -1;
					}
					else if (val_parsed)
						jadard_set_global_variable(temp_str, val_parsed);
					kfree(temp_str);
					temp_str = NULL;
				}
				char_count = 0;
			}
			count++;
			break;
		default:
			if (threshold_found) 
				result[str_count][char_count++] = file_entry->data[count];
			else
				temp_str[char_count++] = file_entry->data[count];
			count++;
			break;
		}
	} while ((count < file_entry->size) && (str_count < data_lines));

	if (threshold_found) 
		return 0;
	else
		return -1;
}

static int jadard_parse_sorting_threshold_file(void)
{
	int err = JD_SORTING_OK, i;
	const struct firmware *file_entry = NULL;
	char *file_name = "jd_sorting_threshold.txt";
	char **result = NULL;
	int data_lines = (pjadard_ic_data->JD_X_NUM + 1) * JD_SORTING_ACTIVE_ITEM;
	int line_max_len = pjadard_ic_data->JD_Y_NUM * 12 + 4;

	JD_D("%s,Entering \n", __func__);
	JD_I("sorting threshold file name = %s\n", file_name);

	err = request_firmware(&file_entry, file_name, pjadard_ts_data->dev);
	if (err < 0) {
		JD_E("%s: Open file fail(err:%d)\n", __func__, err);
		err = JD_FILE_OPEN_FAIL;
		goto END_FILE_OPEN_FAIL;
	}

	result = kzalloc(data_lines * sizeof(char *), GFP_KERNEL);
	for (i = 0 ; i < data_lines; i++)
		result[i] = kzalloc(line_max_len * sizeof(char), GFP_KERNEL);

	JD_I("data_lines=%d ,line_max_len=%d\n", data_lines, line_max_len);
	JD_I("file_size=%d\n", (int)file_entry->size);

	err = jadard_remove_space_cr(file_entry, result, data_lines, line_max_len);
	if (err != JD_SORTING_OK) {
		JD_E("%s: Find threshold section/global variables from file fail, go end!\n", __func__);
		goto END_FUNC;
	}

	err = jadard_set_sorting_threshold(result, data_lines);
	if (err != JD_SORTING_OK) {
		JD_E("%s: Load criteria from file fail, go end!\n", __func__);
		goto END_FUNC;
	}

END_FUNC:
	for (i = 0 ; i < data_lines; i++) {
		kfree(result[i]);
	}
	kfree(result);
	release_firmware(file_entry);

END_FILE_OPEN_FAIL:
	JD_I("%s,end \n", __func__);

	return err;
}

static char *get_date_time_str(void)
{
	struct timespec now_time;
	struct rtc_time rtc_now_time;
	static char time_data_buf[128] = { 0 };

	getnstimeofday(&now_time);
	rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
	snprintf(time_data_buf, sizeof(time_data_buf), "%04d%02d%02d-%02d%02d%02d",
		(rtc_now_time.tm_year + 1900), rtc_now_time.tm_mon + 1,
		rtc_now_time.tm_mday, rtc_now_time.tm_hour, rtc_now_time.tm_min,
		rtc_now_time.tm_sec);

	return time_data_buf;
}
static int jadard_read_sorting_threshold(void)
{
	int ret = JD_SORTING_OK;
	int i;
#if JD_SORTING_THRESHOLD_DBG
	int j;
#endif

	JD_SORTING_ACTIVE_ITEM = (JD_SORTING_ITEM_END - 1) - JD_SORTING_SLEEP_OUT;
	JD_READ_THRESHOLD_SIZE = (JD_SORTING_ACTIVE_ITEM)*2;

	JD_I("JD_SORTING_ACTIVE_ITEM(%d) JD_READ_THRESHOLD_SIZE(%d)\n", JD_SORTING_ACTIVE_ITEM, JD_READ_THRESHOLD_SIZE);

	jd_g_sorting_threshold = kzalloc(sizeof(int *)*JD_READ_THRESHOLD_SIZE, GFP_KERNEL);
	for (i = 0; i < JD_READ_THRESHOLD_SIZE; i++) {
		jd_g_sorting_threshold[i] = kzalloc(sizeof(int)*(pjadard_ic_data->JD_X_NUM * pjadard_ic_data->JD_Y_NUM), GFP_KERNEL);
	}
	ret = jadard_parse_sorting_threshold_file();

	jd_g_file_path = kzalloc(64 * sizeof(char), GFP_KERNEL);
	jd_g_rslt_data = kzalloc(JD_OUTPUT_BUFFER_SIZE * JD_SORTING_ACTIVE_ITEM * sizeof(char), GFP_KERNEL);
	GET_RAWDATA = kzalloc(pjadard_ic_data->JD_X_NUM * pjadard_ic_data->JD_Y_NUM * sizeof(uint16_t), GFP_KERNEL);
			sprintf(jd_g_file_path, "%s%s%s.txt", JD_RSLT_OUT_PATH, JD_RSLT_OUT_FILE, get_date_time_str());

#if JD_SORTING_THRESHOLD_DBG
	for (i = 0; i < JD_READ_THRESHOLD_SIZE; i = i + 2) {
		JD_I("[%d]", i);
		for (j = 0; j < pjadard_ic_data->JD_X_NUM * pjadard_ic_data->JD_Y_NUM; j++) {
			if(j % pjadard_ic_data->JD_Y_NUM == 0) {
				JD_I("\n");
			} else {
				JD_I("%d:%d ", jd_g_sorting_threshold[i][j], jd_g_sorting_threshold[i + 1][j]);
			}
		}
	}
#endif

	return ret;
}

static int jadard_sorting_test(void)
{
	uint32_t ret = JD_SORTING_OK;

	JD_I("%s: enter \n", __func__);

	ret = jadard_read_sorting_threshold();
	if (ret != JD_SORTING_OK) {
		JD_E("jadard_read_sorting_threshold fail!\n");
		goto END_SORTING_TEST;
	}

#if JD_SORTING_LPWUG_CHECK
	/* 0.Sleep in/out */
	JD_I("[JD_SORTING_SLEEP_IN]\n");
	ret += jadard_TestItem_Select(JD_SORTING_SLEEP_IN);
	JD_I("JD_SORTING_SLEEP_IN: End %d\n\n", ret);

	JD_I("[JD_SORTING_SLEEP_OUT]\n");
	ret += jadard_TestItem_Select(JD_SORTING_SLEEP_OUT);
	JD_I("JD_SORTING_SLEEP_OUT: End %d\n\n", ret);
#endif

	/* 1.Open Test */
	JD_I("[JD_SORTING_OPEN_CHECK]\n");
	ret += jadard_TestItem_Select(JD_SORTING_OPEN_CHECK);
	JD_I("JD_SORTING_OPEN_CHECK: End %d\n\n", ret);

	/* 2. Short Test */
	JD_I("[JD_SORTING_SHORT_CHECK]\n");
	ret += jadard_TestItem_Select(JD_SORTING_SHORT_CHECK);
	JD_I("JD_SORTING_SHORT_CHECK: End %d\n\n", ret);

	/* 3. NORMAL ACTIVE SB_DEV Test */
	JD_I("[JD_SORTING_NORMALACTIVE_SB_DEV]\n");
	ret += jadard_TestItem_Select(JD_SORTING_NORMALACTIVE_SB_DEV);
	JD_I("JD_SORTING_NORMALACTIVE_SB_DEV: End %d\n\n", ret);

	/* 4. NORMAL ACTIVE NOISE Test */
	JD_I("[JD_SORTING_NORMALACTIVE_NOISE]\n");
	ret += jadard_TestItem_Select(JD_SORTING_NORMALACTIVE_NOISE);
	JD_I("JD_SORTING_NORMALACTIVE_NOISE: End %d\n\n", ret);

	/* 5. NORMAL IDLE RAWDATA CHECK Test */
	JD_I("[JD_SORTING_NORMALIDLE_RAWDATA_CHECK]\n");
	ret += jadard_TestItem_Select(JD_SORTING_NORMALIDLE_RAWDATA_CHECK);
	JD_I("JD_SORTING_NORMALIDLE_RAWDATA_CHECK: End %d\n\n", ret);

	/* 6. NORMAL IDLE NOISE Test */
	JD_I("[JD_SORTING_NORMALIDLE_NOISE]\n");
	ret += jadard_TestItem_Select(JD_SORTING_NORMALIDLE_NOISE);
	JD_I("JD_SORTING_NORMALIDLE_NOISE: End %d\n\n", ret);

#if JD_SORTING_LPWUG_CHECK
	/* 7. LPWUG ACTIVE SB_DEV Test */
	JD_I("[JD_SORTING_LPWUGACTIVE_SB_DEV]\n");
	ret += jadard_TestItem_Select(JD_SORTING_LPWUGACTIVE_SB_DEV);
	JD_I("JD_SORTING_LPWUGACTIVE_SB_DEV: End %d\n\n", ret);

	/* 8. LPWUG ACTIVE NOISE Test */
	JD_I("[JD_SORTING_LPWUGACTIVE_NOISE]\n");
	ret += jadard_TestItem_Select(JD_SORTING_LPWUGACTIVE_NOISE);
	JD_I("JD_SORTING_LPWUGACTIVE_NOISE: End %d\n\n", ret);

	/* 9. LPWUG IDLE RAWDATA CHECK Test */
	JD_I("[JD_SORTING_LPWUGIDLE_RAWDATA_CHECK]\n");
	ret += jadard_TestItem_Select(JD_SORTING_LPWUGIDLE_RAWDATA_CHECK);
	JD_I("JD_SORTING_LPWUGIDLE_RAWDATA_CHECK: End %d\n\n", ret);

	/* 10. LPWUG IDLE NOISE Test */
	JD_I("[JD_SORTING_LPWUGIDLE_NOISE]\n");
	ret += jadard_TestItem_Select(JD_SORTING_LPWUGIDLE_NOISE);
	JD_I("JD_SORTING_LPWUGIDLE_NOISE: End %d\n\n", ret);
#endif

	jd_test_data_pop_out(jd_g_rslt_data, jd_g_file_path);
	/* Enter normal mode */
	jadard_pst_enter_normal_mode();
	mdelay(1000);
	g_module_fp.fp_soft_reset();
	g_module_fp.fp_StartMCU();

END_SORTING_TEST:
	jadard_sorting_data_deinit();

	JD_I("Sorting test result = %d \n", ret);

	if (ret != 0)
		ret = 1;

	JD_I("%s:OUT\n", __func__);
	return ret;
}

void jadard_sorting_init(void)
{
	JD_I("%s: enter \n", __func__);
	g_module_fp.fp_sorting_test = jadard_sorting_test;
}
