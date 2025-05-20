#ifdef  REVO_SUSPEND_INTERFACES


#define REVO_GET_TS_DATA   s_revo_set_ts_data(NULL)
static struct fts_ts_data *s_revo_set_ts_data(void *tsData)
{
    static struct fts_ts_data * cts_data = NULL ; 
	
	cts_data = (tsData != NULL) ? tsData : NULL ;

	if( cts_data == NULL )
		REVO_E_LOG("Error NOT set fts_ts_data , pls set fts_ts_data after probe success !!!! ");
	
	return cts_data ; 
}



/*
(function name) = (Global) + (touchscreen dir ) + ( resume | suspend )

ex:  (glb)+(focaltech/ft8756)+(resume|suspend)   = 
       + glb_focaltech_ft8756_resume  
       ` glb_focaltech_ft8756_suspend 
*/

int glb_focaltech_ft8756_resume(void)
{
	 struct fts_ts_data * cts_data = REVO_GET_TS_DATA ;
	 if(cts_data == NULL )
	 {
		 REVO_E_LOG("Error NOT set fts_ts_data  !!!! ");
		 return  -1 ; 
	 }
	 return fts_ts_resume(cts_data->dev);	 
}

int glb_focaltech_ft8756_suspend(void)
{
	 struct fts_ts_data * cts_data = REVO_GET_TS_DATA ;
	 if(cts_data == NULL )
	 {
		 REVO_E_LOG("Error NOT set fts_ts_data  !!!! ");
		 return -1 ; 
	 }
	 return fts_ts_suspend(cts_data->dev);	 
}

#endif