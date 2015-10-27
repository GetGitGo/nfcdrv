#ifndef DBG_OUT_H 
#define DBG_OUT_H 

//#define dbgout(format, args...)

#ifndef dbgout
#if 0
#define dbgout(format, args...) \
	{ \
  		fprintf(stdout,"[%s] "format"\n",__FUNCTION__,##args); \
		fflush(stdout); \
	}
#endif
#if 1
#define dbgout(format, args...) \
	{ \
		struct timeval tv; \
		gettimeofday(&tv, NULL); \
  		fprintf(stdout,"[%d.%06d][%s +%d %s] "format"\n",(int)tv.tv_sec,(int)tv.tv_usec, \
			__FILE__,__LINE__,__FUNCTION__,##args); \
		fflush(stdout); \
	}
#endif
#endif

#ifndef errout
#define errout(format, args...) \
	{ \
		struct timeval tv; \
		gettimeofday(&tv, NULL); \
  		fprintf(stderr,"[%d.%06d][%s +%d %s] "format"\n",(int)tv.tv_sec,(int)tv.tv_usec, \
			__FILE__,__LINE__,__FUNCTION__,##args); \
		fflush(stderr); \
	}
#endif

void hexout( char * data, int len );

#endif
