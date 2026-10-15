/* Diagnostic single-plane MJPEG MMAP metadata probe. Never opens motors. */
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

static int call(int fd,unsigned long request,void *arg) {
    int result;do {result=ioctl(fd,request,arg);} while(result<0 && errno==EINTR);return result;
}
static double mono(void) {struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec/1e9;}
static void pause_ms(long ms) {struct timespec t={ms/1000,(ms%1000)*1000000};while(nanosleep(&t,&t)<0 && errno==EINTR){} }
int main(int argc,char **argv) {
    if(argc!=4){fprintf(stderr,"usage: probe DEVICE EXPECTED_WIDTH EXPECTED_HEIGHT\n");return 2;}
    int fd=-1,streaming=0,status=1;unsigned int count=0;
    void **maps=NULL;size_t *lengths=NULL;
    fd=open(argv[1],O_RDWR|O_NONBLOCK);if(fd<0){perror("open camera");goto done;}
    struct v4l2_format fmt={0};fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(call(fd,VIDIOC_G_FMT,&fmt)<0){perror("G_FMT");goto done;}
    struct v4l2_streamparm parm={0};parm.type=fmt.type;
    int has_parm=call(fd,VIDIOC_G_PARM,&parm)==0;
    printf("{\"kind\":\"format\",\"width\":%u,\"height\":%u,\"fourcc\":%u,\"fps_numerator\":%u,\"fps_denominator\":%u}\n",
        fmt.fmt.pix.width,fmt.fmt.pix.height,fmt.fmt.pix.pixelformat,
        has_parm?parm.parm.capture.timeperframe.denominator:0,has_parm?parm.parm.capture.timeperframe.numerator:0);fflush(stdout);
    if(fmt.fmt.pix.width!=(unsigned)atoi(argv[2]) || fmt.fmt.pix.height!=(unsigned)atoi(argv[3]) || fmt.fmt.pix.pixelformat!=V4L2_PIX_FMT_MJPEG) {
        fprintf(stderr,"Current format differs from expected MJPEG mode; no format/controls changed.\n");goto done;
    }
    struct v4l2_requestbuffers req={0};req.count=1;req.type=fmt.type;req.memory=V4L2_MEMORY_MMAP;
    if(call(fd,VIDIOC_REQBUFS,&req)<0 || req.count==0){perror("REQBUFS");goto done;}
    count=req.count;maps=calloc(count,sizeof(*maps));lengths=calloc(count,sizeof(*lengths));
    if(!maps || !lengths){perror("allocate");goto done;}
    for(unsigned i=0;i<count;i++) {
        struct v4l2_buffer b={0};b.index=i;b.type=fmt.type;b.memory=V4L2_MEMORY_MMAP;
        if(call(fd,VIDIOC_QUERYBUF,&b)<0){perror("QUERYBUF");goto done;}
        lengths[i]=b.length;maps[i]=mmap(NULL,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,b.m.offset);
        if(maps[i]==MAP_FAILED){perror("mmap");goto done;}
        if(call(fd,VIDIOC_QBUF,&b)<0){perror("QBUF");goto done;}
    }
    enum v4l2_buf_type type=fmt.type;
    if(call(fd,VIDIOC_STREAMON,&type)<0){perror("STREAMON");goto done;}streaming=1;
    printf("{\"kind\":\"buffers\",\"requested\":1,\"allocated\":%u}\n",count);fflush(stdout);
    for(int i=0;i<24;i++) {
        if(i==16)pause_ms(250); /* Requeued before pause: expose queued-frame age. */
        struct pollfd pollfd={fd,POLLIN,0};int ready;
        do {ready=poll(&pollfd,1,2000);}while(ready<0 && errno==EINTR);
        if(ready<=0){fprintf(stderr,"frame timeout/poll error\n");goto done;}
        struct v4l2_buffer b={0};b.type=type;b.memory=V4L2_MEMORY_MMAP;
        if(call(fd,VIDIOC_DQBUF,&b)<0){perror("DQBUF");goto done;}
        double host=mono(),stamp=b.timestamp.tv_sec+b.timestamp.tv_usec/1e6;
        if(b.index>=count || b.bytesused>lengths[b.index]){fprintf(stderr,"invalid dequeued buffer\n");goto done;}
        printf("{\"kind\":\"frame\",\"ordinal\":%d,\"warmup\":%s,\"after_pause\":%s,\"sequence\":%u,\"flags\":%u,\"bytesused\":%u,\"driver_timestamp_s\":%.9f,\"host_dequeue_monotonic_s\":%.9f}\n",
            i,i<8?"true":"false",i==16?"true":"false",b.sequence,b.flags,b.bytesused,stamp,host);fflush(stdout);
        if(call(fd,VIDIOC_QBUF,&b)<0){perror("requeue");goto done;}
    }
    status=0;
done:
    if(streaming){enum v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;call(fd,VIDIOC_STREAMOFF,&type);}
    if(maps && lengths)for(unsigned i=0;i<count;i++)if(maps[i] && maps[i]!=MAP_FAILED)munmap(maps[i],lengths[i]);
    free(maps);free(lengths);if(fd>=0)close(fd);return status;
}
