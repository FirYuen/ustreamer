/**
 * @file   yuv2rgb.h
 * @author Late Lee
 * @date   2012.01.20
 * 
 * @brief  
 *         YUV422P/YUV420P/YUV422SPתRGB24ʵ�ִ���
 *
 * @note
 *         1����Windows�������
 *         2��������ʹ�õ�YUV420P��YUV422P��ƽ��(planar)��ʽ�����Ǵ��(packed)��ʽ
 *
 *         3��qcif: 176*144
              cif: 352*288
 * @todo  ȷ����RGB���򵽵���ʲô
 *
 * @log   2013-10-26 �ο�422p������ʵ��422spת��422p��ʽ����������ʼ���ӿ����أ������⹫��
 *        2014-02-10 ��װͳһһ��������
 *
 * �ʼǣ�
            ÿ��Y��U��V��R��G��B��ռ��1���ֽ�
            YUV422Pƽ���ʽ
            Y��U��V�ֿ��洢��Yռw*h��U��V�ֱ�ռw*h/2��ÿ��Y��w����U��V��w/2����
            ����Y����һ��U��V������������Y����2����U��V������һ������4�ֽڣ����һ������ռ2�ֽڡ�

            �ڴ�ֲ����£�
                    w
            +--------------------+
            |Y0Y1Y2Y3...         |
            |...                 |   h
            |...                 |
            |                    |
            +--------------------+
            |U0U1      |
            |...       |   h
            |...       |
            |          |
            +----------+
            |V0V1      |
            |...       |  h
            |...       |
            |          |
            +----------+
                w/2

            ��һ�����أ�Y0 U0 V0
            �ڶ������أ�Y1 U0 V0
            ���������أ�Y2 U1 V1
            ���ĸ����أ�Y3 U1 V1

			ÿ�ָ�ʽ�ֲ���ת��������
 */

#ifndef _YUV2RGB_H_
#define _YUV2RGB_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  YUV422Pת��ΪRGB24(�����)
 * 
 * @param  yuvbuffer[IN]    YUV422Pƽ���ʽ������
 * @param  rgbbuffer[OUT]   RGB������
 * @param  width[IN]        ͼ���
 * @param  height[IN]       ͼ���
 * 
 * @return no
 *
 * @note
 *         1.YUV422P��ʽYUV��������СΪw * h * 2
 *         2.rgbbuffer��������ΪRGB���籣��BMP����Ҫ����ΪBGR
 */
void yuv422p_to_rgb24(unsigned char* yuvbuffer, unsigned char* rgbbuffer, int width, int height);
void convert_yuv_to_rgb_buffer(unsigned char *yuv, unsigned char *rgb, unsigned int width, unsigned int height);
void yuyv_to_rgb(unsigned char *yuyv, unsigned char *rgb,  int width,int height);

/** 
 * @brief YUV420PתRGB24(�����)
 * 
 * @param yuvbuffer  YUV420P��ʽ������
 * @param rgbbuffer  RGB24��ʽ������
 * @param width      ͼ���
 * @param height     ͼ���
 *
 *
 * @note
 *        1��YUV420P��ʽYUV��������СΪw * h * 3 / 2
 *        2��rgbbuffer��������ΪRGB���籣��BMP����Ҫ����ΪBGR
 */
void yuv420p_to_rgb24(unsigned char* yuvbuffer,unsigned char* rgbbuffer, int width, int height);

/** 
 * @brief YUV422SPתRGB24(�����)
 * 
 * @param yuvbuffer  YUV422SP��ʽ������
 * @param rgbbuffer  RGB24��ʽ������
 * @param width      ͼ���
 * @param height     ͼ���
 *
 *
 * @note
 *        1��YUV422SP��ʽYUV��������СΪw * h * 2
 *        2��rgbbuffer��������ΪRGB���籣��BMP����Ҫ����ΪBGR
 */
void yuv422sp_to_rgb24(unsigned char* yuvbuffer,unsigned char* rgbbuffer, int width, int height);

// TODO���������һ���ֵ
typedef enum
{
    YUV420P = 0xffff0001,
    YUV422P = 0xffff0002,
	YUV422SP = 0xfff0003,
    YUV422= 0xfff0004,
}YUV_TYPE;

/** 
 * @brief YUVתRGB24(�����)
 * 
 * @param type       YUV��ʽ����
 * @param yuvbuffer  YUV��ʽ������
 * @param rgbbuffer  RGB24��ʽ������
 * @param width      ͼ���
 * @param height     ͼ���
 *
 * @return 0: OK -1: failed
 *
 * @note
 *        1��YUV422 buffer: w * h * 2 YUV420 buffer: w * h * 3 / 2
 *        2��rgbbuffer��������ΪRGB���籣��BMP����Ҫ����ΪBGR
 */
int yuv_to_rgb24(YUV_TYPE type, unsigned char* yuvbuffer,unsigned char* rgbbuffer, int width, int height);

////////////////////////////////////////////////////////////////////////////////////////////////////
// todo
void yuv422sp_to_yuv422p(unsigned char* yuv422sp, unsigned char* yuv422p, int width, int height);

void yuv420sp_to_yuv420p(unsigned char* yuv420sp, unsigned char* yuv420p, int width, int height);


/////////////////////////////////////////////////

void yuv420_to_rgb24_1(unsigned char* yuv420, unsigned char* rgb, int width, int height);

void yuv420_to_rgb24_2(unsigned char *yuv420, unsigned char *rgb24, int width, int height) ;

void yuv420_to_rgb24_3(unsigned char* yuv, unsigned char* rgb, int width, int height);

#ifdef __cplusplus
}
#endif

#endif /* _YUV2RGB_H_ */