/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2023-03-16 02:27:44
 * @LastEditors: moxiaosang_vec@.163.com moxiaosang_vec@163.com
 * @LastEditTime: 2023-09-03 23:07:05
 * @FilePath: /ESKF-LCGINS/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <fstream>
#include <string.h>
#include "common.h"
#include "gi_lib.h"


#define     DEBUG_MODE

#define     BUFF_LEN            (1024)

// int decode_buff(const char* buff, const char sep, const int max_data_num, double* data)
// {
//     int len = strlen(buff);
//     int j = 0;
//     for (size_t i = 0; i < len; i++)
//     {
//         /* code */
//         if (buff[i] == sep || buff[i] == '\n')
//         {
//             sscanf(buff, "%lf", data + j);

//         }
//     }
    
// }

void decode_imu(FILE *fpin_imu, gins_imu_t &imu)
{
    char buff[BUFF_LEN] = { 0 };
    memset(buff, 0, strlen(buff));
    fgets(buff, BUFF_LEN, fpin_imu);
    sscanf(buff, "%lf %lf %lf %lf %lf %lf %lf", 
        &imu.time_stamp, 
        &imu.gyro[0], &imu.gyro[1], &imu.gyro[2], 
        &imu.acc[0], &imu.acc[1], &imu.acc[2]);  
    for (size_t i = 0; i < 3; i++)
    {
        imu.gyro[i] /= 0.005;
        imu.acc[i]  /= 0.005;
    }

}

void decode_gpos(FILE *fpin_gpos, gins_gpos_t &gpos)
{
    char buff[BUFF_LEN] = { 0 };
    memset(buff, 0, strlen(buff));
    fgets(buff, BUFF_LEN, fpin_gpos);
    sscanf(buff, "%lf %lf %lf %lf %lf %lf %lf", 
            &gpos.time_stamp, 
            &gpos.blh[0], &gpos.blh[1], &gpos.blh[2], 
            &gpos.std[0], &gpos.std[1], &gpos.std[2]);
}


int main(int argc, char** argv) {
    const char* path_root = "../dataset/";
    char path_imu[128] = { 0 };
    char path_gnss[128] = { 0 };
    char path_result[128] = { 0 };
    char buff[BUFF_LEN] = { 0 };
    FILE *fpin_imu = NULL;
    FILE *fpin_gnss = NULL;
    FILE *fpout_result = NULL;
    double time_stamp = 0.0;
    status_t ret = GILIB_STATUS_OK;
    syslog_t log = { 0 };
    gins_imu_t imu = { 0 };
    gins_gpos_t gpos = { 0 };
    GILib gilib;
    navstate_t nav;
    strcpy(path_imu, path_root);
    strcat(path_imu, "Leador-A15.txt");
    strcpy(path_gnss, path_root);
    strcat(path_gnss, "GNSS-RTK.txt");
    strcpy(path_result, path_root);
    strcat(path_result, "result.txt");
    
    fpin_imu = fopen(path_imu, "r");
    fpin_gnss = fopen(path_gnss, "r");
    fpout_result = fopen(path_result, "w");
    if (fpin_imu == NULL || fpin_gnss == NULL || fpout_result == NULL)
    {
        printf("[open file error]: \n");
        return -1;
    }

    /* 初始化 */
    if (!feof(fpin_imu))
    {
        decode_imu(fpin_imu, imu);
    }
    if (!feof(fpin_gnss))
    {
        decode_gpos(fpin_gnss, gpos);
    }
    
    // 3. 循环
    while (!feof(fpin_imu))
    {
        if (imu.time_stamp < gpos.time_stamp)
        {
            // printf("log imu\n");
            log.sys_time = imu.time_stamp;
            log.prctype  = prctype_imu;
            ret = gilib.process(prctype_imu, &imu, sizeof(imu_t), NULL, 0);
            decode_imu(fpin_imu, imu);
        }
        else
        {
            // printf("log gpos\n");
            log.sys_time = gpos.time_stamp;
            log.prctype  = prctype_gpos;
            ret = gilib.process(prctype_gpos, &gpos, sizeof(gnss_pos_t), NULL, 0);
            decode_gpos(fpin_gnss, gpos);
        }

        ret = gilib.return_nav(nav);
        if (TRUE == nav.flag && nav.time_stamp > 1e-6)
        {
            fprintf(fpout_result, "%.3lf", nav.time_stamp);
            fprintf(fpout_result, "   %.10lf %.10lf %.10lf", nav.blh[0], nav.blh[1], nav.blh[2]);
            fprintf(fpout_result, "   %.5lf %.5lf %.5lf", nav.vel[0], nav.vel[1], nav.vel[2]);
            fprintf(fpout_result, "   %.5lf %.5lf %.5lf", nav.att[0], nav.att[1], nav.att[2]);
            fprintf(fpout_result, "\n");
        }
        #ifdef DEBUG_MODE
            // printf("syslog: [time]%.3lf, [type]:%d, ", log.sys_time, (int)log.prctype);
            // printf("%.3lf, ", nav.time_stamp);
            // printf("%.12lf, %.12lf, %.5lf, ", nav.blh[0], nav.blh[1], nav.blh[2]);
            // printf("%.5lf, %.5lf, %.5lf, ", nav.vel[0], nav.vel[1], nav.vel[2]);
            // printf("\n");
        #endif
    }

    //
    fclose(fpin_imu);
    fclose(fpin_gnss);
    fclose(fpout_result);
    return 0;

}
